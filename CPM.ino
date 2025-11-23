// Bibliotecas:
#include <AccelStepper.h>
#include <WiFi.h> 

// --- Definiciones de Pines para ESP32 ---
// Driver TB6600
const int STEP_PIN = 2;
const int DIR_PIN = 4;
const int ENABLE_PIN = 16;

// PINES DE ENTRADA
const int ENDSTOP_PIN = 22;  // Fin de carrera
const int BUTTON_PIN = 21;   // Botón de control/parada

// LEDS
const int LED_DETENIDO_PIN = 5;   
const int LED_FUNCIONANDO_PIN = 17; 

// SENSOR ANGULO
const int POT_PIN = 34; 

//////////////////////////////////////////
// --- Configuración de la red WiFi --- //
//////////////////////////////////////////
const char* ssid = "CPM-Fisiocoach";
const char* password = "tfg-cpm2025";

WiFiServer server(80);
String header;

// --- Variables para el Control (Web) ---
int userTargetAngle = 0;       
int selectionValue = 15;       
String controlState = "Detenido"; 

// --- Variables de TIEMPO DE TERAPIA ---
unsigned long sessionDurationMs = 15 * 60 * 1000UL; 
unsigned long sessionStartTime = 0; 
long remainingTimeSeconds = 15 * 60; 

// --- Variables de tiempo (para el timeout del cliente) ---
unsigned long currentTime = millis();
unsigned long previousTime = 0;
const long timeoutTime = 200; // Reducido para mejorar respuesta del motor

//////////////////////////////////////////

// ------------------------ CONSTANTES PID ------------------------
// Kp: Fuerza de reacción inmediata (más alto = más rápido, pero puede oscilar)
// Ki: Corrige errores pequeños acumulados (cuidado, puede causar "windup")
// Kd: Amortigua el movimiento (evita que se pase de largo)
const float Kp = 150.0;
const float Ki = 0.5;
const float Kd = 50.0;

float pidIntegral = 0;
float pidLastError = 0;
unsigned long lastPidTime = 0;
const int PID_SAMPLE_TIME = 20; // El PID se calcula cada 20ms

// Objetivo Dinámico del PID
int dynamicSetPoint = 0; 
// Zona muerta (histéresis)
const int PID_DEADBAND = 1; 

// Temporizador de validación ---
unsigned long arrivalStartTime = 0;
const unsigned long TARGET_HOLD_TIME = 500; // 0.5 segundos de espera

// Filtro
float filteredAngle = 0.0; 
const float FILTER_ALPHA = 0.05; // Factor de suavizado (0.1 = muy suave, 1.0 = sin filtro)

// Temporizador para lectura de sensor (para no bloquear el motor)
unsigned long lastSensorReadTime = 0;
const int SENSOR_READ_INTERVAL = 10; // Leer sensor cada 10ms

////////////////////////////////////////////////////////////////////////

// --- Rango de salida ---
const int MIN_ANGLE_DEGREES = 0;
const int MAX_ANGLE_DEGREES = 130;

// --- VALORES DE CALIBRACIÓN SENSOR ---
const int ADC_VALUE_AT_0_DEGREES = 1930;
const int ADC_VALUE_AT_130_DEGREES = 460;
const int ZERO_OFFSET_DEGREES = 0;

// --- Configuración Motor ---
const float VELOCIDAD_MAXIMA = 8000.0; // 2500 velocidad recomendada para terapia. Agregar en el GUI capacidad de ajustar velocidad
const float VELOCIDAD_HOMING = 5000.0;

// --- LEDs ---
const unsigned long BLINK_INTERVAL_MS = 250;
unsigned long lastBlinkTime = 0;
bool ledToggleState = false;

// --- Objetos y Variables de Estado ---
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

enum MotorState {
    IDLE,        
    HOMING,      
    RUNNING,     
    STOPPED_EMG  
};
MotorState currentState = IDLE;
MotorState previousStateBeforeStop = IDLE; 
bool hasHomed = false; 

// Botón
unsigned long buttonPressStartTime = 0;
const long LONG_PRESS_DURATION_MS = 2500; 
int runDirection = 1; 

// Variables globales
int currentAngle = 0;

// ----------------------------------------------------------------------
// FUNCIÓN DE CONTROL DE LEDS
// ----------------------------------------------------------------------
void updateLEDs() {
    if (millis() - lastBlinkTime >= BLINK_INTERVAL_MS) {
        ledToggleState = !ledToggleState; 
        lastBlinkTime = millis();
    }

    switch (currentState) {
        case RUNNING:
            digitalWrite(LED_DETENIDO_PIN, LOW);
            digitalWrite(LED_FUNCIONANDO_PIN, HIGH);
            if (arrivalStartTime != 0) controlState = "Manteniendo posición...";
            else controlState = "En movimiento";
            break;
        case HOMING:
            digitalWrite(LED_DETENIDO_PIN, ledToggleState ? LOW : HIGH);      
            digitalWrite(LED_FUNCIONANDO_PIN, ledToggleState ? HIGH : LOW);  
            controlState = "Homing";
            break;
        case IDLE:
            controlState = "Esperando indicación...";
            if (!hasHomed) {
                digitalWrite(LED_DETENIDO_PIN, HIGH);
                digitalWrite(LED_FUNCIONANDO_PIN, LOW);
            } else {
                int ledStatus = ledToggleState ? HIGH : LOW;
                digitalWrite(LED_DETENIDO_PIN, ledStatus);
                digitalWrite(LED_FUNCIONANDO_PIN, ledStatus);
            }
            break;
        case STOPPED_EMG:
            digitalWrite(LED_DETENIDO_PIN, HIGH);
            digitalWrite(LED_FUNCIONANDO_PIN, LOW);
            controlState = "PARADA EMERGENCIA";
            break;
    }
}

// ----------------------------------------------------------------------
// FUNCIÓN DE LECTURA DE ÁNGULO (CON FILTRO)
// ----------------------------------------------------------------------
int read_mapAngle() {
    long sum = 0;
    const int SAMPLES = 10;
    
    for(int i=0; i < SAMPLES; i++) {
        sum += analogRead(POT_PIN);
    }
    
    int rawValue = sum / SAMPLES; // Promedio bruto
    
    // --- Mapeo ---
    long angle_mapped = map(rawValue, 
                            ADC_VALUE_AT_0_DEGREES, 
                            ADC_VALUE_AT_130_DEGREES, 
                            (long)MIN_ANGLE_DEGREES, 
                            (long)MAX_ANGLE_DEGREES); 
    
    // --- Filtro EMA (Suavizado final) ---
    if (filteredAngle == 0.0 && angle_mapped > 0) filteredAngle = angle_mapped;
    filteredAngle = (FILTER_ALPHA * angle_mapped) + ((1.0 - FILTER_ALPHA) * filteredAngle);

    int angle_final = (int)filteredAngle - ZERO_OFFSET_DEGREES;
    
    if (angle_final < MIN_ANGLE_DEGREES) angle_final = MIN_ANGLE_DEGREES;
    else if (angle_final > MAX_ANGLE_DEGREES) angle_final = MAX_ANGLE_DEGREES;

    return angle_final;
}
// ----------------------------------------------------------------------
// LÓGICA PID
// ----------------------------------------------------------------------
void computePID() {
    unsigned long now = millis();
    if (now - lastPidTime >= PID_SAMPLE_TIME) {
        
        float error = (float)(dynamicSetPoint - currentAngle);
        
        // 1. Comprobar si se llegó a la meta
        if (abs(error) <= PID_DEADBAND) {
            error = 0; // Anular error para no oscilar
            
            // --- LOGICA DE ESPERA DE 2 SEGUNDOS ---
            if (arrivalStartTime == 0) {
                arrivalStartTime = millis(); // Empezar a contar tiempo
            }
            
            // Verificar si ya pasaron 2 segundos manteniendo la posición
            if (millis() - arrivalStartTime >= TARGET_HOLD_TIME) {
                // Tiempo cumplido: Cambiar dirección
                if (dynamicSetPoint == 0) {
                    dynamicSetPoint = userTargetAngle; 
                } else {
                    dynamicSetPoint = 0; 
                }
                pidIntegral = 0; 
                arrivalStartTime = 0; // Resetear timer para el próximo viaje
            }
        } else {
            // Si salimos de la zona muerta (por ruido o movimiento), reiniciamos el timer
            // Esto asegura que debe ser estable por 2 segundos CONTINUOS
            if (abs(error) > (PID_DEADBAND + 1)) { // Pequeña histéresis extra
                 arrivalStartTime = 0; 
            }
        }

        // 2. Calcular PID
        pidIntegral += (error * (PID_SAMPLE_TIME / 1000.0));
        if (pidIntegral > 3000) pidIntegral = 3000;
        if (pidIntegral < -3000) pidIntegral = -3000;

        float derivative = (error - pidLastError) / (PID_SAMPLE_TIME / 1000.0);
        float outputSpeed = (Kp * error) + (Ki * pidIntegral) + (Kd * derivative);

        if (outputSpeed > VELOCIDAD_MAXIMA) outputSpeed = VELOCIDAD_MAXIMA;
        if (outputSpeed < -VELOCIDAD_MAXIMA) outputSpeed = -VELOCIDAD_MAXIMA;

        // Zona muerta de motor
        if (abs(outputSpeed) < 50) outputSpeed = 0;

        // Si estamos esperando (arrivalStartTime > 0), forzamos frenado suave
        // para mantener posición sin pelear
        if (arrivalStartTime != 0) {
           // outputSpeed = 0; // Opcional: Descomentar si se quiere apagar fuerza
           // Mantenemos el PID activo para que "sostenga" la pierna
        }

        stepper.setSpeed(outputSpeed);
        pidLastError = error;
        lastPidTime = now;
    }
}

void resetPID() {
    pidIntegral = 0;
    pidLastError = 0;
    arrivalStartTime = 0;
    if (currentAngle < 10) dynamicSetPoint = userTargetAngle;
    else dynamicSetPoint = 0;
}

// ----------------------------------------------------------------------
// SETUP
// ----------------------------------------------------------------------
void setup() {
    Serial.begin(115200);

    // IP Estática
    IPAddress localIP(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(localIP, gateway, subnet);

    Serial.print("Iniciando AP...");
    if (WiFi.softAP(ssid, password)) Serial.println(" OK");
    else Serial.println(" FALLO");
    
    Serial.println("IP Servidor: " + WiFi.softAPIP().toString());
    server.begin();

    analogReadResolution(12);
    pinMode(POT_PIN, INPUT);
    
    // Pre-cargar el filtro con una lectura inicial para evitar salto al inicio
    int initialRead = analogRead(POT_PIN);
    filteredAngle = map(initialRead, ADC_VALUE_AT_0_DEGREES, ADC_VALUE_AT_130_DEGREES, MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES);

    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW); 

    pinMode(ENDSTOP_PIN, INPUT_PULLUP);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    
    pinMode(LED_DETENIDO_PIN, OUTPUT);
    pinMode(LED_FUNCIONANDO_PIN, OUTPUT);
    
    stepper.setMaxSpeed(VELOCIDAD_MAXIMA);
    stepper.setAcceleration(0); 
    stepper.setCurrentPosition(0); 
    
    remainingTimeSeconds = selectionValue * 60;
    currentState = HOMING;
}

const unsigned long SERIAL_INTERVAL_MS = 200;
unsigned long lastSerialTime = 0;

// ----------------------------------------------------------------------
// LOOP
// ----------------------------------------------------------------------
void loop() {
    // 1. Motor (PID)
    if (currentState == RUNNING) {
        computePID();      
        stepper.runSpeed(); 
    }

    // 2. Lecturas
    if (millis() - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
        currentAngle = read_mapAngle();
        updateLEDs();
        lastSensorReadTime = millis();
    }

    // 3. Lógica de seguridad en RETORNO (switch manda sobre sensor)
    bool endStopActive = (digitalRead(ENDSTOP_PIN) == LOW);
    
    if (currentState == RUNNING && dynamicSetPoint == 0 && endStopActive) {
        Serial.println("⚠️ Switch activado en bajada. Forzando llegada a 0.");
        
        currentAngle = 0; 
        filteredAngle = 0; // Reset filtro
        
        if (arrivalStartTime == 0) arrivalStartTime = millis();
        
        stepper.setSpeed(0); 
    }
    
    // 4. Gestión de Tiempo
    if (currentState == RUNNING) {
        unsigned long elapsed = millis() - sessionStartTime;
        if (elapsed < sessionDurationMs) {
            remainingTimeSeconds = (sessionDurationMs - elapsed) / 1000;
        } else {
            Serial.println("⛔ Tiempo finalizado. Volviendo a inicio.");
            currentState = HOMING;
            stepper.setCurrentPosition(0);
            remainingTimeSeconds = 0;
        }
    } else {
        remainingTimeSeconds = selectionValue * 60; 
    }

    // Debug para graficar
    if (millis() - lastSerialTime >= SERIAL_INTERVAL_MS) {
      Serial.print("Meta:");      // Etiqueta para la leyenda (opcional en versiones nuevas IDE)
      Serial.print(dynamicSetPoint);
      Serial.print(",");          // Coma separadora
      Serial.print("Medido:");    // Etiqueta
      Serial.println(currentAngle); // Salto de línea final
      
      lastSerialTime = millis();
    }

    bool buttonPressed = (digitalRead(BUTTON_PIN) == LOW);   
    
    // 5. Botón Físico
    if (buttonPressed) {
        if (buttonPressStartTime == 0) buttonPressStartTime = millis(); 

        if (currentState == RUNNING || currentState == HOMING) {
            stepper.stop(); 
            previousStateBeforeStop = currentState; 
            currentState = STOPPED_EMG;
            buttonPressStartTime = 0; 
        } 
    } else {
        if (buttonPressStartTime != 0) {
            unsigned long pressDuration = millis() - buttonPressStartTime;
            if (pressDuration < LONG_PRESS_DURATION_MS) {
                if (currentState == IDLE && hasHomed) {
                    resetPID(); 
                    currentState = RUNNING;
                    sessionDurationMs = (unsigned long)selectionValue * 60 * 1000UL;
                    sessionStartTime = millis();
                } 
                else if (currentState == STOPPED_EMG) {
                    currentState = previousStateBeforeStop;
                    resetPID(); 
                }
            } else {
                if (currentState == IDLE || currentState == STOPPED_EMG) {
                    currentState = HOMING; 
                    stepper.setCurrentPosition(0); 
                }
            }
            buttonPressStartTime = 0; 
        }
    }

    // 6. Homing
    switch (currentState) {
        case HOMING:
            if (!endStopActive) {
                stepper.setSpeed(-VELOCIDAD_HOMING); 
                stepper.runSpeed(); 
            } else {
                stepper.setCurrentPosition(0); 
                runDirection = 1; 
                hasHomed = true;
                currentState = IDLE; 
            }
            break;
        default: break;
    }

    // 7. Web
    WiFiClient client = server.available();

    if (client) {
      currentTime = millis();
      previousTime = currentTime;
      String currentLine = "";
      
      while (client.connected() && currentTime - previousTime <= timeoutTime) {
        currentTime = millis();
        
        if (currentState == RUNNING) {
            computePID();
            stepper.runSpeed();
        }
        if (currentState == HOMING && !endStopActive) stepper.runSpeed();

        if (client.available()) {
          char c = client.read();
          header += c;
          if (c == '\n') {
            if (currentLine.length() == 0) {
              
              if (header.indexOf("GET /status") >= 0) {
                  int m = remainingTimeSeconds / 60;
                  int s = remainingTimeSeconds % 60;
                  client.println("HTTP/1.1 200 OK");
                  client.println("Content-type:application/json");
                  client.println("Connection: close");
                  client.println();
                  client.print("{\"a\":"); client.print(currentAngle); 
                  client.print(",\"s\":\""); client.print(controlState);
                  client.print("\",\"m\":"); client.print(m);
                  client.print(",\"t\":"); client.print(s);
                  client.print("}");
                  break; 
              }

              if (header.indexOf("GET /update?") >= 0) {
                  int newTargetAngle = userTargetAngle;
                  int anglePos = header.indexOf("angle=");
                  if (anglePos > 0) {
                      String s = header.substring(anglePos + 6);
                      int end = s.indexOf('&');
                      if(end > 0) s = s.substring(0, end);
                      newTargetAngle = s.toInt();
                      if (newTargetAngle > 130) newTargetAngle = 130; 
                  }
                  int selPos = header.indexOf("selection=");
                  if (selPos > 0) {
                      String s = header.substring(selPos + 10);
                      int end = s.indexOf('&');
                      if(end > 0) s = s.substring(0, end);
                      selectionValue = s.toInt();
                  }
                  userTargetAngle = newTargetAngle;
                  if (currentState == RUNNING && dynamicSetPoint != 0) dynamicSetPoint = userTargetAngle;
                  
                  sessionDurationMs = (unsigned long)selectionValue * 60 * 1000UL;
                  if (currentState != RUNNING) remainingTimeSeconds = selectionValue * 60;
              } 
              else if (header.indexOf("GET /start") >= 0) {
                if (currentState == IDLE && hasHomed) {
                    resetPID();
                    currentState = RUNNING;
                    sessionDurationMs = (unsigned long)selectionValue * 60 * 1000UL;
                    sessionStartTime = millis(); 
                } else if (currentState == STOPPED_EMG) {
                    currentState = previousStateBeforeStop;
                    resetPID();
                }
              } 
              else if (header.indexOf("GET /stop") >= 0) {
                if (currentState == RUNNING || currentState == HOMING) {
                    stepper.stop();
                    previousStateBeforeStop = currentState;
                    currentState = STOPPED_EMG;
                }
              }              
              // HTML
              client.println("HTTP/1.1 200 OK");
              client.println("Content-type:text/html");
              client.println("Connection: close");
              client.println();
  
              client.println("<!DOCTYPE html><html>");
              client.println("<head><meta charset=\"UTF-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
              client.println("<link rel=\"icon\" href=\"data:,\">");
              
              client.println("<style>");
              client.println("html { font-family: 'Segoe UI', Arial, sans-serif; display: flex; justify-content: center; align-items: flex-start; min-height: 100vh; background-color: #F8F9FA;}");  
              client.println("body { margin-top: 2px; color: #343A40; width: 100%; max-width: 550px; text-align: center; }");  
              client.println("h1 { color: #007BFF; margin-bottom: 2px; }");
              client.println("h2, h3 { color: #007BFF; margin-top: 20px; border-bottom: 2px solid #E9ECEF; padding-bottom: 5px;}");  
              client.println(".container { width: 100%; background-color: #FFFFFF; padding: 15px; border-radius: 12px; box-shadow: 0 4px 8px rgba(0,0,0,0.08); }");
              client.println(".config-row { display: flex; flex-direction: row; justify-content: space-around; gap: 10px; margin-bottom: 5px; }");
              client.println(".config-item.angle { flex-basis: 60%; min-width: 55%; }");
              client.println(".config-item.time { flex-basis: 40%; min-width: 40%; }");  
              client.println("label { display: block; margin-bottom: 5px; font-weight: bold; color: #555; text-align: center;}");  
              client.println(".slider-container { background-color: #E9ECEF; padding: 10px 15px; border-radius: 8px; margin-top: 5px; }");  
              client.println(".slider { width: 100%; height: 10px; border-radius: 5px; background: #D0D0D0; outline: none; -webkit-appearance: none; appearance: none; }");
              client.println("#angleDisplay { font-size: 4.0em; font-weight: bold; color: #FF7F00; margin: 5px 0; display: block; }");  
              client.println(".radio-group { display: flex; flex-direction: column; align-items: flex-start; margin-top: 10px; }");  
              client.println(".radio-item { margin-bottom: 5px; display: flex; align-items: center; }");
              client.println(".radio-item input[type='radio'] { width: 1em; height: 1em; margin: 0; flex-shrink: 0; }");
              client.println(".button-container { margin-top: 15px; display: flex; justify-content: center; gap: 20px; }");  
              client.println(".button { padding: 15px 30px; font-size: 1.2em; border-radius: 8px; cursor: pointer; color: #FFF; border: none; }");
              client.println(".button-start { background-color: #007BFF; } .button-stop { background-color: #FF7F00; }");
              client.println(".button-submit { background-color: #6C757D; margin-top: 15px; width: 75%; padding: 12px; font-size: 1.2em; color: #FFF; border: none; border-radius: 8px;}");  
              client.println(".status-card { background-color: #E9ECEF; padding: 15px; border-radius: 10px; margin-top: 15px; text-align: center; border-left: 5px solid #007BFF; }");  
              client.println("</style>");
              
              client.println("<script>");
              client.println("function updateSliderDisplay(val) { document.getElementById('angleValue').innerHTML = val; }");
              client.println("setInterval(function() {");
              client.println("  fetch('/status').then(res => res.json()).then(data => {");
              client.println("    document.getElementById('realAngleStatus').innerText = data.a;"); 
              client.println("    document.getElementById('stateStatus').innerText = data.s;"); 
              client.println("    let m = String(data.m).padStart(2, '0');");
              client.println("    let s = String(data.t).padStart(2, '0');");
              client.println("    document.getElementById('timeStatus').innerText = m + ':' + s;");
              client.println("  });");
              client.println("}, 500);"); 
              client.println("</script>");
              client.println("</head>");
  
              client.println("<body>");
              client.println("<div class=\"container\">");  
              client.println("<h1>CPM Rodilla</h1>");  
              client.println("<h2>Configuración</h2>");
              client.println("<form action=\"/update\" method=\"GET\">");
              client.println("<div class=\"config-row\">");
              client.println("<div class=\"config-item angle\"><label><strong>ÁNGULO META:</strong></label><div class=\"slider-container\"><span id=\"angleDisplay\"><span id=\"angleValue\">" + String(userTargetAngle) + "</span>°</span><input type=\"range\" name=\"angle\" min=\"0\" max=\"130\" value=\"" + String(userTargetAngle) + "\" class=\"slider\" oninput=\"updateSliderDisplay(this.value)\"></div></div>");  
              client.println("<div class=\"config-item time\"><label><strong>TIEMPO:</strong></label><div class=\"radio-group\">");
              
              int opts[] = {15, 20, 25, 30};
              for(int i=0; i<4; i++) {
                  String checked = (selectionValue == opts[i]) ? " checked" : "";
                  client.println("<div class=\"radio-item\"><input type=\"radio\" name=\"selection\" value=\""+String(opts[i])+"\""+checked+"><label>"+String(opts[i])+" min</label></div>");
              }
              client.println("</div></div></div>"); 
              client.println("<input type=\"submit\" value=\"Confirmar valores\" class=\"button-submit\"></form>");
              
              client.println("<div class=\"button-container\">");
              client.println("<a href=\"/start\"><button class=\"button button-start\"><strong>Iniciar</strong></button></a>");
              client.println("<a href=\"/stop\"><button class=\"button button-stop\"><strong>Detener</strong></button></a>");
              client.println("</div>");
  
              client.println("<br><h2>Estado</h2>");
              client.println("<div class=\"status-card\">");
              client.println("<p><strong>Ángulo medido:</strong> <span id=\"realAngleStatus\">" + String(currentAngle) + "</span>°</p>");
              
              int mDisplay = remainingTimeSeconds / 60;
              int sDisplay = remainingTimeSeconds % 60;
              String tFormatted = (mDisplay < 10 ? "0" : "") + String(mDisplay) + ":" + (sDisplay < 10 ? "0" : "") + String(sDisplay);
              
              client.println("<p><strong>Tiempo restante:</strong> <span id=\"timeStatus\">" + tFormatted + "</span></p>");
              client.println("<p><strong>Estado:</strong> <span id=\"stateStatus\">" + controlState + "</span></p>");
              client.println("</div></div></body></html>");
  
              client.println();
              break;
            } else currentLine = "";
          } else if (c != '\r') currentLine += c;
        }
      }
      header = "";
      client.stop();
    }
}
