#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

// Pines del Joystick 
const int pinX = 32;        // GPIO32 - Eje X
const int pinY = 33;        // GPIO33 - Eje Y
const int pinZ = 34;        // GPIO34 - Eje Z 
const int botonReset = 13;  // Cambiado de 25 a 13 - Botón de calibración

// Definir pines para botones
#define I2C_SDA 21
#define I2C_SCL 22
#define BTN_UP 26    // Cambiado de 27 a 26
#define BTN_DOWN 27  // Cambiado de 26 a 27
#define BTN_ENTER 14 // Cambiado de 25 a 14
#define BTN_BACK 25  // Cambiado de 14 a 25

// Umbral para la Dead Zone
const int DEAD_ZONE = 9;    // Valores dentro de ±3 se considerarán como 0

// Variables globales para sumas y factor de acumulación
float SumaX = 0;
float SumaY = 0;
float SumaZ = 0;
const float factorAcumulacion = 2000.0;

// Contadores de movimientos
int contadorX = 0;
int contadorY = 0;

// Variables para rastrear el estado
bool estadoX = false;
bool estadoY = false;

// Variables para la calibración
int offsetX = 0;
int offsetY = 0;
int offsetZ = 0;

unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 1000; // 1000ms = 1 segundo

// Variables para el debounce del botón de reset
unsigned long ultimaPulsacionReset = 0;
const unsigned long debounceDelay = 50;  // Tiempo para evitar rebotes (200 ms)

// Variables para calibración inicial
bool calibracionInicialCompletada = false;
unsigned long tiempoInicioCalib = 0;
const unsigned long TIEMPO_ESPERA_CALIBRACION = 500; // 500ms de espera antes de calibrar

// Estructura para los datos del joystick
struct JoystickData {
    int velocidadX;
    int velocidadY;
    int velocidadZ;
    uint8_t modo;  // 1 = modo libre, 2 = modo recorrido, etc.
};

// Variables para almacenar valores mapeados del joystick
int valorXMapeado = 0;
int valorYMapeado = 0;
int valorZMapeado = 0;

#define EEPROM_SIZE 12  // 4 bytes por variable float (3 variables)

// Direcciones EEPROM para cada variable
#define ADDR_EXPOSURE 0
#define ADDR_TOTAL_TIME 4
#define ADDR_INTERVAL 8

// Límites para tiempo de exposición (en segundos)
const float EXPOSURE_MIN = 0.002;
const float EXPOSURE_MAX = 60.0;
const float EXPOSURE_DEFAULT = 1.0;
const float EXPOSURE_INCREMENT = 0.5;

// Límites para tiempo total (en segundos)
const int TOTAL_TIME_MIN = 10;
const int TOTAL_TIME_MAX = 7200;
const int TOTAL_TIME_DEFAULT = 60;
const int TOTAL_TIME_INCREMENT = 10;

// Límites para intervalo (en segundos)
const float INTERVAL_MIN = 0.5;
const float INTERVAL_MAX = 60.0;
const float INTERVAL_DEFAULT = 1.0;
const float INTERVAL_INCREMENT = 0.5;

// Límites para posiciones
const int POS_X_MIN = 0;       // Posición mínima del rail en mm
const int POS_X_MAX = 1000;    // Posición máxima del rail en mm (1 metro)
const int POS_X_INCREMENT = 10; // Incrementos de 10mm

const int ROT_Y_MIN = -90;     // Rotación Y mínima en grados
const int ROT_Y_MAX = 90;      // Rotación Y máxima en grados
const int ROT_Y_INCREMENT = 5;  // Incrementos de 5 grados

const int ROT_Z_MIN = -90;     // Rotación Z mínima en grados
const int ROT_Z_MAX = 90;      // Rotación Z máxima en grados
const int ROT_Z_INCREMENT = 5;  // Incrementos de 5 grados

// Definición de valores de obturación
const int shutterDenominators[] = {1, 2, 4, 8, 15, 30, 60, 125, 250, 500};
const int numShutterValues = 10;
int currentShutterIndex = 0;  // Índice para saber en qué valor estamos

// Variables de configuración
float exposureTime = EXPOSURE_DEFAULT;
int totalTime = TOTAL_TIME_DEFAULT;
float interval = INTERVAL_DEFAULT;
int posX = 0;
int rotY = 0;
int rotZ = 0;

// Estados del menú
enum MenuState {
    MENU_MAIN,
    MENU_CAMERA,
    MENU_TIMELAPSE,
    MENU_TOTAL_TIME,
    MENU_INTERVAL,
    MENU_POSITION,
    MENU_POSITION_HOME,
    MENU_POSITION_PATH,
    MENU_POSITION_FREE,
    MENU_POSITION_X,
    MENU_POSITION_Y,
    MENU_POSITION_Z,
    MENU_REVIEW
};

MenuState currentState = MENU_MAIN;
int currentSelection = 0;

// Estructura para los botones
struct Button {
    const uint8_t PIN;
    bool lastState;
    unsigned long lastDebounceTime;
    bool pressed;
};

Button buttonUp = {BTN_UP, false, 0, false};
Button buttonDown = {BTN_DOWN, false, 0, false};
Button buttonEnter = {BTN_ENTER, false, 0, false};
Button buttonBack = {BTN_BACK, false, 0, false};

// Objeto LCD
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Función para calcular el tiempo total mínimo basado en el tiempo de exposición
int calculateMinTotalTime() {
    // Calculamos 5 veces el tiempo de exposición en segundos
    int minTime = exposureTime * 5;
    // Retornamos el mayor entre el mínimo calculado y TOTAL_TIME_MIN
    return max(minTime, TOTAL_TIME_MIN);
}

// Función para calcular el intervalo mínimo basado en el tiempo de exposición
float calculateMinInterval() {
    // Tiempo de exposición + 1 segundo de estabilización
    return exposureTime + 1.0;
}

// Función para calcular el intervalo máximo
float calculateMaxInterval() {
    // El mínimo entre 60s y tiempo_total/2
    return min(60.0, totalTime/2.0);
}


void loadSettings() {
    exposureTime = EEPROM.readFloat(ADDR_EXPOSURE);
    Serial.print("Loaded exposureTime: ");
    Serial.println(exposureTime);

    if (isnan(exposureTime)) exposureTime = EXPOSURE_DEFAULT;
    
    totalTime = EEPROM.readInt(ADDR_TOTAL_TIME);
    if (totalTime < TOTAL_TIME_MIN || totalTime > TOTAL_TIME_MAX) 
        totalTime = TOTAL_TIME_DEFAULT;
    
    interval = EEPROM.readFloat(ADDR_INTERVAL);
    if (isnan(interval)) interval = INTERVAL_DEFAULT;
}

// Función para formatear tiempo
void formatTime(int seconds, char* buffer) {
    if(seconds < 60) {
        sprintf(buffer, "%d seg", seconds);
    }
    else if(seconds < 3600) {
        int minutes = seconds / 60;
        int secs = seconds % 60;
        sprintf(buffer, "%dm %ds", minutes, secs);
    }
    else {
        int hours = seconds / 3600;
        int minutes = (seconds % 3600) / 60;
        sprintf(buffer, "%dh %dm", hours, minutes);
    }
}

void setupLCD() {
    Wire.begin(I2C_SDA, I2C_SCL);
    lcd.begin(20, 4);
    lcd.backlight();
    lcd.clear();
}

void setupButtons() {
    pinMode(buttonUp.PIN, INPUT_PULLDOWN);
    pinMode(buttonDown.PIN, INPUT_PULLDOWN);
    pinMode(buttonEnter.PIN, INPUT_PULLDOWN);
    pinMode(buttonBack.PIN, INPUT_PULLDOWN);
}

bool readButton(Button &button) {
    bool currentState = digitalRead(button.PIN);
    bool buttonPressed = false;
    
    if (currentState != button.lastState) {
        button.lastDebounceTime = millis();
    }
    
    if ((millis() - button.lastDebounceTime) > debounceDelay) {
        if (currentState == HIGH && !button.pressed) {
            buttonPressed = true;
            button.pressed = true;
        } 
        else if (currentState == LOW) {
            button.pressed = false;
        }
    }
    
    button.lastState = currentState;
    return buttonPressed;
}

void showMainMenu() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MENU TIMELAPSE");
    lcd.setCursor(0, 1);
    lcd.print(currentSelection == 0 ? "> " : "  ");
    lcd.print("Config. Camara");
    lcd.setCursor(0, 2);
    lcd.print(currentSelection == 1 ? "> " : "  ");
    lcd.print("Config. Timelapse");
    lcd.setCursor(0, 3);
    lcd.print(currentSelection == 2 ? "> " : "  ");
    lcd.print("Posicionar Gimbal");
}

void showCameraMenu() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("CONFIG. CAMARA");
    lcd.setCursor(0, 1);
    lcd.print("T.Exposicion:");
    lcd.setCursor(0, 2);
    
    if(exposureTime < 1.0) {
        lcd.print("1/");
        for(int i = 0; i < numShutterValues; i++) {
            if(abs(exposureTime - 1.0/shutterDenominators[i]) < 0.0001) {
                lcd.print(shutterDenominators[i]);
                break;
            }
        }
        lcd.print("s");
    } else {
        lcd.print(exposureTime, 1);
        lcd.print("s");
    }
    
    lcd.setCursor(0, 3);
    lcd.print("UP/DOWN para ajustar");
}

void showTimeLapseMenu() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("CONFIG. TIMELAPSE");
    lcd.setCursor(0, 1);
    lcd.print(currentSelection == 0 ? "> " : "  ");
    lcd.print("Tiempo Total");
    lcd.setCursor(0, 2);
    lcd.print(currentSelection == 1 ? "> " : "  ");
    lcd.print("Intervalo Fotos");
    lcd.setCursor(0, 3);
    lcd.print("ENTER para ajustar");
}

void showIntervalMenu() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("CONFIG. INTERVALO");
    lcd.setCursor(0, 1);
    lcd.print("Tiempo entre fotos:");
    
    lcd.setCursor(0, 2);
    lcd.print(interval, 1);
    lcd.print("s");
    
    lcd.setCursor(0, 3);
    lcd.print("UP/DOWN para ajustar");
}

void adjustExposureTime(bool increase) {
    if(exposureTime <= 1.0) {
        for(int i = 0; i < numShutterValues; i++) {
            if(abs(exposureTime - 1.0/shutterDenominators[i]) < 0.0001) {
                currentShutterIndex = i;
                break;
            }
        }
        
        if(increase) {
            if(currentShutterIndex > 0) {
                currentShutterIndex--;
                exposureTime = 1.0/shutterDenominators[currentShutterIndex];
            } else {
                exposureTime = 2.0;
            }
        } else {
            if(currentShutterIndex < numShutterValues - 1) {
                currentShutterIndex++;
                exposureTime = 1.0/shutterDenominators[currentShutterIndex];
            }
        }
    } 
    else {
        if(increase) {
            if(exposureTime < EXPOSURE_MAX) {
                if(exposureTime < 30.0) {
                    exposureTime += 1.0;
                } else {
                    exposureTime += 5.0;
                }
                if(exposureTime > EXPOSURE_MAX) {
                    exposureTime = EXPOSURE_MAX;
                }
            }
        } else {
            if(exposureTime > 30.0) {
                exposureTime -= 5.0;
            } else if(exposureTime > 1.0) {
                exposureTime -= 1.0;
            } else {
                exposureTime = 1.0/shutterDenominators[0];
            }
        }
    }
    showCameraMenu();
    
    // Actualizar el tiempo total si es necesario
    int minRequired = calculateMinTotalTime();
    if(totalTime < minRequired) {
        totalTime = minRequired;
    }
    
    // Actualizar el intervalo si es necesario
    float minIntervalRequired = calculateMinInterval();
    if(interval < minIntervalRequired) {
        interval = minIntervalRequired;
    }
}

void showTotalTimeMenu() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("TIEMPO TOTAL");
    lcd.setCursor(0, 1);
    
    char timeBuffer[20];
    formatTime(totalTime, timeBuffer);
    
    lcd.setCursor(0, 2);
    lcd.print(timeBuffer);
    
    lcd.setCursor(0, 3);
    lcd.print("UP/DOWN para ajustar");
}

void adjustTotalTime(bool increase) {
    int minRequired = calculateMinTotalTime();
    
    if(increase) {
        if(totalTime < TOTAL_TIME_MAX) {
            if(totalTime < 60) {
                totalTime += 5;
            }
            else if(totalTime < 600) {
                totalTime += 30;
            }
            else if(totalTime < 3600) {
                totalTime += 300;
            }
            else {
                totalTime += 900;
            }

            if(totalTime > TOTAL_TIME_MAX) {
                totalTime = TOTAL_TIME_MAX;
            }
        }
    } else {
        if(totalTime > minRequired) {
            if(totalTime <= 60) {
                totalTime -= 5;
            }
            else if(totalTime <= 600) {
                totalTime -= 30;
            }
            else if(totalTime <= 3600) {
                totalTime -= 300;
            }
            else {
                totalTime -= 900;
            }

            if(totalTime < minRequired) {
                totalTime = minRequired;
            }
        }
    }
    
    showTotalTimeMenu();  // Cambiado de showTimeLapseMenu a showTotalTimeMenu
    
    // Actualizar el intervalo si es necesario
    float maxIntervalAllowed = calculateMaxInterval();
    if(interval > maxIntervalAllowed) {
        interval = maxIntervalAllowed;
    }
}

void adjustInterval(bool increase) {
    float minInterval = calculateMinInterval();
    float maxInterval = calculateMaxInterval();
    
    if(increase) {
        if(interval < maxInterval) {
            if(interval < 10.0) {
                interval += 0.5;  // Incrementos de 0.5s hasta 10s
            }
            else if(interval < 30.0) {
                interval += 1.0;  // Incrementos de 1s hasta 30s
            }
            else {
                interval += 5.0;  // Incrementos de 5s después de 30s
            }

            if(interval > maxInterval) {
                interval = maxInterval;
            }
        }
    } else {
        if(interval > minInterval) {
            if(interval <= 10.0) {
                interval -= 0.5;
            }
            else if(interval <= 30.0) {
                interval -= 1.0;
            }
            else {
                interval -= 5.0;
            }

            if(interval < minInterval) {
                interval = minInterval;
            }
        }
    }
    showIntervalMenu();
}

void showPositionXMenu() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("POSICION RAIL (X)");
    lcd.setCursor(0, 1);
    lcd.print("Posicion actual:");
    lcd.setCursor(0, 2);
    lcd.print(posX);
    lcd.print("mm");
    lcd.setCursor(0, 3);
    lcd.print("UP/DOWN para ajustar");
}

void showPositionYMenu() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ROTACION (Y)");
    lcd.setCursor(0, 1);
    lcd.print("Angulo actual:");
    lcd.setCursor(0, 2);
    lcd.print(rotY);
    lcd.print(" grados");
    lcd.setCursor(0, 3);
    lcd.print("UP/DOWN para ajustar");
}

void showPositionZMenu() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ROTACION (Z)");
    lcd.setCursor(0, 1);
    lcd.print("Angulo actual:");
    lcd.setCursor(0, 2);
    lcd.print(rotZ);
    lcd.print(" grados");
    lcd.setCursor(0, 3);
    lcd.print("UP/DOWN para ajustar");
}

void adjustPositionX(bool increase) {
    if(increase) {
        if(posX < POS_X_MAX) {
            posX += POS_X_INCREMENT;
            if(posX > POS_X_MAX) posX = POS_X_MAX;
        }
    } else {
        if(posX > POS_X_MIN) {
            posX -= POS_X_INCREMENT;
            if(posX < POS_X_MIN) posX = POS_X_MIN;
        }
    }
    showPositionXMenu();
}

void adjustPositionY(bool increase) {
    if(increase) {
        if(rotY < ROT_Y_MAX) {
            rotY += ROT_Y_INCREMENT;
            if(rotY > ROT_Y_MAX) rotY = ROT_Y_MAX;
        }
    } else {
        if(rotY > ROT_Y_MIN) {
            rotY -= ROT_Y_INCREMENT;
            if(rotY < ROT_Y_MIN) rotY = ROT_Y_MIN;
        }
    }
    showPositionYMenu();
}

void adjustPositionZ(bool increase) {
    if(increase) {
        if(rotZ < ROT_Z_MAX) {
            rotZ += ROT_Z_INCREMENT;
            if(rotZ > ROT_Z_MAX) rotZ = ROT_Z_MAX;
        }
    } else {
        if(rotZ > ROT_Z_MIN) {
            rotZ -= ROT_Z_INCREMENT;
            if(rotZ < ROT_Z_MIN) rotZ = ROT_Z_MIN;
        }
    }
    showPositionZMenu();
}

void showPositionMenu() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("POSICIONAR GIMBAL");
    lcd.setCursor(0, 1);
    lcd.print(currentSelection == 0 ? "> " : "  ");
    lcd.print("Posicion 0,0,0");
    lcd.setCursor(0, 2);
    lcd.print(currentSelection == 1 ? "> " : "  ");
    lcd.print("Crear Recorrido");
    lcd.setCursor(0, 3);
    lcd.print(currentSelection == 2 ? "> " : "  ");
    lcd.print("Movimiento Libre");
}

void showPositionFreeMenu() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MOVIMIENTO LIBRE");
    lcd.setCursor(0, 1);
    lcd.print(currentSelection == 0 ? "> " : "  ");
    lcd.print("Rail (X)");
    lcd.setCursor(0, 2);
    lcd.print(currentSelection == 1 ? "> " : "  ");
    lcd.print("Rotacion Y");
    lcd.setCursor(0, 3);
    lcd.print(currentSelection == 2 ? "> " : "  ");
    lcd.print("Rotacion Z");
}

// Función de calibración que se usará tanto en setup como con el botón
void calibrarJoystick() {

   SumaX = 0;
   SumaY = 0;
   SumaZ = 0;

   int valorX = analogRead(pinX);
   int valorY = analogRead(pinY);
   int valorZ = analogRead(pinZ);
   
   offsetX = -map(valorX, 0, 4095, -100, 100);
   offsetY = map(valorY, 0, 4095, -100, 100);
   offsetZ = map(valorZ, 0, 4095, -100, 100);
   
   
   contadorX = 0;
   contadorY = 0;
   
   Serial.println("Calibración completada:");
   Serial.print("Offsets -> X: "); Serial.print(offsetX);
   Serial.print(" Y: "); Serial.print(offsetY);
   Serial.print(" Z: "); Serial.println(offsetZ);
}

void goToHomePosition() {
    // Actualizamos los offsets para que la posición actual sea (0,0,0)
    offsetX += valorXMapeado;  // El valor actual se convertirá en 0
    offsetY += valorYMapeado;  // El valor actual se convertirá en 0
    offsetZ += valorZMapeado;  // El valor actual se convertirá en 0
    
    // Reseteamos las sumas acumuladas
    SumaX = 0;
    SumaY = 0;
    SumaZ = 0;
    
    Serial.println("Posición actual establecida como Home (0,0,0)");
}


void saveSettings() {
    EEPROM.writeFloat(ADDR_EXPOSURE, exposureTime);
    EEPROM.writeInt(ADDR_TOTAL_TIME, totalTime);
    EEPROM.writeFloat(ADDR_INTERVAL, interval);
    EEPROM.commit();

}

void resetAll() {
   calibrarJoystick();
   exposureTime = EXPOSURE_DEFAULT;
   totalTime = TOTAL_TIME_DEFAULT;
   interval = INTERVAL_DEFAULT;
   saveSettings();
   Serial.println("¡Valores de configuración reseteados!");
}
// Función joystickGimbal modificada
void joystickGimbal() {
    int valorX = analogRead(pinX);
    int valorY = analogRead(pinY);
    int valorZ = analogRead(pinZ);

    valorXMapeado = -map(valorX, 0, 4095, -100, 100) - offsetX;
    valorYMapeado = -(map(valorY, 0, 4095, -100, 100) - offsetY);
    valorZMapeado = (map(valorZ, 0, 4095, -100, 100) - offsetZ);

    if (abs(valorXMapeado) <= DEAD_ZONE) valorXMapeado = 0;
    if (abs(valorYMapeado) <= DEAD_ZONE) valorYMapeado = 0;
    if (abs(valorZMapeado) <= DEAD_ZONE) valorZMapeado = 0;

    // Calibración manual con botón
    if (digitalRead(botonReset) == HIGH) {
        if (millis() - ultimaPulsacionReset > debounceDelay) {
            ultimaPulsacionReset = millis();
            resetAll();
           
        }
    }

    float factorX = valorXMapeado / 100.0;
    float factorY = valorYMapeado / 100.0;
    float factorZ = valorZMapeado / 100.0;


   /*
    // Mantener el comportamiento existente para las sumas
    SumaX += valorXMapeado / factorAcumulacion;
    SumaY += valorYMapeado / factorAcumulacion;
    SumaZ += valorZMapeado / factorAcumulacion;
    */

    // Acumulación más suave
    SumaX += (factorX * factorX * factorX) * 100.0 / factorAcumulacion;
    SumaY += (factorY * factorY * factorY) * 100.0 / factorAcumulacion;
    SumaZ += (factorZ * factorZ * factorZ) * 100.0 / factorAcumulacion;

    SumaX = constrain(SumaX, -100, 100);
    SumaY = constrain(SumaY, -100, 100);
    SumaZ = constrain(SumaZ, -100, 100);

}

void printSystemStatus() {
    // Valores X agrupados
    Serial.print("offX: ");
    Serial.print(offsetX);
    Serial.print(" / valorX: ");
    Serial.print(analogRead(pinX));
    Serial.print(" / valorXMap: ");
    Serial.print(valorXMapeado);
    Serial.print(" / SumaX: ");
    Serial.print(SumaX, 1);
    
    // Valores Y agrupados
    Serial.print(" / offY: ");
    Serial.print(offsetY);
    Serial.print(" / valorY: ");
    Serial.print(analogRead(pinY));
    Serial.print(" / valorYMap: ");
    Serial.print(valorYMapeado);
    Serial.print(" / SumaY: ");
    Serial.print(SumaY, 1);
    
    // Valores Z agrupados
    Serial.print(" / offZ: ");
    Serial.print(offsetZ);
    Serial.print(" / valorZ: ");
    Serial.print(analogRead(pinZ));
    Serial.print(" / valorZMap: ");
    Serial.print(valorZMapeado);
    Serial.print(" / SumaZ: ");
    Serial.print(SumaZ, 1);
    
    // Valores de posición
    Serial.print(" / posX: ");
    Serial.print(posX);
    Serial.print("mm / rotY: ");
    Serial.print(rotY);
    Serial.print("° / rotZ: ");
    Serial.print(rotZ);
    Serial.println("°");
}
void setup() {

    Serial.begin(115200);
    EEPROM.begin(EEPROM_SIZE);
    loadSettings();  // Carga los valores guardados
    pinMode(pinX, INPUT_PULLDOWN);  // Configura pin X con resistencia pull-down interna
    pinMode(pinY, INPUT_PULLDOWN);  // Configura pin Y con resistencia pull-down interna
    pinMode(pinZ, INPUT_PULLDOWN);  // Configura pin Z con resistencia pull-down interna
    setupLCD();
    setupButtons();


    pinMode(botonReset, INPUT_PULLDOWN);  // Configurar el botón con pull-down
    tiempoInicioCalib = millis();  // Iniciar el temporizador de calibración
    
    // Asegurarnos que el tiempo total inicial cumple con el mínimo requerido
    int minRequired = calculateMinTotalTime();
    if(totalTime < minRequired) {
        totalTime = minRequired;
    }
    
    // Asegurarnos que el intervalo inicial cumple con los requisitos
    float minInterval = calculateMinInterval();
    float maxInterval = calculateMaxInterval();
    if(interval < minInterval) {
        interval = minInterval;
    } else if(interval > maxInterval) {
        interval = maxInterval;
    }
    
    showMainMenu();
}

void loop() {
// Manejo de la calibración inicial
  if (!calibracionInicialCompletada) {
       if (millis() - tiempoInicioCalib >= TIEMPO_ESPERA_CALIBRACION) {
           calibrarJoystick();
           Serial.println("Calibración inicial completada");
           calibracionInicialCompletada = true;
       }
       return; // Esperar a que se complete la calibración inicial
  }
  joystickGimbal();


  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
      printSystemStatus();
      lastPrintTime = currentTime;
  } 



    if (currentState == MENU_MAIN) {
        if (readButton(buttonUp)) {
            if (currentSelection > 0) {
                currentSelection--;
                showMainMenu();
            }
        }
        
        if (readButton(buttonDown)) {    
            if (currentSelection < 2) {
                currentSelection++;
                showMainMenu();
            }
        }
        
        if (readButton(buttonEnter)) {
            switch(currentSelection) {
                case 0:
                    currentState = MENU_CAMERA;
                    showCameraMenu();
                    break;
                case 1:
                    currentState = MENU_TIMELAPSE;
                    currentSelection = 0;
                    showTimeLapseMenu();
                    break;
                case 2:
                    currentState = MENU_POSITION;
                    currentSelection = 0;
                    showPositionMenu();
                    break;
            }
        }
    }
    else if (currentState == MENU_CAMERA) {
        if (readButton(buttonUp)) {
            adjustExposureTime(true);
        }
        
        if (readButton(buttonDown)) {
            adjustExposureTime(false);
        }
        
        if (readButton(buttonEnter)) {
            saveSettings();  // Guarda al avanzar
            currentState = MENU_TIMELAPSE;
            currentSelection = 0;
            showTimeLapseMenu();
        }
        
        if (readButton(buttonBack)) {
            saveSettings();  // Guarda al retroceder
            currentState = MENU_MAIN;
            showMainMenu();
        }
    }
    else if (currentState == MENU_TIMELAPSE) {
        if (readButton(buttonUp)) {
            if (currentSelection > 0) {
                currentSelection--;
                showTimeLapseMenu();
            }
        }
        
        if (readButton(buttonDown)) {
            if (currentSelection < 1) {
                currentSelection++;
                showTimeLapseMenu();
            }
        }
        
        if (readButton(buttonEnter)) {
            if (currentSelection == 0) {
                currentState = MENU_TOTAL_TIME;
                showTotalTimeMenu();
            } else {
                currentState = MENU_INTERVAL;
                showIntervalMenu();
            }
        }
        
        if (readButton(buttonBack)) {
            currentState = MENU_MAIN;
            currentSelection = 0;
            showMainMenu();
        }
    }
    else if (currentState == MENU_TOTAL_TIME) {
        if (readButton(buttonUp)) {
            adjustTotalTime(true);
        }
        
        if (readButton(buttonDown)) {
            adjustTotalTime(false);
        }
        
        if (readButton(buttonEnter)) {
            saveSettings();
            currentState = MENU_TIMELAPSE;
            showTimeLapseMenu();
        }
        
        if (readButton(buttonBack)) {
            saveSettings();
            currentState = MENU_TIMELAPSE;
            showTimeLapseMenu();
        }
    }
    else if (currentState == MENU_INTERVAL) {
        if (readButton(buttonUp)) {
            adjustInterval(true);
        }
        
        if (readButton(buttonDown)) {
            adjustInterval(false);
        }
        
        if (readButton(buttonEnter)) {
            saveSettings();
            currentState = MENU_TIMELAPSE;
            showTimeLapseMenu();
        }
        
        if (readButton(buttonBack)) {
            saveSettings();
            currentState = MENU_TIMELAPSE;
            showTimeLapseMenu();
        }
    }
    else if (currentState == MENU_POSITION) {
        if (readButton(buttonUp)) {
            if (currentSelection > 0) {
                currentSelection--;
                showPositionMenu();
            }
        }
        
        if (readButton(buttonDown)) {
            if (currentSelection < 2) {
                currentSelection++;
                showPositionMenu();
            }
        }
        
        if (readButton(buttonEnter)) {
            switch(currentSelection) {
                case 0:
                    currentState = MENU_POSITION_HOME;
                    goToHomePosition();  // Llamamos a la función
            
                    // Después de establecer el home, volvemos al menú de posición
                    currentState = MENU_POSITION;
                    currentSelection = 0;
                    showPositionMenu();
                    break;
                case 1:
                    currentState = MENU_POSITION_PATH;
                    // Aquí irá la función para crear rnew_ecorrido
                    // showPathCreationMenu();
                    break;
                case 2:
                    currentState = MENU_POSITION_FREE;
                    currentSelection = 0;
                    showPositionFreeMenu();
                    break;
            }
        }
        
        if (readButton(buttonBack)) {
            currentState = MENU_MAIN;
            currentSelection = 0;
            showMainMenu();
        }
    }
    else if (currentState == MENU_POSITION_FREE) {
        if (readButton(buttonUp)) {
            if (currentSelection > 0) {
                currentSelection--;
                showPositionFreeMenu();
            }
        }
        
        if (readButton(buttonDown)) {
            if (currentSelection < 2) {
                currentSelection++;
                showPositionFreeMenu();
            }
        }
        
        if (readButton(buttonEnter)) {
            switch(currentSelection) {
                case 0:
                    currentState = MENU_POSITION_X;
                    showPositionXMenu();
                    break;
                case 1:
                    currentState = MENU_POSITION_Y;
                    showPositionYMenu();
                    break;
                case 2:
                    currentState = MENU_POSITION_Z;
                    showPositionZMenu();
                    break;
            }
        }
        
        if (readButton(buttonBack)) {
            currentState = MENU_POSITION;
            currentSelection = 0;
            showPositionMenu();
        }
    }
    else if (currentState == MENU_POSITION_X) {
        if (readButton(buttonUp)) {
            adjustPositionX(true);
        }
        
        if (readButton(buttonDown)) {
            adjustPositionX(false);
        }
        
        if (readButton(buttonEnter) || readButton(buttonBack)) {
            currentState = MENU_POSITION_FREE;
            showPositionFreeMenu();
        }
    }
    else if (currentState == MENU_POSITION_Y) {
        if (readButton(buttonUp)) {
            adjustPositionY(true);
        }
        
        if (readButton(buttonDown)) {
            adjustPositionY(false);
        }
        
        if (readButton(buttonEnter) || readButton(buttonBack)) {
            currentState = MENU_POSITION_FREE;
            showPositionFreeMenu();
        }
    }
    else if (currentState == MENU_POSITION_Z) {
        if (readButton(buttonUp)) {
            adjustPositionZ(true);
        }
        
        if (readButton(buttonDown)) {
            adjustPositionZ(false);
        }
        
        if (readButton(buttonEnter) || readButton(buttonBack)) {
            currentState = MENU_POSITION_FREE;
            showPositionFreeMenu();
        }
    }
 
}