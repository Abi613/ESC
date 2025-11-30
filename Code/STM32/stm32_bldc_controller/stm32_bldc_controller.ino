#define PWM_U       PA0
#define PWM_V       PA1
#define PWM_W       PA2

#define IN_U        PA3
#define IN_V        PA4
#define IN_W        PA5

#define HAL_U       PB5
#define HAL_V       PB6
#define HAL_W       PB7

#define VBATT       PA0
#define CUR_U       PA1
#define CUR_V       PA2
#define CUR_W       PA3

#define CWR         PA8

#define PWM_FREQUENCY   16000
#define PWM_RESOLUTION  12
#define PWM_MAX         4095

volatile uint8_t Hall_State = 0;

float BattVoltage = 0.0;
uint16_t VoltageRAW = 0;

uint16_t U_CurrentRAW = 0;
uint16_t V_CurrentRAW = 0;
uint16_t W_CurrentRAW = 0;

float U_Current = 0.0;
float V_Current = 0.0;
float W_Current = 0.0;

float Voltage_Coef = (55.0 / 4095.0);
float U_Current_Coef = (5.0 / 2048.0);
float V_Current_Coef = (5.0 / 2048.0);
float W_Current_Coef = (5.0 / 2048.0);

uint16_t PWM = 2475;

unsigned long previousMillis = 0;
int interval = 100;

void MotorRun(uint8_t SensorState);
uint8_t HallSensorsRead(void);
void HallSensorISR(void);

void setup() {
    Serial.begin(115200);
    
    pinMode(PWM_U, PWM);
    pinMode(PWM_V, PWM);
    pinMode(PWM_W, PWM);
    
    analogWriteFrequency(PWM_FREQUENCY);
    analogWriteResolution(PWM_RESOLUTION);
    
    pinMode(IN_U, OUTPUT);
    pinMode(IN_V, OUTPUT);
    pinMode(IN_W, OUTPUT);
    
    pinMode(CWR, OUTPUT);
    
    pinMode(HAL_U, INPUT_PULLUP);
    pinMode(HAL_V, INPUT_PULLUP);
    pinMode(HAL_W, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(HAL_U), HallSensorISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(HAL_V), HallSensorISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(HAL_W), HallSensorISR, CHANGE);
    
    pinMode(VBATT, INPUT_ANALOG);
    pinMode(CUR_U, INPUT_ANALOG);
    pinMode(CUR_V, INPUT_ANALOG);
    pinMode(CUR_W, INPUT_ANALOG);
    
    analogReadResolution(12);
    
    MotorRun(0);
    delay(1000);
    
    Hall_State = HallSensorsRead();
    MotorRun(Hall_State);
}

void loop() {
    VoltageRAW = analogRead(VBATT);
    U_CurrentRAW = analogRead(CUR_U);
    V_CurrentRAW = analogRead(CUR_V);
    W_CurrentRAW = analogRead(CUR_W);
    
    BattVoltage = VoltageRAW * Voltage_Coef;
    U_Current = (U_CurrentRAW - 2048) * U_Current_Coef;
    V_Current = (V_CurrentRAW - 2048) * V_Current_Coef;
    W_Current = (W_CurrentRAW - 2048) * W_Current_Coef;
    
    unsigned long currentMillis = millis();
    
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        
        Serial.print(U_Current, 2);
        Serial.print(",");
        Serial.print(V_Current, 2);
        Serial.print(",");
        Serial.print(W_Current, 2);
        Serial.print(",");
        Serial.print(Hall_State);
        Serial.print(",");
        Serial.println(BattVoltage, 2);
    }
}

void HallSensorISR(void) {
    Hall_State = HallSensorsRead();
    MotorRun(Hall_State);
}

uint8_t HallSensorsRead(void) {
    uint8_t state = 0;
    state |= (digitalRead(HAL_U) << 2);
    state |= (digitalRead(HAL_V) << 1);
    state |= digitalRead(HAL_W);
    return state;
}

void MotorRun(uint8_t SensorState) {
    switch (SensorState) {
        case 1:
            analogWrite(PWM_U, PWM); digitalWrite(IN_U, LOW);
            analogWrite(PWM_V, PWM); digitalWrite(IN_V, HIGH);
            analogWrite(PWM_W, 0);   digitalWrite(IN_W, LOW);
            break;
            
        case 2:
            analogWrite(PWM_U, 0);   digitalWrite(IN_U, LOW);
            analogWrite(PWM_V, PWM); digitalWrite(IN_V, LOW);
            analogWrite(PWM_W, PWM); digitalWrite(IN_W, HIGH);
            break;
            
        case 3:
            analogWrite(PWM_U, PWM); digitalWrite(IN_U, LOW);
            analogWrite(PWM_V, 0);   digitalWrite(IN_V, LOW);
            analogWrite(PWM_W, PWM); digitalWrite(IN_W, HIGH);
            break;
            
        case 4:
            analogWrite(PWM_U, PWM); digitalWrite(IN_U, HIGH);
            analogWrite(PWM_V, 0);   digitalWrite(IN_V, LOW);
            analogWrite(PWM_W, PWM); digitalWrite(IN_W, LOW);
            break;
            
        case 5:
            analogWrite(PWM_U, 0);   digitalWrite(IN_U, LOW);
            analogWrite(PWM_V, PWM); digitalWrite(IN_V, HIGH);
            analogWrite(PWM_W, PWM); digitalWrite(IN_W, LOW);
            break;
            
        case 6:
            analogWrite(PWM_U, PWM); digitalWrite(IN_U, HIGH);
            analogWrite(PWM_V, PWM); digitalWrite(IN_V, LOW);
            analogWrite(PWM_W, 0);   digitalWrite(IN_W, LOW);
            break;
            
        default:
            analogWrite(PWM_U, 0); digitalWrite(IN_U, LOW);
            analogWrite(PWM_V, 0); digitalWrite(IN_V, LOW);
            analogWrite(PWM_W, 0); digitalWrite(IN_W, LOW);
            Hall_State = HallSensorsRead();
            break;
    }
}