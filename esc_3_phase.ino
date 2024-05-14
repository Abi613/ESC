int HIN_A = 3;   // PWM pin for High-side input for Phase A
int EN_A = 5;   // PWM pin for Low-side input for Phase A
int HIN_B = 6;   // PWM pin for High-side input for Phase B
int EN_B = 9;   // PWM pin for Low-side input for Phase B
int HIN_C = 10;  // PWM pin for High-side input for Phase C
int EN_C = 11;  // PWM pin for Low-side input for Phase C

int enable = 2;
int emfA = A0;
int emfB = A1;
int emfC = A2;

int phase = 1;

int IN = A3;
int tiempo = 4000;

unsigned long previousMillis = 0;

void setup() {
  Serial.begin(250000);

  pinMode(HIN_A, OUTPUT);
  pinMode(EN_A, OUTPUT);
  pinMode(HIN_B, OUTPUT);
  pinMode(EN_B, OUTPUT);
  pinMode(HIN_C, OUTPUT);
  pinMode(EN_C, OUTPUT);

  pinMode(enable, OUTPUT);

  pinMode(IN, INPUT);
  
  digitalWrite(enable, HIGH);
  digitalWrite(EN_A, HIGH);
  digitalWrite(EN_B, HIGH);
  digitalWrite(EN_C, HIGH);

  previousMillis = millis();
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= tiempo) {
    previousMillis += tiempo;

    // Phase switching logic
    switch (phase) {
      case 1:
        analogWrite(HIN_A, 255);  // Full duty cycle
        digitalWrite(HIN_A, LOW);
        digitalWrite(HIN_B, LOW);
        digitalWrite(HIN_B, LOW);
        analogWrite(HIN_C, 255);  // Full duty cycle
        digitalWrite(HIN_C, LOW);
        break;

      case 2:
        digitalWrite(HIN_A, LOW);
        digitalWrite(HIN_A, LOW);
        analogWrite(HIN_B, 255);  // Full duty cycle
        digitalWrite(HIN_B, LOW);
        digitalWrite(HIN_C, LOW);
        digitalWrite(HIN_C, LOW);
        break;

      case 3:
        digitalWrite(HIN_A, LOW);
        digitalWrite(HIN_A, HIGH);
        analogWrite(HIN_B, 255);  // Full duty cycle
        digitalWrite(HIN_B, LOW);
        digitalWrite(HIN_C, LOW);
        digitalWrite(HIN_C, LOW);
        break;

      case 4:
        digitalWrite(HIN_A, LOW);
        digitalWrite(HIN_A, HIGH);
        digitalWrite(HIN_B, LOW);
        digitalWrite(HIN_B, LOW);
        analogWrite(HIN_C, 255);  // Full duty cycle
        digitalWrite(HIN_C, LOW);
        break;

      case 5:
        digitalWrite(HIN_A, LOW);
        digitalWrite(HIN_A, LOW);
        digitalWrite(HIN_B, LOW);
        analogWrite(HIN_B, 255);  // Full duty cycle
        analogWrite(HIN_C, 255);  // Full duty cycle
        digitalWrite(HIN_C, LOW); 
        break;

      case 6:
        analogWrite(HIN_A, 255);  // Full duty cycle
        digitalWrite(HIN_A, LOW);
        digitalWrite(HIN_B, LOW);
        analogWrite(HIN_B, 255);  // Full duty cycle
        digitalWrite(HIN_C, LOW);
        digitalWrite(HIN_C, LOW);
        break;
    }

    // Read analog input to adjust switching time
    int t = analogRead(IN);
    tiempo = map(t, 0, 1023, 1, 1000); // Adjust the mapping range according to your requirements

    // Increment phase counter
    if (phase < 6) {
      phase++;
    } else {
      phase = 1;
    }
  }
}
