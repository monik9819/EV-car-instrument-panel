// HEARTBEAT
const int PulseSensorPurplePin = 0;        // Pulse Sensor PURPLE WIRE connected to ANALOG PIN 0
const int Threshold = 550;            // Determine which Signal to "count as a beat", and which to ingore.
const int offset = 60;
const int min_heartbeat = 65;
const int max_heartbeat = 90;

// TEMPERATURE: BATTERY
const int temp_battery_pin = 1;
const int min_temp_battery = 30;
const int max_temp_battery = 50;

// TEMPERATURE: MOTOR
int temp_motor_pin = 2;
const int min_temp_motor = 25;
const int max_temp_motor = 50;

// ULTRASONIC 1
int trigPin1 = 7;    // Trigger
int echoPin1 = 6;    // Echo
const int min_ultrasonic = 0;
const int max_ultrasonic = 2;

// ULTRASONIC 2
int trigPin2 = 8;    // Trigger
int echoPin2 = 9;    // Echo

void setup() {
  Serial.begin(115200);         // Set's up Serial Communication at certain speed.

  // ULTRASONIC 1
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);

  // ULTRASONIC 2
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
}

void loop() {

  // HEARTBEAT
  int raw =  analogRead(PulseSensorPurplePin);
  int heartbeat_final = raw==0 ? 0 : raw + offset;
  //if( heartbeat_final>=min_heartbeat && heartbeat_final<=max_heartbeat)
  //Serial.println("S3 " + String(heartbeat_final) + "#");
  sendData(heartbeat_final, min_heartbeat, max_heartbeat, "S3");
  // delay(600);

  // TEMPERATURE: BATTERY
  raw = analogRead(temp_battery_pin);
  float cel = raw*0.48828 - 100; 
  /*if(cel>=25){
    Serial.println("S4 " + String(cel) + "#");
  }*/
  sendData_Float(cel, min_temp_battery, max_temp_battery, "S4");
  // delay(500);

  // TEMPERATURE: MOTOR
  raw = analogRead(temp_motor_pin);
  float rawHigh = 99.6,
        rawLow = 0.5,
        refHigh = 99.6,
        refLow = 0;
  float rawRange = rawHigh - rawLow;
  float refRange = refHigh - refLow;
  float callibratedValue = (((raw - rawLow)*refRange) / rawRange ) + refLow;
  float finalValue = callibratedValue - 148.0f;
  /*if(finalValue >= minTemp) {
    Serial.println("S5 " + String(finalValue) + "#") ;
//    delay(500);
  }*/
  sendData(finalValue, min_temp_motor, max_temp_motor, "S5");

  // ULTRASONIC 1
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigPin1, LOW);
 
  pinMode(echoPin1, INPUT);
  long duration = pulseIn(echoPin1, HIGH);
 
  long cm = (duration/2) / 29.1;    
  long feet = cm/ 30.48;
  //Serial.print("S1 " + String(feet) + "#");
  sendData(feet, min_ultrasonic, max_ultrasonic, "S1");

  // ULTRASONIC 2
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigPin2, LOW);
     
  pinMode(echoPin2, INPUT);
  duration = pulseIn(echoPin2, HIGH);
 
  cm = (duration/2) / 29.1;    
  feet = cm/ 30.48;
  // Serial.println("S2 " + String(feet) + "#"); 
  sendData(feet, min_ultrasonic, max_ultrasonic, "S2");

  delay(700);
}

void sendData_Float(float val, long min_val, long max_val, String prefix) {
  if(val<=min_val) val = min_val;
  else if(val>=max_val) val = max_val;
  Serial.println( prefix + " " + String(val) + "#" );
}

void sendData(long val, long min_val, long max_val, String prefix) {
  if(val<=min_val) val = min_val;
  else if(val>=max_val) val = max_val;
  Serial.println( prefix + " " + String(val) + "#" );
}

