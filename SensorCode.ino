#define HallEffect 3 //Hall Effect digital pin 3
#define TempProbe A2 //Temp probe analog pin A2
#define MAGNETS 4 //Number of magnets attached to the axle for the Hall Effect sensor
#define RPM_INT 1000 // Period of time (milliseconds) over which RPM (Revolutions per Minute) is calculated 

volatile int pulseCount = 0;
unsigned long lastUpdateTime = 0;
float rpm;
float temp;

void setup() {
  pinMode(HallEffect, INPUT);
  attachInterrupt(digitalPinToInterrupt(HallEffect), countPulse, RISING);
  Serial.begin(9600);
}

//Temperature and Hall Effect Sensor
void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastUpdateTime >= RPM_INT) {
    noInterrupts();
    //Access pulse count and then reset it to 0
    int count = pulseCount;
    pulseCount = 0;
    interrupts();

    //Calculate RPM - count pulses per second, divide by MAGNETS attached to axle
    rpm = (float)count / MAGNETS * 60;

    //Read the voltage from the temperature probe
    temp = analogRead(TempProbe);
    //Convert analog voltage to Celsius
    temp = temp * 0.48828125;

    lastUpdateTime = currentMillis;
  }
}

//Count Hall effect sensor pulses
void countPulse() {
  pulseCount++;
}