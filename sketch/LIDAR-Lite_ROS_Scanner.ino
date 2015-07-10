/* Arduino Mega LIDAR-Lite Scanner Shield */

#include <TimerOne.h>
#include <I2C.h>

char LIDARLite_ADDRESS = 0x62;

int reading = 0;
long dT;
long tNew, tOld;  // time in milliseconds()

int pin_led = 13;
int pin_step = 11;
int pin_dir = 10;
int pin_lidar_power = 8;

int motorPos;

volatile int motorStepCnt = 0;
int stepsPerRevolution = 1599; //value-1 due to 0 index
void stepmotor( void )
{ 
  digitalWrite(pin_step, HIGH);
  if ( ++motorStepCnt > stepsPerRevolution )
  {
    motorStepCnt = 0;
  }
  digitalWrite(pin_step, LOW);
}

float secondsPerDegree = 0;
int lastMotorPos = -1;
int lastSpeed = 0;
void calculate_speed( int p_position, int p_interval )
{
  int degreesTraveled;
  if ( p_position > lastMotorPos )
  {
   degreesTraveled = p_position - lastMotorPos; 
  }
  else
  {
    degreesTraveled = p_position - lastMotorPos + 360;
  }
  lastMotorPos = p_position;
  float secsPerDeg = ( p_interval / 1000.0/*milliseconds*/ ) / degreesTraveled;
  secondsPerDegree = ( 0.9 * secondsPerDegree ) + ( 0.1 * secsPerDeg );
}


void cycle_lidar_power(void)
{
   digitalWrite(pin_lidar_power, LOW);
   delay(1);
   digitalWrite(pin_lidar_power, HIGH);
   delay(1);
}

void setup()
{
  Serial.begin(115200);     
  I2c.begin();
  delay(100); 
  I2c.timeOut(50);

  Timer1.initialize(1100); // 1100 fast scanning, 3k is about every 1 to 2 degrees
  Timer1.attachInterrupt( stepmotor );
  
  pinMode(pin_led, OUTPUT);
  pinMode(pin_step, OUTPUT);
  pinMode(pin_dir, OUTPUT);
  pinMode(pin_lidar_power, OUTPUT);
  
  digitalWrite(pin_lidar_power, HIGH); //Turn lidar on
  
  digitalWrite(pin_dir,LOW); //Motor Step Direction
}

void loop()
{
    reading = llGetDistance();
    tNew = millis();
    dT = (tNew - tOld);
    if ( reading < 1024 && reading > 0 )
    {
        motorPos = (360.0 / stepsPerRevolution) * motorStepCnt;
        
        calculate_speed( motorPos, dT );
        
        Serial.print( motorPos );
        Serial.print(",");
        Serial.print(reading);
        Serial.print(",");
        Serial.print(secondsPerDegree,3);
        Serial.print(",");
        Serial.print(dT);
        Serial.println();
    }
    tOld=tNew;
    
    if ( dT > 100 )
    {
      cycle_lidar_power(); 
    }
}

void llWriteAndWait(char myAddress, char myValue)
{
  uint8_t nackack = 100;    
  while (nackack != 0)
  {
    nackack = I2c.write(LIDARLite_ADDRESS,myAddress, myValue);
    
    delay(2); // Wait 2 ms to prevent overpolling
  }
}

byte llReadAndWait(char myAddress, int numOfBytes, byte arrayToSave[2]){
  uint8_t nackack = 100;   
  while (nackack != 0)
  {
    nackack = I2c.read(LIDARLite_ADDRESS,myAddress, numOfBytes, arrayToSave);
    delay(2); // Wait 2 ms to prevent overpolling
  }
  return arrayToSave[2];
}

void llSetAcqSpeed( void )
{
  llWriteAndWait(0x02,0x80); //0x80 is default
}

int llGetDistance()
{
  llWriteAndWait(0x00,0x04);
  byte myArray[2];
  llReadAndWait(0x8f,2,myArray);
  int distance = (myArray[0] << 8) + myArray[1];
  return(distance);
}


