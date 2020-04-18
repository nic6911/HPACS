#include <OneWire.h>
#include <DallasTemperature.h>
#include <Nextion.h>//Remember to go into config.h for the library and change Serial1 to Serial1 !
#include <EEPROMex.h>

// Defines
#define ONE_WIRE_BUS 2
#define OFF 0
#define ON 1
#define AUTOTUNE 2
#define ABOVE 1
#define BELOW 0

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

//********* GLOBAL VARIABLES ****************//
float setPoint = 55;
float Tmeas=0;
float error=0;
float Ki = 0.002;
float Kp = 1.1860;
float Kd = 467.2840;
uint8_t ontime = 0;
uint8_t stateMachine = OFF;
bool initTuning = 0;
uint8_t tempLimit = 100;// We do not allow more than 100 degree temperatures

//********* SETUP OF NEXTION EVENTS ****************//
NexButton b0 = NexButton(0,3,"b0");//AUTOTUNE
NexButton b4 = NexButton(0,23,"b4");//ON
NexButton b1 = NexButton(0,24,"b1");//OFF
NexButton b5 = NexButton(0,25,"b5");//SAVE
NexNumber x2=NexNumber(0,6,"x2");
NexNumber x3=NexNumber(0,7,"x3");
NexNumber x4=NexNumber(0,31,"x4");
NexNumber x0=NexNumber(0,2,"x0");

NexTouch *nex_listen_list[] = 
{
  &b1,
  &b4,
  &b0,
  &b5,
  &x2,
  &x3,
  &x4,
  &x0,
  NULL
};

void b5PushCallback(void *ptr)
{
  b5.detachPush();  // Button press detached to avoid multiple presses to save
  uint32_t setPointMem = 0;
  uint32_t KpMem = 0;
  uint32_t KiMem = 0;
  uint32_t KdMem = 0;
  x2.getValue(&KpMem);
  Kp = (float)(KpMem*0.001);
  x3.getValue(&KiMem);
  Ki = (float)(KiMem*0.001);
  x4.getValue(&KdMem);
  Kd = (float)(KdMem*0.001);
  x0.getValue(&setPointMem);
  setPoint = (float)(setPointMem*0.1);
  //uint16_t KpMem = (uint16_t)(Kp*1000);
  Serial.println(KdMem);
  EEPROM.writeLong(0,KpMem);
  EEPROM.writeLong(4,KiMem);
  EEPROM.writeLong(8,KdMem);
  EEPROM.writeLong(12,setPointMem);
  Serial1.print("b5.pic=");Serial1.print(10);Serial1.write(0xff);Serial1.write(0xff);Serial1.write(0xff);
}
void b4PushCallback(void *ptr)
{
  uint32_t number2 = 0;
  stateMachine = ON;
  x0.getValue(&number2);
  setPoint = (float)(number2*0.1);
}

void b1PushCallback(void *ptr)
{
  stateMachine = OFF;
}

void b0PushCallback(void *ptr)
{
  uint32_t number2 = 0;
  stateMachine = AUTOTUNE;
  x0.getValue(&number2);
  setPoint = (float)(number2*0.1);
}

void x2PushCallback(void *ptr)
{
  b5.attachPush(b5PushCallback);// If someone may have eited a PID parameter we enable the save button again
}

void x3PushCallback(void *ptr)
{
  b5.attachPush(b5PushCallback);// If someone may have eited a PID parameter we enable the save button again
}

void x4PushCallback(void *ptr)
{
  b5.attachPush(b5PushCallback);// If someone may have eited a PID parameter we enable the save button again
}

void setup(){
  // Load PID and setpoint from EEPROM
  uint32_t KpMem = EEPROM.readLong(0);
  uint32_t KiMem = EEPROM.readLong(4);
  uint32_t KdMem = EEPROM.readLong(8);
  uint32_t setPointMem = EEPROM.readLong(12);
  Serial.begin(9600);
  Serial.println(KdMem);
  Kp = (float)KpMem*0.001;
  Ki = (float)KiMem*0.001;
  Kd = (float)KdMem*0.001;
  setPoint = (float)setPointMem*0.1;
  
  Serial1.begin(9600);
  pinMode(8, OUTPUT);
  digitalWrite(8,HIGH);
  sensors.begin();
  delay(1000);
  sensors.requestTemperatures(); // Send the command to get temperatures
  sensors.getTempCByIndex(0);
  //Send loaded values to display
  Serial1.print("x2.val=");Serial1.print(KpMem);Serial1.write(0xff);Serial1.write(0xff);Serial1.write(0xff);
  Serial1.print("x3.val=");Serial1.print(KiMem);Serial1.write(0xff);Serial1.write(0xff);Serial1.write(0xff);
  Serial1.print("x4.val=");Serial1.print(KdMem);Serial1.write(0xff);Serial1.write(0xff);Serial1.write(0xff);
  Serial1.print("x0.val=");Serial1.print(setPointMem);Serial1.write(0xff);Serial1.write(0xff);Serial1.write(0xff);
  Serial1.print("va1.val=");Serial1.print(0);Serial1.write(0xff);Serial1.write(0xff);Serial1.write(0xff);
  Serial1.print("va2.val=");Serial1.print(0);Serial1.write(0xff);Serial1.write(0xff);Serial1.write(0xff);
  Serial1.print("va3.val=");Serial1.print(0);Serial1.write(0xff);Serial1.write(0xff);Serial1.write(0xff);
  // Attach pushbuttons from nextion
  b4.attachPush(b4PushCallback);  // Button press
  b1.attachPush(b1PushCallback);  // Button press
  b0.attachPush(b0PushCallback);  // Button press
  b5.attachPush(b5PushCallback);  // Button press
  x2.attachPush(x2PushCallback);
  x3.attachPush(x3PushCallback);
  x4.attachPush(x4PushCallback);
  
  //Setup the PWM for the SS relay
cli();//stop interrupts
  DDRH |= (1 << DDH3);//D6 on arduino mega for PWM
  //set timer4 for PWM at 0.238Hz = 4.2seconds
  TCCR4A = 0;// set entire TCCR1A register to 0
  TCCR4B = 0;// same for TCCR1B
  TCNT4  = 0;//initialize counter value to 0
  // set TOP to 16bit
  ICR4 = 0xFFFF;
  //Dutycycle of the PWM
  OCR4A = 0x0000;//0xEFFF;//25% duty
  // set none-inverting mode
  TCCR4A |= (1 << COM4A1);
  // set Fast PWM mode using ICR1 as TOP
  TCCR4A |= (1 << WGM41);
  TCCR4B |= (1 << WGM42)|(1 << WGM43);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR4B |= (1 << CS42) | (1 << CS40);  

// Set timer5 interrupt at 1Hz to run the control
 TCCR5A = 0;// set entire TCCR1A register to 0
 TCCR5B = 0;// same for TCCR1B
 TCNT5  = 0;//initialize counter value to 0
 // set compare match register for 1hz increments
 OCR5A = 15625/1;// = (16*10^6) / (1*1024) - 1 (must be <65536) -> 15625
 // turn on CTC mode
 TCCR5B |= (1 << WGM52);
 // Set CS12 and CS10 bits for 1024 prescaler
 TCCR5B |= (1 << CS52) | (1 << CS50);  
 // enable timer compare interrupt
 TIMSK5 |= (1 << OCIE5A);  
sei();//allow interrupts
}//end setup

ISR(TIMER5_COMPA_vect){//timer5 interrupt 1Hz 
 sensors.requestTemperatures(); // Send the command to get temperatures
 Tmeas = sensors.getTempCByIndex(0);// Get temperature
 error = (setPoint-Tmeas);// Calculate error
 // State machine here
 if(stateMachine == ON)
 {
  ontime = Controller(error);
  initTuning = 1;
 }
 else if(stateMachine == OFF)
 {
  ontime = 0;
  initTuning = 1;
 }
 else if(stateMachine == AUTOTUNE)
 {
  tuning();
 }

 // Send temperature to display
 Serial1.print("x1.val=");Serial1.print((uint16_t)(Tmeas*10));Serial1.write(0xff);Serial1.write(0xff);Serial1.write(0xff);

 if(Tmeas>tempLimit)// We do not allow more than the tempLimit !
 {
  ontime = 0;
 }

 // Update PWM register
 OCR4A = map(ontime,0,100,0x0,0xFFFF);//Set PWM duty cycle
}

//********* CONTROLLER ****************//
uint8_t Controller(float error)
{
  float limit = 100.0;
  float y;
  static float yOld;
  static float integralOld;
  static float errorOld;
  static int i;
  float integral;
  float der;
  
  der=error-errorOld;// Do the derivative over two samples - could be done on more samples or on a filtered signal instead...
  if(i>1){
    errorOld = error;
    i=0;
  }
  i++;

  // Anti wind-up using clamping of the integrator
  if((yOld>limit && error > 0)|| (yOld<0 && error < 0))
  {
    integral = integralOld;
  }
  else
  {
    integral = integralOld + (error*Ki);
  }
 
  y = (integral + Kp*error + Kd*der);
  
  yOld = y;
  if(y<0)
  {
    y=0;
  }
  if(y>limit)
  {
    y=limit;
  }
  
  integralOld = integral;
  return (uint8_t)y;
}

//********* AUTOTUNE ****************//
void tuning()
{
  uint8_t cycles = 3;// How many proper cycles we want - can be put as an editable global
  float duty = 40;// How much dutycycle we want to do the tuning with - can be but as an editable global
  uint16_t timeVariation = 90;//Allowable cycle time variation in seconds - can be but as an editable global
  float tempVariation = 2.0;// Allowable variation of temprature between cycles  - can be but as an editable global
  uint32_t time1 = 0;// For calculating cycle time
  float Tu;// Ultimate period
  float Ku;// Ultimate gain
  // Variables below are required to be static
  static float deltaT[11];
  static uint32_t deltaTime[11];
  static uint8_t count = 0;
  static bool state = BELOW;
  static float Tmax = 0;// For recording the max temperature of a cycle
  static float Tmin = 255;// For recording the min temperature of a cycle
  static uint32_t time2 = 0;// For calculating cycle time
  static uint8_t k = 0;
  static bool tuneOver = 0;
  static bool bufFull = 0;
  static float avgDeltaT = 0;
  static uint8_t check = 0;
  static bool flag = 0;
  static float avgDeltaTime = 0;

  if(initTuning > 0)//reinitialize tuning parameters
  {
    count = 0;
    state = BELOW;
    Tmax = 0;
    Tmin = 255;
    time2 = 0;
    k = 0;
    tuneOver = 0;
    bufFull = 0;
    avgDeltaT = 0;
    check = 0;
    flag = 0;
    avgDeltaTime = 0;
    initTuning = 0;
    if(error>0)
    {
      state = ABOVE;
    }
  }

  // Limit the number of allowable cycles
  if(cycles > 10)
  {
    cycles = 10;
  }
  else if(cycles < 3)
  {
    cycles = 3;
  }
  
  if(bufFull<1)// If we do not have a full buffer (i.e. less than defined cycles have completed)
  {
    if(error<0 && state == BELOW)
    {
      count++;
      if(count>10)
      {
        if(k>=cycles)
        {
          k = 0;
          bufFull = 1;
          flag = 1;// This is to see that we have looped over the FIFO once and thus do not need to start all over
        }
        ontime = 0;
        count = 0;
        state = ABOVE;
        time1 = millis()*0.001;
        if(time2>0)//ignore the first time since we do not have a valid time2
        {
          deltaTime[k] = time1-time2;
          deltaT[k] = Tmax-Tmin;
          k++;
        }
        Tmin = 255;
        Tmax = 0;
        time2 = time1;
        if(flag == 1)// Since we have been at FIFO end once the buffer is still considered full
        {
          bufFull=1;
        }
      }
    }
    
    if(state == ABOVE && error < 0)
    {
      if(Tmeas>Tmax)
      {
        Tmax = Tmeas;
      }
      if(error < -0.2*setPoint)
      {
        ontime = 0;
      }
    }
  
    if(state ==  ABOVE && error>0)
    {
        count++;
        if(count>10)
        {
          ontime = duty;
          count = 0;
          state = BELOW;  
        }
    }
    
    if(state == BELOW && error>0)
    {
      if(Tmeas<Tmin)
      {
        Tmin = Tmeas;
      }
      if(error>0.2*setPoint)
      {
        ontime = duty;
      }
    }
  }
  else
  {
    for(int i=1;i<cycles;i++)
    {
      int32_t periodVar = deltaTime[0]-deltaTime[i];
      float tempVar = deltaT[0]-deltaT[i];
      if(abs(periodVar)<timeVariation)
      {
        check++;
      }
      if(abs(tempVar)<tempVariation)
      {
        check++;
      }
    }
    if(check == 2*(cycles-1))
      {
        tuneOver = 1;
        avgDeltaT = 0;
        check=0;
      }
      else
      {
        bufFull = 0;
        state = ABOVE;
        check=0;
      }
  }
  
  if(tuneOver >0)
  {
    ontime = 0;
    for(int i=0;i<cycles;i++)
    {
      avgDeltaT += deltaT[i]/cycles;
      avgDeltaTime += deltaTime[i]/cycles;
    }
    // Calculate ultimate period and gain
    Tu = avgDeltaTime;
    Ku = (4*duty)/(PI*avgDeltaT);
    // Ziegler Nichols no overshoot PID parameters with lowered Kd
    Kp = Ku*0.2;
    Ki = (0.4*Ku)/Tu;
    Kd = Ku*Tu*0.06;
    stateMachine = OFF;
    Serial1.print("x2.val=");Serial1.print((uint32_t)(Kp*1000));Serial1.write(0xff);Serial1.write(0xff);Serial1.write(0xff);
    Serial1.print("x3.val=");Serial1.print((uint32_t)(Ki*1000));Serial1.write(0xff);Serial1.write(0xff);Serial1.write(0xff);
    Serial1.print("x4.val=");Serial1.print((uint32_t)(Kd*1000));Serial1.write(0xff);Serial1.write(0xff);Serial1.write(0xff);
    Serial1.print("b5.pic=");Serial1.print(7);Serial1.write(0xff);Serial1.write(0xff);Serial1.write(0xff);// Save icon is gray
    Serial1.print("va1.val=");Serial1.print(0);Serial1.write(0xff);Serial1.write(0xff);Serial1.write(0xff);
    Serial1.print("va2.val=");Serial1.print(0);Serial1.write(0xff);Serial1.write(0xff);Serial1.write(0xff);
    Serial1.print("va3.val=");Serial1.print(0);Serial1.write(0xff);Serial1.write(0xff);Serial1.write(0xff);
    b5.attachPush(b5PushCallback);  // Button press enabled to be able to save parameters
  }
}

void loop(){
  nexLoop(nex_listen_list);  // Check for any touch event
}
