#include <OneWire.h>
#include <DallasTemperature.h>
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
// Controller variables
float integralOld;
float errorOld;
float yOld;
uint8_t cnt = 0;
uint8_t Tsample = 5;// sampletime = Tsample x 1 second
bool logging = 0;
/*********** Below two variables are slack on the autotune where multiple cycles must be within these limts to be accepted ! **********************/
uint16_t timeVariation = 120;//Allowable cycle time variation in seconds - 
float tempVariation = 2.0;// Allowable variation of temprature between cycles  - 



void setup(){

  //the pin 8 is only because I power my temp sensor from here - just remove it !
  pinMode(8, OUTPUT);
  digitalWrite(8,HIGH);
  Serial.begin(250000);
  // Load PID and setpoint from EEPROM
  Kp = EEPROM.readFloat(0);
  Ki = EEPROM.readFloat(4);
  Kd = EEPROM.readFloat(8);
  setPoint = EEPROM.readFloat(12);
  sensors.begin();
  delay(1000);
  sensors.requestTemperatures(); // Send the command to get temperatures
  sensors.getTempCByIndex(0);
 
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
    if(cnt>(Tsample-1))
    {
      ontime = Controller(error);
      cnt=0;
    }
      cnt++;
      initTuning = 1;
  }
  else if(stateMachine == OFF)
  {
  ontime = 0;
  initTuning = 1;
  // Reset controller
  integralOld = 0;
  errorOld = 0;
  yOld = 0;
  }
  else if(stateMachine == AUTOTUNE)
  {
  tuning();
  }
  
  if(Tmeas>tempLimit)// We do not allow more than the tempLimit !
  {
  ontime = 0;
  }
  
  // Update PWM register
  OCR4A = map(ontime,0,100,0x0,0xFFFF);//Set PWM duty cycle
  if(logging > 0)
  { 
  Serial.println(Tmeas);
  }
}

//********* CONTROLLER ****************//
uint8_t Controller(float error)
{
  float limit = 100.0;
  float y;
  float integral;
  static float der;
  float alpha = 0.5;//Alpha = 1 equals no filtering whereas alpha = 0.1 is a lot of low-pass filtering...
  
  der = der + alpha*((error-errorOld) - der);
  
  errorOld = error;

  // Anti wind-up using clamping of the integrator
  if((yOld>limit && error > 0)|| (yOld<0 && error < 0))
  {
    integral = integralOld;
  }
  else
  {
    integral = (integralOld + (error*Tsample))*Ki;
  }
  if(integral > limit)//this may actually be needed if we have a very slow process. The D can give a negative term that allows the integrator to build up and exceed the limit theoretically...
  {
    integral = integralOld;
  }

  y = (integral + Kp*error + Kd*der);// 
  
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
    Tu = avgDeltaTime;//1800
    Ku = (4*duty)/(PI*avgDeltaT);//23.16
    EEPROM.writeFloat(16,Tu);
    EEPROM.writeFloat(20,Ku);

    //CLASSICPID
    Kp = 0.6*Ku;
    Ki = 1.2*Ku/Tu;
    Kd = Ku*Tu*0.075;
    
    Kd = Kd/Tsample;
    stateMachine = OFF;
    Serial.println("Tuning done");
  }
}

void selectControl(int type)
{
  float Tu;
  float Ku;
  Tu = EEPROM.readFloat(16);
  Ku = EEPROM.readFloat(20);
  
    if(type == 1)
    {
      // P-control
      Kp = 0.5*Ku;
      Ki = 0;
      Kd = 0;
    }
    else if(type == 2)
    {
      // PI-control
      Kp = 0.45*Ku;//
      Ki = 0.54*Ku/Tu;//
      Kd = 0;
    }
    else if(type == 3)
    {
      // PD-control
      Kp = 0.8*Ku;
      Ki = 0;
      Kd = Ku*Tu*0.1;
    }
    else if(type == 4)
    {
      //Classic PID-control
      Kp = 0.6*Ku;
      Ki = 1.2*Ku/Tu;
      Kd = Ku*Tu*0.075;
    }
    else if(type == 5)
    {
      // Pessen Control
      Kp = 0.7*Ku;
      Ki = 1.75*Ku/Tu;
      Kd = Ku*Tu*0.105;
    }
    else if(type == 6)
    {
      // Some Overshoot
      Kp = 0.333*Ku;
      Ki = 0.666*Ku/Tu;
      Kd = Ku*Tu*0.111;
    }
    else if(type == 7)
    {
      // No Overshoot
      Kp = Ku*0.2;//10.355Ku
      Ki = (0.4*Ku)/Tu;//1380Tu
      Kd = Ku*Tu*0.067;
    }
    Kd = Kd/Tsample;
}

void printHelp(void)
{
  Serial.println(F("HPACS help menu:"));
  Serial.println(F(""));
  Serial.println(F("Show this help menu: 'H'"));
  Serial.println(F("Get PID Parameters and setpoint: 'G'"));
  Serial.println(F("Set new target: 'T=55.0'"));
  Serial.println(F("Set Proportional constant: 'P=5'"));
  Serial.println(F("Set Integral constant: 'I=0.01'"));
  Serial.println(F("Set Differential constant: 'D=100'"));
  Serial.println(F("Start temperature readout: 'L'"));
  Serial.println(F("Save values in EEPROM: 'S'"));
  Serial.println(F("Start process: 'N'"));
  Serial.println(F("Stop process: 'F'"));
  Serial.println(F("Start tuning: 'C'"));
  Serial.println(F("Select controller type: 'K'"));
  Serial.println(F(""));
  Serial.println(F(""));
}

void saveValues(void)
{
  EEPROM.writeFloat(0,Kp);
  EEPROM.writeFloat(4,Ki);
  EEPROM.writeFloat(8,Kd);
  EEPROM.writeFloat(12,setPoint);  
}

void checkSensor(void)
{
  if(Tmeas < 0)
  {
    stateMachine = OFF;
  }
  //check also if the temperature is below setpoint, and if, then if it is moving upwards when ontime > 0 for some time
}

void loop(){
  // Menu structure for serial
  printHelp();
  while(1)
  {
    checkSensor();
  /* must be able to edit SP, Kd, Ki, Kp. Must be able to start tuning, stop and start controlling. Must be able to save values. */
    if(Serial.available() > 0)
    {
      int input = Serial.read();
  
      switch(input)
      {
        case 'H':
          printHelp();          
          break;
          
        case 'G':
          Serial.print("Target:");Serial.println(setPoint);
          Serial.print("Kp:");Serial.println(Kp,4);
          Serial.print("Ki:");Serial.println(Ki,4);
          Serial.print("Kd:");Serial.println(Kd,4);
          Serial.println(F(""));
          Serial.println(F(""));
          break;     

        case 'T':
          setPoint = Serial.parseFloat();
          Serial.print("New Target:");Serial.println(setPoint);
          Serial.println(F(""));
          Serial.println(F(""));
          break;
          
        case 'P':
          Kp = Serial.parseFloat();
          Serial.print("New Kp:");Serial.println(Kp,4);
          Serial.println(F(""));
          Serial.println(F(""));
          break;

        case 'I':
          Kp = Serial.parseFloat();
          Serial.print("New Ki:");Serial.println(Ki,4);
          Serial.println(F(""));
          Serial.println(F(""));
          break;

        case 'D':
          Kp = Serial.parseFloat();
          Serial.print("New Kd:");Serial.println(Kd,4);
          Serial.println(F(""));
          Serial.println(F(""));
          break;
          
        case 'L':
          if(logging < 1)
          {
            Serial.println("Temperature readout at 1 Hz starting - send T to stop");
            Serial.println(F(""));
            Serial.println(F(""));
          }
          else
          {
            Serial.println("Temperature readout stopped");
            Serial.println(F(""));
            Serial.println(F(""));
          }
          logging ^= 1;//toggle logging/temperature output
          break;
          
        case 'S':
          saveValues();
          Serial.println("Values saved in EEPROM:");
          Serial.print("Target:");Serial.println(setPoint);
          Serial.print("Kp:");Serial.println(Kp,4);
          Serial.print("Ki:");Serial.println(Ki,4);
          Serial.print("Kd:");Serial.println(Kd,4);
          Serial.println(F(""));
          Serial.println(F(""));
          break;

        case 'N':
          stateMachine = ON;
          Serial.println("Process is now running, send F to stop");
          Serial.println(F(""));
          Serial.println(F(""));
          break;

        case 'F':
          stateMachine = OFF;
          Serial.println("Process is now stopped");
          Serial.println(F(""));
          Serial.println(F(""));
          break;

        case 'C':
          stateMachine = AUTOTUNE;
          Serial.println("Tuning initiated, send F to abort");
          Serial.println(F(""));
          Serial.println(F(""));
          break;

        case 'K':
          Serial.println("Select a controller type:");
          Serial.println(F(""));
          Serial.println(F("P-control: '1'"));
          Serial.println(F("PI-control: '2'"));
          Serial.println(F("PD-control: '3'"));
          Serial.println(F("PID-control: '4'"));
          Serial.println(F("Pessen: '5'"));
          Serial.println(F("Some overshoot: '6'"));
          Serial.println(F("No overshoot: '7'"));
          RETRY:
          while(!Serial.available());
          int type = Serial.parseInt();
          if(type>0)
          {
            selectControl(type);
            Serial.print("You selected:");Serial.println(type);
            Serial.println(F(""));
            Serial.println(F(""));
            break;
          }
          else
          {
            goto RETRY;
          }
               
        default:
          break;
      }
    }
  }
}
