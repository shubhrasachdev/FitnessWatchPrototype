#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "RTClib.h"
//#include<WiFi.h>
//#include "ThingSpeak.h"

//WiFi Credentials
//const char * ssid = "SSID";
//const char * pwd = "PASSWORD";

//rtc obj
RTC_DS3231 rtc;

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(128, 64, &Wire, -1);

//WiFiClient  client;
//unsigned long channel_id = CHANNEL_id;
//const char * api_key = "WRITE-API-KEY";


//PulseSensor Variables
// these variables are volatile because they are used during the interrupt service routine!
volatile int BPM;                   // used to hold the pulse rate
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // holds the time between beats, must be seeded! 
volatile boolean Pulse = false;     // true when pulse wave is high, false when it's low
volatile boolean QS = false;        // becomes true when ESP32 finds a beat.
volatile int rate[10];                    // array to hold last ten IBI values
volatile unsigned long sampleCounter = 0; // used to determine pulse timing
volatile unsigned long lastBeatTime = 0;  // used to find IBI
volatile int P =512;                      // used to find peak in pulse wave, seeded
volatile int T = 512;                     // used to find trough in pulse wave, seeded
volatile int thresh = 512;                // used to find instant moment of heart beat, seeded
volatile int amp = 100;                   // used to hold amplitude of pulse waveform, seeded
volatile boolean firstBeat = true;        // used to seed rate array so we startup with reasonable BPM
volatile boolean secondBeat = false;      // used to seed rate array so we startup with reasonable BPM
hw_timer_t * timer = NULL;
static int i=0;

//Accelerometer variables
int xpin=12, ypin=13, zpin=14;
float xval[100]={0};
float yval[100]={0};
float zval[100]={0};
float xavg, yavg, zavg;
int steps=0;
float totvect = 0;

void setup(){
  Serial.begin(115200);  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println("SSD1306 allocation failed");
    for(;;);
  }
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  //WiFi.begin(ssid, pwd);
  /*while(WiFi.status()!=WL_CONNECTED)
  {
    Serial.println(".");
  }*/
  Serial.println("Connected");
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  interruptSetup();                 // sets up to read Pulse Sensor signal every 2mS 
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Calibration");
  display.display();
  delay(1000);
  calibrate();
  display.clearDisplay();
  //ThingSpeak.begin(client);  // Initialize ThingSpeak
}

void loop(){
  if (QS == true){                       // Quantified Self flag is true when ESP32 finds a heartbeat
        Serial.print("BPM:");
        Serial.println(BPM);
        QS = false;                      // reset the Quantified Self flag for next time    
  }
  float xaccl=0, yaccl=0, zaccl=0, temp=0;
  xaccl = float(analogRead(xpin));
  delay(1);
  yaccl = float(analogRead(ypin));
  delay(1);
  zaccl = float(analogRead(zpin));
  delay(1);
  temp=totvect;
  totvect = sqrt(((xaccl-xavg)* (xaccl-xavg))+ ((yaccl - yavg)*(yaccl - yavg)) + ((zaccl - zavg)*(zaccl - zavg)));
  float diff = fabs(temp-totvect);
  Serial.println(totvect);
  Serial.println(totvect-temp);
  //calculate steps 
  if (diff>=80.0)
    {
      if(temp!=0) steps=steps+1;
     }
  float cal = 0.035 * steps;
  DateTime now = rtc.now();
  Serial.print("steps=");
  Serial.println(steps);
  display.clearDisplay();
  display.setCursor(0,0);
  display.print("DATE:  ");
  display.print(now.day());
  display.print('/');
  display.print(now.month());
  display.print('/');
  display.print(now.year());
  display.setCursor(0,10);
  display.print("TIME:  ");
  display.print(now.hour());
  display.print(':');
  display.print(now.minute());
  display.setCursor(0,20);
  display.print("BPM:  ");
  display.print(BPM);
  display.setCursor(0,30);
  display.print("STEPS:  ");
  display.print(steps);
  display.setCursor(0,40);
  display.print("CALORIES BURNT:  ");
  display.print(cal);
  display.display();
  delay(750);
  //ThingSpeak.setField(1,BPM);
  //ThingSpeak.writeFields(channel_id, api_key);
}

void interruptSetup(){     
  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more info)
  timer = timerBegin(0, 80, true);
  // Initializes Timer to run the ISR to sample every 2mS as per original Sketch.
  // Attach ISRTr function to our timer.
  timerAttachInterrupt(timer, &ISRTr, true);
  // Set alarm to call isr function every 2 milliseconds (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, 2000, true);
  // Start an alarm
  timerAlarmEnable(timer);
} 

// THIS IS THE HW-TIMER INTERRUPT SERVICE ROUTINE - Timer makes sure that we take a reading every 2 miliseconds
void ISRTr(){                                 // triggered when timer fires
  Signal = analogRead(34);                    // read the Pulse Sensor on pin 34 3.3v sensor power
  Signal = map(Signal, 0, 4095, 0, 1023);     // Map the value back to original sketch range......
  sampleCounter += 2;                         // keep track of the time in mS with this variable
  int N = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise

    //  find the peak and trough of the pulse wave
  if(Signal < thresh && N > (IBI/5)*3){       // avoid dichrotic noise by waiting 3/5 of last IBI
    if (Signal < T){                        // T is the trough
      T = Signal;                         // keep track of lowest point in pulse wave 
    }
  }
  if(Signal > thresh && Signal > P){          // thresh condition helps avoid noise
    P = Signal;                             // P is the peak
  }                                        // keep track of highest point in pulse wave

  //  NOW IT'S TIME TO LOOK FOR THE HEART BEAT - signal surges up in value every time there is a pulse
  if (N > 250){                                   // avoid high frequency noise
    if ( (Signal > thresh) && (Pulse == false) && (N > (IBI/5)*3) ){        
      Pulse = true;                               // set the Pulse flag when we think there is a pulse
      
      IBI = sampleCounter - lastBeatTime;         // measure time between beats in mS
      lastBeatTime = sampleCounter;               // keep track of time for next pulse

      if(secondBeat){                        // if this is the second beat, if secondBeat == TRUE
        secondBeat = false;                  // clear secondBeat flag
        for(int i=0; i<=9; i++){             // seed the running total to get a realisitic BPM at startup
          rate[i] = IBI;                      
        }
      }

      if(firstBeat){                         // if it's the first time we found a beat, if firstBeat == TRUE
        firstBeat = false;                   // clear firstBeat flag
        secondBeat = true;                   // set the second beat flag
        sei();                               // enable interrupts again
        return;                              // IBI value is unreliable so discard it
      }   


      // keep a running total of the last 10 IBI values
      word runningTotal = 0;                  // clear the runningTotal variable    

      for(int i=0; i<=8; i++){                // shift data in the rate array
        rate[i] = rate[i+1];                  // and drop the oldest IBI value 
        runningTotal += rate[i];              // add up the 9 oldest IBI values
      }

      rate[9] = IBI;                          // add the latest IBI to the rate array
      runningTotal += rate[9];                // add the latest IBI to runningTotal
      runningTotal /= 10;                     // average the last 10 IBI values 
      BPM = 60000/runningTotal;              
      QS = true;                              // set Quantified Self flag 
      // QS FLAG IS NOT CLEARED INSIDE THIS ISR
    }                       
  }
  if (Signal < thresh && Pulse == true){   // when the values are going down, the beat is over
    
    Pulse = false;                         // reset the Pulse flag so we can do it again
    amp = P - T;                           // get amplitude of the pulse wave
    thresh = amp/2 + T;                    // set thresh at 50% of the amplitude
    P = thresh;                            // reset these for next time
    T = thresh;
  }
  if (N > 2500){                           // if 2.5 seconds go by without a beat
    thresh = 512;                          // set thresh default
    P = 512;                               // set P default
    T = 512;                               // set T default
    lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date        
    firstBeat = true;                      // set these to avoid noise
    secondBeat = false;                    // when we get the heartbeat back
  }
}

void calibrate()

{
  float sum=0;
  float sum1=0;
  float sum2=0;
  for (int i=0;i<100;i++)
  {
    xval[i]=float(analogRead(xpin));
    sum=xval[i]+sum;
  }
  delay(100);
  xavg=sum/100.0;
  Serial.println(xavg);
  for (int j=0;j<100;j++)
  {
    yval[j]=float(analogRead(ypin));
    sum1=xval[j]+sum1;
  }
  yavg=sum1/100.0;
  Serial.println(yavg);
  delay(100);
  for (int i=0;i<100;i++)
  {
  zval[i]=float(analogRead(zpin));
  sum2=zval[i]+sum2;
  }
  zavg=sum2/100.0;
  delay(100);
  Serial.println(zavg);
}
