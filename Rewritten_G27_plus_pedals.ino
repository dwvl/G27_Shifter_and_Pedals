//  G27 shifter to USB interface
//  for Sparkfun Pro Micro
//  Original by pascalh from insidesimracing.tv
//  http://insidesimracing.tv/forums/topic/13189-diy-g25-shifter-interface-with-h-pattern-sequential-and-handbrake-modes/
//
//  this version by David Lewis 2020-04-18
//
//  1 operating mode (all others stripped out of this code version)
//  - H-pattern shifter
//
//
//  G27 shifter pinout
//
//  DB9  Color    Description       ProMicro
//  1    Purple   Button Clock      Pin 15 "15"
//  2    Grey     Button Data       Pin 14 "14"
//  3    Yellow   Button !CS & !PL  Pin 16 "16"
//  4    Orange   Shifter X axis    Pin 18 "A0"
//  5    White    Unused (SPI input)
//  6    Black    GND               
//  7    Red      +5V               
//  8    Green    Shifter Y axis    Pin 19 "A1"
//  9    Red      +5V

//  Driving Force Pro pedals pinout
//
//  DB9  Color    Description       ProMicro
//  1             Unused
//  2    White    Brake axis        Pin 21 "A3"
//  3    Green    Accelerator axis  Pin 20 "A2"
//  4    Red      GND
//  5             Unused (SPI input) 
//  6    Black    +5V
//  7    White    Accelerator axis
//  8    Green    Brake axis
//  9    Red      GND

// Pin definitions for ProMicro
#define YELLOW_RX_LED_PIN  17
#define DATA_IN_PIN        14
#define MODE_PIN           16
#define CLOCK_PIN          15
#define X_AXIS_PIN         18
#define Y_AXIS_PIN         19
#define ACCEL_AXIS_PIN     20
#define BRAKE_AXIS_PIN     21

// H-shifter mode analog axis thresholds
#define HS_XAXIS_12_MIN       200 // Minumum reading when in Gears 1,2
#define HS_XAXIS_12_MAX       430 // Maxumum reading when in Gears 1,2
#define HS_XAXIS_34_MIN       460 // Minumum reading when in Gears 3,4
#define HS_XAXIS_34_MAX       660 // Maxumum reading when in Gears 3,4
#define HS_XAXIS_56_MIN       670 // Minumum reading when in Gears 5,6, R
#define HS_XAXIS_56_MAX       900 // Maxumum reading when in Gears 5,6, R
#define HS_YAXIS_135       700 // A bit away from neutral towards Gears 1,3,5
#define HS_YAXIS_246       360 // A bit away from neutral towards Gears 2,4,6, R

// Pedal limits
#define ACCEL_FOOT_OFF_COUNTS     1010 //A-D counts
#define ACCEL_FOOT_ON_COUNTS      400
#define BRAKE_FOOT_OFF_COUNTS     300
#define BRAKE_FOOT_ON_COUNTS      1010

#define ACCEL_FOOT_OFF_OUTPUT     0    //axis position units
#define ACCEL_FOOT_ON_OUTPUT      1023
#define BRAKE_FOOT_OFF_OUTPUT     0
#define BRAKE_FOOT_ON_OUTPUT      1023

const float accelGradient = (float)(ACCEL_FOOT_ON_OUTPUT - ACCEL_FOOT_OFF_OUTPUT)
                          / (float)(ACCEL_FOOT_ON_COUNTS - ACCEL_FOOT_OFF_COUNTS); //Save time later by doing this now.
const float brakeGradient = (float)(BRAKE_FOOT_ON_OUTPUT - BRAKE_FOOT_OFF_OUTPUT)
                          / (float)(BRAKE_FOOT_ON_COUNTS - BRAKE_FOOT_OFF_COUNTS);

int accelCounts = 512;    // current pedal positions
int brakeCounts = 512;

int x = 540;    // current shifter position - neutral(ish)
int y = 510;
                          
// Shifter serially-received buttons definitions
#define DI_REVERSE         1

#define DI_RED_CENTERRIGHT 4
#define DI_RED_CENTERLEFT  5
#define DI_RED_RIGHT       6
#define DI_RED_LEFT        7
#define DI_BLACK_TOP       8
#define DI_BLACK_RIGHT     9
#define DI_BLACK_LEFT      10
#define DI_BLACK_BOTTOM    11
#define DI_DPAD_RIGHT      12
#define DI_DPAD_LEFT       13
#define DI_DPAD_BOTTOM     14
#define DI_DPAD_TOP        15

#include "Joystick.h";  // Matthew Heironimus joystick library
// Create the Joystick
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_MULTI_AXIS,
  16,   // number of buttons
  1,    // number of hat switches
  false, // whether x-axis is present
  false, // whether y-axis is present
  false, // whether z-axis is present
  false, // whether x-axis rotation is present 
  false, // whether y-axis rotation is present
  false, // whether z-axis rotation is present
  true,  // whether rudder is present
  true,  // whether throttle is present
  false, // whether accelerator is present
  false, // whether brake is present
  false); // whether steering is present

// LED blink counter
int led=0;

// Clever way of converting dPad buttons to hat switch position needs a lookup array:
const int hatSwitchPositions[] = {-1,0,90,45,180,-1,135,-1,270,315,-1,-1,225,-1,-1,-1};  // 0 to 360 degrees (-1 is released)

// Called at startup
// Must initialize hardware and software modules
void setup()
{
  // G27 shifter analog inputs configuration 
  pinMode(X_AXIS_PIN, INPUT_PULLUP);   // X axis
  pinMode(Y_AXIS_PIN, INPUT_PULLUP);   // Y axis

  // G27 shift register interface configuration 
  pinMode(DATA_IN_PIN, INPUT);         // Data in
  pinMode(MODE_PIN, OUTPUT);           // Parallel/serial mode
  pinMode(CLOCK_PIN, OUTPUT);          // Clock
  
  // Driving force pro analog inputs configuration 
  pinMode(ACCEL_AXIS_PIN, INPUT_PULLUP);   // Accelerator axis
  pinMode(BRAKE_AXIS_PIN, INPUT_PULLUP);   // Brake axis
  
  // LED outputs mode configuration 
  pinMode(YELLOW_RX_LED_PIN, OUTPUT);     // Yellow LED
  
  // Virtual joystick
  Joystick.begin(false);  // don't use autosend
  
//  // Virtual serial interface configuration - remove comment for debugging
//  Serial.begin(38400);

  // Virtual joystick initialization
  Joystick.setRudderRange(0, 1023);
  Joystick.setRudder(512);  //512 is center
  Joystick.setThrottleRange(0, 1023);
  Joystick.setThrottle(512);  //512 is center

  // Digital outputs initialization
  digitalWrite(YELLOW_RX_LED_PIN, LOW);
  digitalWrite(MODE_PIN, HIGH);
  digitalWrite(CLOCK_PIN, HIGH); 
}

// Called in a loop after initialization
void loop() 
{
  // Reading of button states from G27 shift register
  int shifterButtons[16];

  digitalWrite(MODE_PIN, LOW);         // Switch to parallel mode: digital inputs are read into shift register
  delayMicroseconds(10);               // Wait for signal to settle
  digitalWrite(MODE_PIN, HIGH);        // Switch to serial mode: one data bit is output on each clock falling edge

  for(int i=0; i<16; i++)              // Iteration over both 8 bit registers
  {
    digitalWrite(CLOCK_PIN, LOW);      // Generate clock falling edge
    delayMicroseconds(10);             // Wait for signal to settle

    shifterButtons[i]=digitalRead(DATA_IN_PIN);     // Read data bit and store it into bit array

    digitalWrite(CLOCK_PIN, HIGH);     // Generate clock rising edge
    delayMicroseconds(10);             // Wait for signal to settle
  }

  // Reading of shifter position
  int xRaw = analogRead(X_AXIS_PIN);                 // X axis
  int yRaw = analogRead(Y_AXIS_PIN);                 // Y axis

  x += (float)(xRaw - x)*0.1;  // Low-pass filter noise
  y += (float)(yRaw - y)*0.1;
  
  // Current gear calculation
  int gear=0;                          // Default value is neutral

  if(x>HS_XAXIS_12_MIN and x<HS_XAXIS_12_MAX)       // Shifter on the left?
  {
    if(y>HS_YAXIS_135) gear=1;       // 1st gear
    if(y<HS_YAXIS_246) gear=2;       // 2nd gear
  }
  else if(x>HS_XAXIS_56_MIN and x<HS_XAXIS_56_MAX)  // Shifter on the right?
  {
    if(y>HS_YAXIS_135) gear=5;       // 5th gear
    if(y<HS_YAXIS_246) gear=6;       // 6th gear
  }
  else if(x>HS_XAXIS_34_MIN and x<HS_XAXIS_34_MAX)  // Shifter is in the middle
  {
    if(y>HS_YAXIS_135) gear=3;       // 3rd gear
    if(y<HS_YAXIS_246) gear=4;       // 4th gear
  }

  if(gear!=6) shifterButtons[DI_REVERSE]=0;         // Reverse gear is allowed only on 6th gear position
  if(shifterButtons[DI_REVERSE]==1) gear=-1;        // Seems we're in reverse, not 6th gear

  // Release joystick buttons for all gears (Joystick buttons are zero-based)
  Joystick.setButton(0, LOW); // first gear
  Joystick.setButton(1, LOW); // second
  Joystick.setButton(2, LOW); // third
  Joystick.setButton(3, LOW); // fourth
  Joystick.setButton(4, LOW); // fifth
  Joystick.setButton(5, LOW); // sixth
  
  // Depress virtual button for current gear
  if(gear > 0) Joystick.setButton(gear-1, HIGH);  // Joystick buttons are zero-based

  // Set state of virtual buttons for all the physical buttons (including reverse)
  // Joystick.setButton(6, 0 == gear); // neutral (if required!)
  Joystick.setButton(7, shifterButtons[DI_REVERSE]);
  Joystick.setButton(8, shifterButtons[DI_BLACK_TOP]);
  Joystick.setButton(9, shifterButtons[DI_BLACK_LEFT]);
  Joystick.setButton(10, shifterButtons[DI_BLACK_RIGHT]);
  Joystick.setButton(11, shifterButtons[DI_BLACK_BOTTOM]);
  Joystick.setButton(12, shifterButtons[DI_RED_LEFT]);
  Joystick.setButton(13, shifterButtons[DI_RED_CENTERLEFT]);
  Joystick.setButton(14, shifterButtons[DI_RED_CENTERRIGHT]);
  Joystick.setButton(15, shifterButtons[DI_RED_RIGHT]);

  int dPadSED = 8*shifterButtons[DI_DPAD_LEFT] +
                4*shifterButtons[DI_DPAD_BOTTOM] +
                2*shifterButtons[DI_DPAD_RIGHT] +
                1*shifterButtons[DI_DPAD_TOP];  // create 4-bit encoded value of dPad buttons
  Joystick.setHatSwitch(0, hatSwitchPositions[dPadSED]);  // look up hat switch position

  // Read pedal positions and clip if necessary
  int accelCountsRaw = analogRead(ACCEL_AXIS_PIN);

  accelCounts += (float)(accelCountsRaw - accelCounts)*0.1;  // Low-pass filter noise
  
  if (accelCounts < min(ACCEL_FOOT_OFF_COUNTS,ACCEL_FOOT_ON_COUNTS))
      accelCounts = min(ACCEL_FOOT_OFF_COUNTS,ACCEL_FOOT_ON_COUNTS); // clip to minimum
  if (accelCounts > max(ACCEL_FOOT_OFF_COUNTS,ACCEL_FOOT_ON_COUNTS))
      accelCounts = max(ACCEL_FOOT_OFF_COUNTS,ACCEL_FOOT_ON_COUNTS); // clip to maximum
      
  
  int brakeCountsRaw = analogRead(BRAKE_AXIS_PIN);

  brakeCounts += (float)(brakeCountsRaw - brakeCounts)*0.1;  // Low-pass filter noise
  
  if (brakeCounts < min(BRAKE_FOOT_OFF_COUNTS,BRAKE_FOOT_ON_COUNTS))
      brakeCounts = min(BRAKE_FOOT_OFF_COUNTS,BRAKE_FOOT_ON_COUNTS); // clip to minimum
  if (brakeCounts > max(BRAKE_FOOT_OFF_COUNTS,BRAKE_FOOT_ON_COUNTS))
      brakeCounts = max(BRAKE_FOOT_OFF_COUNTS,BRAKE_FOOT_ON_COUNTS); // clip to maximum

  // Scale pedal positions for output
  int accel = ACCEL_FOOT_OFF_OUTPUT + (float)(accelCounts - ACCEL_FOOT_OFF_COUNTS)*accelGradient; // Accelerator axis
  int brake = BRAKE_FOOT_OFF_OUTPUT + (float)(brakeCounts - BRAKE_FOOT_OFF_COUNTS)*brakeGradient; // Brake axis
  Joystick.setRudder(brake);
  Joystick.setThrottle(accel);
  
  // Write new virtual joystick state
  Joystick.sendState();

//  // Write inputs and outputs (remove comments to debug)
//  Serial.print(" X: ");
//  Serial.print(x);
//  Serial.print(" Y: ");
//  Serial.print(y);
//  Serial.print(" rA: ");
//  Serial.print(accelCountsRaw);
//  Serial.print(" rB: ");
//  Serial.print(brakeCountsRaw);
//  Serial.print(" Shifter buttons: ");
//  for(int i=0; i<16; i++) Serial.print(shifterButtons[i]);
//  Serial.print("   Gear: ");
//  Serial.print(gear);
//  Serial.print(" A: ");
//  Serial.print(accel);
//  Serial.print(" B: ");
//  Serial.print(brake);
//  Serial.println(".");

  // Blink the on-board yellow LED
  if(++led==100) led=0;                     // Period is 100 cycles * 10ms = 1s
  if(led==0)
  {
    digitalWrite(YELLOW_RX_LED_PIN, LOW);    // LED is off for 50 cycles
  }
  
  if(led==50)
  {
    digitalWrite(YELLOW_RX_LED_PIN, HIGH);   // LED is on for 50 cycles 
  }
  
  // Wait
  delay(10);                                // Wait for 10ms
}
