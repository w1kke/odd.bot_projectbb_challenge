// Include libraries for the motor driver
#include <SPI.h>
#include <HighPowerStepperDriver.h>

// Include libraries for the distance sensor
#include <Wire.h>
#include <VL53L1X.h>

// Include ROS libraries
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>

// Bit set/clear/check/toggle macros
#define SET(x,y) (x |=(1<<y))
#define CLR(x,y) (x &= (~(1<<y)))
#define CHK(x,y) (x & (1<<y))
#define TOG(x,y) (x^=(1<<y))

// Setup ros handlers
// TODO: use a more sensible namespace than projectbb.
ros::NodeHandle nh;

std_msgs::Int32 pushed_z_msg;
std_msgs::Int32 pushed_y_msg;

ros::Publisher publish_z("/projectbb/core_yz/z_axis_base_link_joint", &pushed_z_msg);
ros::Publisher publish_y("/projectbb/core_yz/gripper_z_axis_joint", &pushed_y_msg);

// Transmission constants
#define STEPS_PER_MM 160          // 200 steps/rev; 32 x microstepping; 40 mm/rev
#define SCALE_FACTOR 1000.0       // Scaling factor - convert meter to millimeter (MoveIt)

// Motor constants
#define PULSE_WIDTH 2             // Pulse-width in uS 
#define STEP_DELAY_MAIN 50        // Delay (uS) between motor steps (controls speed)
#define STEP_DELAY_TRASH 200      // Delay (uS) between motor steps (controls speed)
#define CURRENT_LIMIT_MAIN 2000   // Current limit of the motor in mA
#define CURRENT_LIMIT_MAIN_SLEEP 500   // Current limit of the motor when in rest
#define CURRENT_LIMIT_TRASH 1000  // Current limit of the motor in mA

// Define constant values for robot
const long MAX_Z_POS_STEPS = 190L * STEPS_PER_MM;   // was: 240L without Jetson (-3.5 cm)
const long MAX_Y_POS_STEPS = 136L * STEPS_PER_MM;
const long MIN_DEV = 500L;

// Define pins for the motor drivers
const uint8_t DIR_PIN_LEFT = 24,         DIR_PIN_LEFT_PORTA = 2;
const uint8_t STEP_PIN_LEFT = 25,        STEP_PIN_LEFT_PORTA = 3;
const uint8_t CS_PIN_LEFT = 26,          CS_PIN_LEFT_PORTA = 4;
const uint8_t NOT_SLEEP_PIN_LEFT = 37,   NOT_SLEEP_PIN_LEFT_PORTC = 0;

const uint8_t DIR_PIN_RIGHT = 27,        DIR_PIN_RIGHT_PORTA = 5;
const uint8_t STEP_PIN_RIGHT = 28,       STEP_PIN_RIGHT_PORTA = 6;
const uint8_t CS_PIN_RIGHT = 29,         CS_PIN_RIGHT_PORTA = 7;
const uint8_t NOT_SLEEP_PIN_RIGHT = 36,  NOT_SLEEP_PIN_RIGHT_PORTC = 1;

const uint8_t DIR_PIN_TRASH = 42,        DIR_PIN_TRASH_PORTL = 7;
const uint8_t STEP_PIN_TRASH = 43,       STEP_PIN_TRASH_PORTL = 6;
const uint8_t DISABLE_PIN_TRASH = 44,    DISABLE_PIN_TRASH_PORTL = 5;

const uint8_t Y_ENDSTOP = 35,            Y_ENDSTOP_PORTC = 2;
const uint8_t Z_ENDSTOP = 34,            Z_ENDSTOP_PORTC = 3;

const uint8_t IS_READY_PIN = 8,          IS_READY_PIN_PORTH = 5;

// Declare position variables
long z_pos = 0;
long y_pos = 0;

// Declare motor drivers
HighPowerStepperDriver left_sd;
HighPowerStepperDriver right_sd;

// Declare sensor object
VL53L1X sensor;

uint16_t sensorValue = 0;

// Declare booleans flags for the motor direction
bool
CW = true,        // Motor direction flag - clockwise
CCW = false,      // Motor direction flag - counterclockwise
LEFT_DIR,         // Motor directions can be changed in step_motors()
RIGHT_DIR,
TRASH_DIR;

void open_trash() {
  TRASH_DIR = CW;

  CLR(PORTL, DISABLE_PIN_TRASH_PORTL);

  for (long i = 0; i < 2400; i++) {
    step_trash();
  }

  //CLR(PORTL, NOT_SLEEP_PIN_TRASH_PORTL);
}

void close_trash() {
  TRASH_DIR = CCW;

  //SET(PORTL, NOT_SLEEP_PIN_TRASH_PORTL);

  for (long i = 0; i < 2400; i++) {
    step_trash();
  }

  SET(PORTL, DISABLE_PIN_TRASH_PORTL);
}

void step_trash() {
  // Set the motor directions
  (TRASH_DIR == CCW) ? SET(PORTL, DIR_PIN_TRASH_PORTL) : CLR(PORTL, DIR_PIN_TRASH_PORTL);
  _delay_us(PULSE_WIDTH);           // Wait for direction lines to stabilise (PULSE_WIDTH uS)

  // Create pulse
  SET(PORTL, STEP_PIN_TRASH_PORTL); // Create leading edge
  _delay_us(PULSE_WIDTH);           // Keep step pins high for PULSE_WIDTH uS
  CLR(PORTL, STEP_PIN_TRASH_PORTL); // Create trailing edge

  _delay_us(STEP_DELAY_TRASH);      // Delay between steps - determines the speed
}

/**
   Function to move the stepper motors for the CoreYZ system 1 step. First, the function checks the required direction for
   both motors. Then, it creates a pulse for the stepper driver to signal that one/both motor(s) should take one step.
*/
void step_motors() {
  // Read current state PORTA
  byte pattern = PORTA;

  // Set the motor directions - Motor windings reversed
  (LEFT_DIR == CCW) ? SET(pattern, DIR_PIN_LEFT_PORTA) : CLR(pattern, DIR_PIN_LEFT_PORTA);
  (RIGHT_DIR == CCW) ? SET(pattern, DIR_PIN_RIGHT_PORTA) : CLR(pattern, DIR_PIN_RIGHT_PORTA);
  PORTA = pattern;
  _delay_us(PULSE_WIDTH);           // Wait for direction lines to stabilise (PULSE_WIDTH uS)

  // Create the leading edge of step pulses
  pattern = SET(pattern, STEP_PIN_LEFT_PORTA);
  pattern = SET(pattern, STEP_PIN_RIGHT_PORTA);
  PORTA = pattern;
  _delay_us(PULSE_WIDTH);           // Keep step pins high/low for PULSE_WIDTH uS

  // Create trailing-edge of step pulses
  pattern = CLR(pattern, STEP_PIN_LEFT_PORTA);
  pattern = CLR(pattern, STEP_PIN_RIGHT_PORTA);
  PORTA = pattern;

  // Delay between steps - determines the speed
  _delay_us(STEP_DELAY_MAIN);
}

/**
   Function to move the motors one step to the left.
*/
void left() {
  LEFT_DIR = CCW;
  RIGHT_DIR = CCW;
  step_motors();
}

/**
   Function to move the motors one step to the right.
*/
void right() {
  LEFT_DIR = CW;
  RIGHT_DIR = CW;
  step_motors();
}

/**
   Function to move the motors one step up.
*/
void up() {
  LEFT_DIR = CW;
  RIGHT_DIR = CCW;
  step_motors();
}

/**
   Function to move the motors one step down.
*/
void down() {
  LEFT_DIR = CCW;
  RIGHT_DIR = CW;
  step_motors();
}

void wake_up_steppers() {
  byte pattern = PORTC;
  pattern = SET(pattern, NOT_SLEEP_PIN_LEFT_PORTC);
  pattern = SET(pattern, NOT_SLEEP_PIN_RIGHT_PORTC);
  PORTC = pattern;
}

//void put_steppers_to_sleep() {
//  byte pattern = PORTC;
//  pattern = CLR(pattern, NOT_SLEEP_PIN_LEFT_PORTC);
//  pattern = CLR(pattern, NOT_SLEEP_PIN_RIGHT_PORTC);
//  PORTC = pattern;
//}
void steppers_high_power() {
  // Set the current limit
  left_sd.setCurrentMilliamps36v4(CURRENT_LIMIT_MAIN);
  right_sd.setCurrentMilliamps36v4(CURRENT_LIMIT_MAIN);
}

void steppers_low_power() {
  // Set the current limit
  left_sd.setCurrentMilliamps36v4(CURRENT_LIMIT_MAIN_SLEEP);
  right_sd.setCurrentMilliamps36v4(CURRENT_LIMIT_MAIN_SLEEP);
}

/**
   Function to tell ROS what the current position of the coreYZ system is.
*/
void publish_position() {
  pushed_z_msg.data = z_pos;
  publish_z.publish(&pushed_z_msg);
  pushed_y_msg.data = y_pos;
  publish_y.publish(&pushed_y_msg);
}

void move_home() {
  // Move the coreYZ system to the left until it triggers the Y endstop
  while (!CHK(PINC, Y_ENDSTOP_PORTC)) {
    right();
  }

  // Move the coreYZ system up until it triggers the Z endstop
  while (!CHK(PINC, Z_ENDSTOP_PORTC)) {
    up();
  }

  // Set calibrated position
  y_pos = MAX_Y_POS_STEPS;
  z_pos = MAX_Z_POS_STEPS;

  for (int i = 0; i < 1600; i++) {
    down();
    z_pos--;
  }
}

/**
   Move the coreYZ system down until it reaches the sand.
*/
void move_down() {
  sensorValue = sensor.read();

  for (long i = 0; (sensorValue > 110 || sensorValue == 0)&& i<100000L ; i++) { //112 (114) old. Limit to maximum 100000 loops (plenty of time)
    down();
    z_pos--;

    if (i % 1000L == 0L) {
      sensorValue = sensor.read(false);
      Serial1.println(i);
    }
  }
}

void command_callback(const std_msgs::String &cmd) {
  CLR(PORTH, IS_READY_PIN_PORTH);
  steppers_high_power();

  if (strcmp(cmd.data, "home") == 0) {
    move_home();
  } else if (strcmp(cmd.data, "down") == 0) {
    move_down();
  } else if (strcmp(cmd.data, "open") == 0) {
    open_trash();
  } else if (strcmp(cmd.data, "close") == 0) {
    close_trash();
  }

  steppers_low_power(); // Don't put them to sleep, because it will drop the carriage! just put it in low power.
  publish_position();
  SET(PORTH, IS_READY_PIN_PORTH);
}

/**
   Move callback function to move the coreyz system to a specified location. Note that this code is expected to run in a
   serial manner, so parallel execution is __not__ expected to happen or to be supported.
   @param msg - Float32MultiArray object containing the position parameter for the Carthesian frame, index 0: y position
   index 1: z position.
*/
void move_callback(const std_msgs::Int32MultiArray &msg) {
  CLR(PORTH, IS_READY_PIN_PORTH);

  // Calculate the goal in steps per axis from the MoveIt command
  long y_goal = min(max(-MAX_Y_POS_STEPS, msg.data[1]), MAX_Y_POS_STEPS);
  long z_goal = min(max(-MAX_Z_POS_STEPS, msg.data[0]), MAX_Z_POS_STEPS);
  //long y_goal = min(max(-MAX_Y_POS_STEPS, msg.data[1] * STEPS_PER_MM * SCALE_FACTOR), MAX_Y_POS_STEPS));
  //long z_goal = min(max(-MAX_Z_POS_STEPS, msg.data[0] * STEPS_PER_MM * SCALE_FACTOR), MAX_Z_POS_STEPS));

  long
  dy,                               // Line slope
  dz,
  slope,

  longest,                          // Axis lengths
  shortest,
  maximum,

  error,                            // Bresenham thresholds
  threshold;

  // Find longest and shortest axis
  dy = y_goal - y_pos;                  // Horizontal distance
  dz = z_goal - z_pos;                  // Vertical distance
  longest = max(abs(dy), abs(dz));      // Longest axis
  shortest = min(abs(dy), abs(dz));     // Shortest axis

  // Scale Bresenham values by 2 * longest
  error = -longest;                     // Add offset to so we can test at zero
  threshold = 0;                        // Test now done at zero
  maximum = (longest << 1);             // Multiply by two
  slope = (shortest << 1);              // Multiply by two - slope equals (shortest*2/longest*2)

  // Initialise the swap flag
  bool YZswap = true;                   // Used for motor decoding
  if (abs(dy) >= abs(dz)) YZswap = false;

  steppers_low_power();

  // Pretend we are always in octant 0
  for (long i = 0; i < longest; i++) {
    // Move left/right along the Y-axis
    if (YZswap) { // Swap
      if (dz < 0) {
        z_pos--;
        down();   // Move down 1 step
      } else {
        z_pos++;
        up();     // Move up 1 step
      }
    } else { // No swap
      if (dy < 0) {
        y_pos--;
        left();   // Move left 1 step
      } else {
        y_pos++;
        right();  // Move right 1 step
      }
    }

    // Move up/down along the Z-axis
    error += slope;
    if (error > threshold) {
      error -= maximum;

      if (YZswap) {  // Swap
        if (dy < 0) {
          y_pos--;
          left();   // Move left 1 step
        } else {
          y_pos++;
          right();  // Move right 1 step
        }
      } else {  // No swap
        if (dz < 0) {
          z_pos--;
          down();   // Move down 1 step
        } else {
          z_pos++;
          up();     // Move up 1 step
        }
      }
    }

    if (i % 5000L == 0L) {
      if (abs(z_pos - pushed_z_msg.data) > MIN_DEV) {
        pushed_z_msg.data = z_pos;
        publish_z.publish(&pushed_z_msg);
      }
      if (abs(y_pos - pushed_y_msg.data) > MIN_DEV) {
        pushed_y_msg.data = y_pos;
        publish_y.publish(&pushed_y_msg);
      }
    }
  }

  steppers_low_power();
  publish_position();
  SET(PORTH, IS_READY_PIN_PORTH);
}

ros::Subscriber <std_msgs::Int32MultiArray> position_subscriber("/projectbb/core_yz/position", &move_callback);
ros::Subscriber <std_msgs::String> command_subscriber("/projectbb/core_yz/command", &command_callback);

void setup() {
  SPI.begin();
  Serial1.begin(115200);
  // Initialize the motor drivers
  left_sd.setChipSelectPin(CS_PIN_LEFT);
  right_sd.setChipSelectPin(CS_PIN_RIGHT);

  // Set pinMode and drive the STEP, DIR and !SLEEP pins low
  pinMode(STEP_PIN_LEFT, OUTPUT);
  pinMode(STEP_PIN_RIGHT, OUTPUT);
  pinMode(STEP_PIN_TRASH, OUTPUT);
  pinMode(DIR_PIN_LEFT, OUTPUT);
  pinMode(DIR_PIN_RIGHT, OUTPUT);
  pinMode(DIR_PIN_TRASH, OUTPUT);
  pinMode(NOT_SLEEP_PIN_LEFT, OUTPUT);
  pinMode(NOT_SLEEP_PIN_RIGHT, OUTPUT);
  pinMode(DISABLE_PIN_TRASH, OUTPUT);
  digitalWrite(STEP_PIN_LEFT, LOW);
  digitalWrite(STEP_PIN_RIGHT, LOW);
  digitalWrite(STEP_PIN_TRASH, LOW);
  digitalWrite(DIR_PIN_LEFT, LOW);
  digitalWrite(DIR_PIN_RIGHT, LOW);
  digitalWrite(DIR_PIN_TRASH, LOW);
  digitalWrite(NOT_SLEEP_PIN_LEFT, LOW);
  digitalWrite(NOT_SLEEP_PIN_RIGHT, LOW);
  digitalWrite(DISABLE_PIN_TRASH, HIGH);
  Serial1.println("setup");
  // Give the drivers some time to power up
  delay(1);

  // Setup the endstops
  pinMode(Y_ENDSTOP, INPUT_PULLUP);
  pinMode(Z_ENDSTOP, INPUT_PULLUP);

  // Setup is_ready pin
  pinMode(IS_READY_PIN, OUTPUT);
  digitalWrite(IS_READY_PIN, LOW);

  // Reset the driver to its default settings and clear latched status conditions
  left_sd.resetSettings();
  right_sd.resetSettings();

  left_sd.clearStatus();
  right_sd.clearStatus();

  // Set the decay mode to auto mixed decay
  left_sd.setDecayMode(HPSDDecayMode::AutoMixed);
  right_sd.setDecayMode(HPSDDecayMode::AutoMixed);

  // Set the current limit
  left_sd.setCurrentMilliamps36v4(CURRENT_LIMIT_MAIN);
  right_sd.setCurrentMilliamps36v4(CURRENT_LIMIT_MAIN);


  // Set the microstepping
  left_sd.setStepMode(HPSDStepMode::MicroStep32);
  right_sd.setStepMode(HPSDStepMode::MicroStep32);

  // Enable the motor outputs.
  left_sd.enableDriver();
  right_sd.enableDriver();

  // Start the Wire library
  Wire.begin();
  Wire.setClock(100000);    // use 100 kHz I2C

  // Initialize the distance sensor
  sensor.setTimeout(500);
  Serial1.println("setting up sensor");
  //  if (!sensor.init()) { // TODO: we removed the sensor
  //    while (1);
  //  }
  Serial1.println("Sensor setup done");
  // Set distance mode and timing budget (uS)
  sensor.setDistanceMode(VL53L1X::Short);
  sensor.setMeasurementTimingBudget(50000);

  // Start continuous readings at a 50 ms interval
  sensor.startContinuous(50);

  // Initialize the ROS objects
  nh.initNode();
  nh.advertise(publish_z);
  nh.advertise(publish_y);
  nh.subscribe(position_subscriber);
  nh.subscribe(command_subscriber);

  // Signal ready
  digitalWrite(IS_READY_PIN, HIGH);
  Serial1.println("rdy");
  wake_up_steppers(); // so position stays locked
  steppers_low_power();
}
String sdata = ""; // Initialised to nothing.
void loop() {
  nh.spinOnce();
  delay(1);
  byte ch;
  String valStr;
  int val;

  if (Serial1.available()) {
    ch = Serial1.read();

    sdata += (char)ch;

    if (ch == '\r') { // Command received and ready.
      sdata.trim();

      // Process command in sdata.
      switch ( sdata.charAt(0) ) {
        case 'h':
          Serial1.println("Start Process");
          CLR(PORTH, IS_READY_PIN_PORTH);
          steppers_high_power();
          move_home();
          steppers_low_power();
          publish_position();
          SET(PORTH, IS_READY_PIN_PORTH);
          break;
        case 'd':
          Serial1.println("down");
          CLR(PORTH, IS_READY_PIN_PORTH);
          steppers_high_power();
          move_down();
          steppers_low_power();
          publish_position();
          SET(PORTH, IS_READY_PIN_PORTH);
          break;
        case 'v':
          if (sdata.length() > 1) {
            valStr = sdata.substring(1);
            val = valStr.toInt();
          }
          Serial1.print("Val ");
          Serial1.println(val);
          break;
        default: Serial1.println(sdata);
      } // switch

      sdata = ""; // Clear the string ready for the next command.
    } // if \r
  }  // available

}
