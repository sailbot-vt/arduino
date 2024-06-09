#include <SPI.h>
#include <HighPowerStepperDriver.h>
#include <AMT22_Encoder.h>

#define RUDDER_MOTOR_SLEEP_PIN 0
#define RUDDER_MOTOR_CHIP_SELECT_PIN 1 
#define MAST_MOTOR_SLEEP_PIN 7
#define MAST_MOTOR_CHIP_SELECT_PIN 8

#define RUDDER_ENCODER_CHIP_SELECT_PIN 19
#define MAST_ENCODER_CHIP_SELECT_PIN 20

#define CLOCKWISE 1
#define COUNTER_CLOCKWISE 0

#define MAX_RUDDER_CURRENT 2000
#define MAX_MAST_CURRENT 2000

// This is measured in number of steps before you check the encoder and jetson serial values. Encoder and jetson serial have inherent delay. 
// So the more steps you do before you check the encoder and jetson serial, the faster you go, but the harder it is to control
// very similar to p controller gain (Kp)
#define RUDDER_GAIN 300
#define MAST_GAIN 150

// // This is because multiturn encoders don't allow us to zero them
// #define MAST_MOTOR_ZERO_POINT (174.64 + 600)
#define WINCH_ZERO_POINT 200

#define MAX_RUDDER_ANGLE 30
#define MIN_RUDDER_ANGLE -30

#define MAX_MAST_MOTOR_ANGLE 580
#define MIN_MAST_MOTOR_ANGLE -600

#define MAX_SAIL_ANGLE 90
#define MIN_SAIL_ANGLE 0


// #define INVERTED_CONTROLS true
// #define RUDDER_ANGLE_OFFSET 36
// #define mast_motor_angle_OFFSET 0

const float MID_RUDDER_ANGLE = (MAX_RUDDER_ANGLE + MIN_RUDDER_ANGLE) / 2;
const float MID_MAST_MOTOR_ANGLE = (MAX_MAST_MOTOR_ANGLE + MIN_MAST_MOTOR_ANGLE) / 2;

const int MAX_RUDDER_ERROR = (MAX_RUDDER_ANGLE - MIN_RUDDER_ANGLE);
const int MAX_MAST_ERROR = (MAX_MAST_MOTOR_ANGLE - MIN_RUDDER_ANGLE);
const int MAX_SAIL_ERROR = (MAX_SAIL_ANGLE - MIN_SAIL_ANGLE);

// const float STEP_SIZE_DEGREES = 0.13109978148; // (1.8/(13.73));

#define ACCEPTABLE_RUDDER_ERROR 1   // in degrees
#define ACCEPTABLE_SAIL_ERROR 0.5     // in degrees


// This period is the length of the delay between steps, which controls the
// stepper motor's speed.  You can increase the delay to make the stepper motor
// go slower.  If you decrease the delay, the stepper motor will go faster, but
// there is a limit to how fast it can go before it starts missing steps.
#define STEP_PERIOD_US 2000

HighPowerStepperDriver rudder_stepper_driver;
HighPowerStepperDriver mast_stepper_driver;
AMT22_Encoder* rudder_encoder;
AMT22_Encoder* mast_encoder;

float current_rudder_angle = 0; 
float current_mast_motor_angle = 0;

float desired_sail_angle = 0; 
float desired_rudder_angle = 0;


// Initialization --------------------------------------------------------------------------------------------------------------------------------------------

void init_stepper_motor_driver(HighPowerStepperDriver& stepper_driver, int chip_select, float max_current, HPSDStepMode micro_step) {
  SPI.begin();
  stepper_driver.setChipSelectPin(chip_select);
  delay(1);
  stepper_driver.resetSettings();
  stepper_driver.clearStatus();

  stepper_driver.setDecayMode(HPSDDecayMode::AutoMixed);    // Select auto mixed decay.  TI's DRV8711 documentation recommends this mode for most applications, and we find that it usually works well.
  stepper_driver.setCurrentMilliamps36v4(max_current);      // Set the current limit. You should change the number here to an appropriate value for your particular system.
  stepper_driver.setStepMode(micro_step);   // Set the number of microsteps that correspond to one full step.
  
  stepper_driver.enableDriver();    // Enable the motor outputs.
}

void setup() {
  Serial.begin(115200);

  digitalWrite(RUDDER_MOTOR_SLEEP_PIN, HIGH);    // Sleep pin is inverted so when we tie it high, we are telling it to be "awake"
  digitalWrite(MAST_MOTOR_SLEEP_PIN, HIGH);    

  rudder_encoder = new AMT22_Encoder(RUDDER_ENCODER_CHIP_SELECT_PIN);
  mast_encoder = new AMT22_Encoder(MAST_ENCODER_CHIP_SELECT_PIN);

  init_stepper_motor_driver(rudder_stepper_driver, RUDDER_MOTOR_CHIP_SELECT_PIN, MAX_RUDDER_CURRENT, HPSDStepMode::MicroStep1);
  init_stepper_motor_driver(mast_stepper_driver, MAST_MOTOR_CHIP_SELECT_PIN, MAX_MAST_CURRENT, HPSDStepMode::MicroStep4);
}


void clear_serial_buffer() {
  while(Serial.available() > 0) {
    Serial.read();
  }
}


char *str_replace(char *orig, char *rep, char *with) {
  // https://stackoverflow.com/questions/779875/what-function-is-to-replace-a-substring-from-a-string-in-c
  char *result; // the return string
  char *ins;    // the next insert point
  char *tmp;    // varies
  int len_rep;  // length of rep (the string to remove)
  int len_with; // length of with (the string to replace rep with)
  int len_front; // distance between rep and end of last rep
  int count;    // number of replacements

  // sanity checks and initialization
  if (!orig || !rep)
      return NULL;
  len_rep = strlen(rep);
  if (len_rep == 0)
      return NULL; // empty rep causes infinite loop during count
  if (!with)
      with = (char*)"";
  len_with = strlen(with);

  // count the number of replacements needed
  ins = orig;
  for (count = 0; (tmp = strstr(ins, rep)); ++count) {
      ins = tmp + len_rep;
  }

  tmp = result = (char*) malloc(strlen(orig) + (len_with - len_rep) * count + 1);

  if (!result)
      return NULL;

  // first time through the loop, all the variable are set correctly
  // from here on,
  //    tmp points to the end of the result string
  //    ins points to the next occurrence of rep in orig
  //    orig points to the remainder of orig after "end of rep"
  while (count--) {
      ins = strstr(orig, rep);
      len_front = ins - orig;
      tmp = strncpy(tmp, orig, len_front) + len_front;
      tmp = strcpy(tmp, with) + len_with;
      orig += len_front + len_rep; // move to next "end of rep"
  }
  strcpy(tmp, orig);
  return result;
}


float get_sail_angle_from_mast_motor_angle(float mast_motor_angle) {
  // float mast_motor_range = MAX_MAST_MOTOR_ANGLE - MIN_MAST_MOTOR_ANGLE;
  return (mast_motor_angle - WINCH_ZERO_POINT) * 0.08087;
  // return ((mast_motor_angle - MIN_MAST_MOTOR_ANGLE) * 90) / mast_motor_range;
}

// Main Control Loop --------------------------------------------------------------------------------------------------------------------------------------------

void loop() {
  size_t incoming_message_buffer_size = 1000;
  char incoming_message_buffer[incoming_message_buffer_size];
  
  // Get desired mast and rudder angles from the jetson
  if (Serial.available() > 0) {
    Serial.readBytesUntil('\n', incoming_message_buffer, incoming_message_buffer_size);
    clear_serial_buffer();

    // WE KEEP THE MESSAGE AS A CHAR ARRAY INSTEAD OF A STRING TO AVOID HEAP FRAGMENTATION. MCUs generally do not like handling Strings
    char* message = incoming_message_buffer;

    char* sail_angle_message = strtok(message, ";");
    sail_angle_message = str_replace(sail_angle_message, (char*)"mast angle: ", NULL);
    desired_sail_angle = atof(sail_angle_message);
    free(sail_angle_message);

    char* rudder_angle_message = strtok(NULL, ";");
    rudder_angle_message = str_replace(rudder_angle_message, (char*)"rudder angle: ", NULL);
    desired_rudder_angle = fmod(atof(rudder_angle_message), 360);
    free(rudder_angle_message);

    Serial.print("Setting Sail Angle To: ");
    Serial.print(desired_sail_angle); Serial.print("; ");
    Serial.print("Setting Rudder Angle To: ");
    Serial.print(desired_rudder_angle);
  }

  // Get Encoder Values (current positions of the motors)
  current_rudder_angle = rudder_encoder->get_motor_angle();
  current_mast_motor_angle = mast_encoder->get_motor_angle();
  // Serial.println(current_rudder_angle);

  int mast_turn_count = mast_encoder->get_turn_count();
  current_mast_motor_angle += mast_turn_count * 360;

  // current_mast_motor_angle -= MAST_MOTOR_ZERO_POINT;
  // Serial.println(current_mast_motor_angle);
  float current_sail_angle = get_sail_angle_from_mast_motor_angle(current_mast_motor_angle);
  // float current_sail_angle = current_mast_motor_angle;

  // Serial.println(mast_turn_count);
  // Serial.print("Sail angle: "); Serial.println(current_sail_angle);

  if (current_rudder_angle > 180) current_rudder_angle -= 360;
  // if (current_mast_motor_angle > 180) current_mast_motor_angle -= 360;

  // Serial.println(current_sail_angle);
  // Serial.println();
  // Serial.println(current_sail_angle);
  // Closed Feedback Loop
  float rudder_error = current_rudder_angle - desired_rudder_angle;

  // check if the reading is overflow or nan. If so, don't do anything
  if (abs(rudder_error > 1000000) || rudder_error != rudder_error)
    rudder_error = 0;

  if (abs(rudder_error) > ACCEPTABLE_RUDDER_ERROR) {
    if (((int)rudder_error % 360) > 0 && ((int)rudder_error % 360) < 180) {
      rudder_stepper_driver.setDirection(COUNTER_CLOCKWISE);
    }
    
    else {
      rudder_stepper_driver.setDirection(CLOCKWISE);
    }

    // number of steps is some linear function that maps the error of the rudder to a number of steps we want to take per loop.
    // This ends up cooresponding to the speed of the rudder. The higher the rudder_error, the higher the speed of the rudder will be
    int number_of_steps = (int)(abs(rudder_error) * RUDDER_GAIN / MAX_RUDDER_ERROR);  
    if (number_of_steps > 250) {
      Serial.println("Throttling Rudder");
      number_of_steps = 250;
    }

    for (int i = 0; i < number_of_steps; i++) {
      rudder_stepper_driver.step();
      delayMicroseconds(STEP_PERIOD_US);
    }
  }

  float sail_error = current_sail_angle - desired_sail_angle;
  // check if the reading is overflow or nan. If so, don't do anything
  if (abs(sail_error) > 1000000 || sail_error != sail_error) 
    sail_error = 0;

  if (abs(sail_error) > ACCEPTABLE_SAIL_ERROR) {
    if (sail_error > 0) {
      mast_stepper_driver.setDirection(COUNTER_CLOCKWISE);
    }
    
    else {
      mast_stepper_driver.setDirection(CLOCKWISE);
    }

    // number of steps is some linear function that maps the error of the rudder to a number of steps we want to take per loop.
    // This ends up cooresponding to the speed of the rudder. The higher the rudder_error, the higher the speed of the rudder will be
    int number_of_steps = (int)(abs(sail_error) * MAST_GAIN / MAX_SAIL_ERROR);   
    if (number_of_steps > 150) {
      Serial.println("Throttling Sail");
      number_of_steps = 150;
    }

    for (int i = 0; i < number_of_steps; i++) {
      mast_stepper_driver.step();
      delayMicroseconds(STEP_PERIOD_US);
    }
  }
}
