#include <SPI.h>
#include <HighPowerStepperDriver.h>
// #include <AMT22_Encoder.h>

#define RUDDER_MOTOR_SLEEP_PIN 0
#define RUDDER_MOTOR_CHIP_SELECT_PIN 1 
#define MAST_MOTOR_SLEEP_PIN 7
#define MAST_MOTOR_CHIP_SELECT_PIN 8

// #define RUDDER_ENCODER_CHIP_SELECT_PIN 10
// #define MAST_ENCODER_CHIP_SELECT_PIN 8

#define CLOCKWISE 1
#define COUNTER_CLOCKWISE 0

#define MAX_RUDDER_CURRENT 2000
#define MAX_MAST_CURRENT 2000

// This is measured in number of steps before you check the encoder and jetson serial values. Encoder and jetson serial have inherent delay. 
// So the more steps you do before you check the encoder and jetson serial, the faster you go, but the harder it is to control
// very similar to p controller gain (Kp)
#define RUDDER_GAIN 100
#define MAST_GAIN 50

#define MAX_RUDDER_ANGLE 20
#define MIN_RUDDER_ANGLE -20
#define MAX_MAST_ANGLE 20
#define MIN_MAST_ANGLE -20

// #define INVERTED_CONTROLS true
// #define RUDDER_ANGLE_OFFSET 36
// #define MAST_ANGLE_OFFSET 0

const float MID_RUDDER_ANGLE = (MAX_RUDDER_ANGLE + MIN_RUDDER_ANGLE) / 2;
const float MID_MAST_ANGLE = (MAX_MAST_ANGLE + MIN_MAST_ANGLE) / 2;

const int MAX_RUDDER_ERROR = (MAX_RUDDER_ANGLE - MIN_RUDDER_ANGLE);
const int MAX_MAST_ERROR = (MAX_MAST_ANGLE - MIN_RUDDER_ANGLE);

const float STEP_SIZE_DEGREES = 0.03277494537; // (1.8/(4 * 13.73));

#define ACCEPTABLE_RUDDER_ERROR 1   // in degrees
#define ACCEPTABLE_MAST_ERROR 1     // in degrees


// This period is the length of the delay between steps, which controls the
// stepper motor's speed.  You can increase the delay to make the stepper motor
// go slower.  If you decrease the delay, the stepper motor will go faster, but
// there is a limit to how fast it can go before it starts missing steps.
#define STEP_PERIOD_US 2000

HighPowerStepperDriver rudder_stepper_driver;
HighPowerStepperDriver mast_stepper_driver;
// AMT22_Encoder* rudder_encoder;
// AMT22_Encoder* mast_encoder;

float current_rudder_angle = 0; 
float current_mast_angle = 0;

float desired_mast_angle = 0; 
float desired_rudder_angle = 0;


// Initialization --------------------------------------------------------------------------------------------------------------------------------------------

void init_stepper_motor_driver(HighPowerStepperDriver& stepper_driver, int chip_select, float max_current) {
  SPI.begin();
  stepper_driver.setChipSelectPin(chip_select);
  delay(1);
  stepper_driver.resetSettings();
  stepper_driver.clearStatus();

  stepper_driver.setDecayMode(HPSDDecayMode::AutoMixed);    // Select auto mixed decay.  TI's DRV8711 documentation recommends this mode for most applications, and we find that it usually works well.
  stepper_driver.setCurrentMilliamps36v4(max_current);      // Set the current limit. You should change the number here to an appropriate value for your particular system.
  stepper_driver.setStepMode(HPSDStepMode::MicroStep4);   // Set the number of microsteps that correspond to one full step.
  
  stepper_driver.enableDriver();    // Enable the motor outputs.
}

void setup() {
  Serial.begin(9600);
  pinMode(RUDDER_MOTOR_SLEEP_PIN, OUTPUT);
  pinMode(MAST_MOTOR_SLEEP_PIN, OUTPUT);


  digitalWrite(RUDDER_MOTOR_SLEEP_PIN, HIGH);    // Sleep pin is inverted so when we tie it high, we are telling it to be "awake"
  digitalWrite(MAST_MOTOR_SLEEP_PIN, HIGH);    

  // rudder_encoder = new AMT22_Encoder(RUDDER_ENCODER_CHIP_SELECT_PIN);
  // mast_encoder = new AMT22_Encoder(MAST_ENCODER_CHIP_SELECT_PIN);
  init_stepper_motor_driver(rudder_stepper_driver, RUDDER_MOTOR_CHIP_SELECT_PIN, MAX_RUDDER_CURRENT);
  init_stepper_motor_driver(mast_stepper_driver, MAST_MOTOR_CHIP_SELECT_PIN, MAX_MAST_CURRENT);
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

    char* mast_angle_message = strtok(message, ";");
    mast_angle_message = str_replace(mast_angle_message, (char*)"mast angle: ", NULL);
    desired_mast_angle = fmod(atof(mast_angle_message), 360);
    free(mast_angle_message);

    char* rudder_angle_message = strtok(NULL, ";");
    rudder_angle_message = str_replace(rudder_angle_message, (char*)"rudder angle: ", NULL);
    desired_rudder_angle = fmod(atof(rudder_angle_message), 360);
    free(rudder_angle_message);

    Serial.print("Setting Mast Angle To: ");
    Serial.print(desired_mast_angle); Serial.print("; ");
    Serial.print("Setting Rudder Angle To: ");
    Serial.print(desired_rudder_angle);
  }


  // Get Encoder Values (current positions of the motors)
  // float current_rudder_angle = rudder_encoder->get_motor_angle();
  // float current_mast_angle = mast_encoder->get_motor_angle();


  // Closed Feedback Loop
  float rudder_error = current_rudder_angle - desired_rudder_angle;
  int rudder_direction;

  if (abs(rudder_error) > ACCEPTABLE_RUDDER_ERROR) {
    if (((int)rudder_error % 360) > 0 && ((int)rudder_error % 360) < 180) {
      rudder_stepper_driver.setDirection(COUNTER_CLOCKWISE);
      rudder_direction = COUNTER_CLOCKWISE;
    }
    
    else {
      rudder_stepper_driver.setDirection(CLOCKWISE);
      rudder_direction = CLOCKWISE;
    }

    // number of steps is some linear function that maps the error of the rudder to a number of steps we want to take per loop.
    // This ends up cooresponding to the speed of the rudder. The higher the rudder_error, the higher the speed of the rudder will be
    int number_of_steps = (int)(abs(rudder_error) * RUDDER_GAIN / MAX_RUDDER_ERROR);   
    for (int i = 0; i < number_of_steps; i++) {
      rudder_stepper_driver.step();
      delayMicroseconds(STEP_PERIOD_US);

      if (rudder_direction == CLOCKWISE) current_rudder_angle = fmod(current_rudder_angle + STEP_SIZE_DEGREES, 360.0);
      if (rudder_direction == COUNTER_CLOCKWISE) current_rudder_angle = fmod(current_rudder_angle - STEP_SIZE_DEGREES, 360.0);
    }
  }


  float mast_error = current_mast_angle - desired_mast_angle;
  int mast_direction;

  if (abs(mast_error) > ACCEPTABLE_MAST_ERROR) {
    if (((int)mast_error % 360) > 0 && ((int)mast_error % 360) < 180) {
      mast_stepper_driver.setDirection(COUNTER_CLOCKWISE);
      mast_direction = COUNTER_CLOCKWISE;
    }
    
    else {
      mast_stepper_driver.setDirection(CLOCKWISE);
      mast_direction = CLOCKWISE;
    }

    // number of steps is some linear function that maps the error of the rudder to a number of steps we want to take per loop.
    // This ends up cooresponding to the speed of the rudder. The higher the rudder_error, the higher the speed of the rudder will be
    int number_of_steps = (int)(abs(mast_error) * MAST_GAIN / MAX_MAST_ERROR);   
    for (int i = 0; i < number_of_steps; i++) {
      mast_stepper_driver.step();
      delayMicroseconds(STEP_PERIOD_US);

      if (mast_direction == CLOCKWISE) current_mast_angle = fmod(current_mast_angle + STEP_SIZE_DEGREES, 360.0);
      if (mast_direction == COUNTER_CLOCKWISE) current_mast_angle = fmod(current_mast_angle - STEP_SIZE_DEGREES, 360.0);
    }
  }
}
