void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
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


void loop() {
  double desired_mast_angle = 0; double desired_rudder_angle = 0;
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
    desired_mast_angle = atof(mast_angle_message);
    free(mast_angle_message);

    char* rudder_angle_message = strtok(NULL, ";");
    rudder_angle_message = str_replace(rudder_angle_message, (char*)"rudder angle: ", NULL);
    desired_rudder_angle = atof(rudder_angle_message);
    free(rudder_angle_message);

    Serial.print("Setting Mast Angle To: ");
    Serial.print(desired_mast_angle); Serial.print("; ");
    Serial.print("Setting Rudder Angle To: ");
    Serial.print(desired_rudder_angle);
  }
}
