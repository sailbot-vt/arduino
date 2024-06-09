#include <SPI.h>
#include <AMT22_Encoder.h>

AMT22_Encoder* encoder;


void setup() {
  encoder = new AMT22_Encoder(19);
  Serial.begin(9600);
  
  encoder->zero_encoder_value();
  // delay(25000);
  // encoder->zero_encoder_value();
  // encoder->zero_encoder_value();
  // encoder->zero_encoder_value();
  // encoder->zero_encoder_value();
}


void loop() {
  Serial.println(encoder->get_motor_angle());
}

