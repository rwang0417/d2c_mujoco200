// Use pwm to control the 4 motors
int pwm[4] = {7,3,6,5};
double level = 0.5; // throttle ratio

void setup() {
  for (int i = 0 ; i < 4; i++) {
    // 500Hz, motor stop: 1.06ms, motor full: 1.86ms
    // sets the value (range from 0 to 255):
    analogWrite(pwm[i], 240);
  }
  delay(2000);
  for (int i = 0 ; i < 4; i++) analogWrite(pwm[i], 160);
  delay(2000);
  for (int i = 0 ; i < 4; i++) analogWrite(pwm[i], 80*level+160);
}

void loop() {
}
