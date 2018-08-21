void do_test(void)
{
  Serial.println("Stand");
  stand();
  delay(2000);
  
  //  Serial.println("Step forward");
  //  step_forward(5);
  //  delay(2000);
  //  Serial.println("Step back");
  //  step_back(5);
  //  delay(2000);
  //  Serial.println("Turn left");
  //  turn_left(5);
  //  delay(2000);
  //  Serial.println("Turn right");
  //  turn_right(5);
  //  delay(2000);
  //  Serial.println("Hand wave");
  //  hand_wave(3);
  //  delay(2000);
  //  Serial.println("Hand wave");
  //  hand_shake(3);
  //  delay(2000);
  //  Serial.println("Sit");
  //  sit();

  Serial.println("My turn");
  turn_left(1);
  
  delay(5000);
}
