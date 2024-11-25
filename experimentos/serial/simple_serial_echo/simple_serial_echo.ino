void setup() {
  Serial.begin(115200); // Initialize serial communication at 115200 baud
}

void loop() {
  if (Serial.available()) {
    int incomingByte = Serial.read(); // Read the incoming byte
    Serial.write(incomingByte);       // Echo the byte back
  }
}
