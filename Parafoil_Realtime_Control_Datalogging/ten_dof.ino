//void initSensors() {
//  Serial.println("Sensors Initializing");
//  if (!accel.begin()) {
//    /* There was a problem detecting the LSM303 ... check your connections */
//    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
//    while (1);
//  }
//  if (!mag.begin()) {
//    /* There was a problem detecting the LSM303 ... check your connections */
//    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
//    while (1);
//  }
//  if (!bmp.begin()) {
//    /* There was a problem detecting the BMP180 ... check your connections */
//    Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
//    while (1);
//  }
//  Serial.println("Sensors Initialized");
//  sensors_event_t accel_event;
//  sensors_vec_t   orientation;
//}

//void logRoll(){
//  if((flightTime() - most_recent_log) > 100){
//    sensors_event_t accel_event;
//    sensors_vec_t   orientation;
//    accel.getEvent(&accel_event);
//    if (dof.accelGetOrientation(&accel_event, &orientation))
//    {
//      short roll = (short)(orientation.roll*10);
//      Serial.print(F("Roll: "));
//      Serial.print(roll);
//      Serial.print(F("; "));
//      Serial.println(F(""));
//      EEPROM_writeAnything((1+log_number) * 2, roll);
//      most_recent_log = flightTime();
//      log_number++;
//    } 
//  }
//}

