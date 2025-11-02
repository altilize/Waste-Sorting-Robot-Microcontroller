void calculate_position() {
  pos_x = Odometry1 / 10;
  pos_y = Odometry2 / 10;
}

void read_compass() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  float absoluteHeading = orientationData.orientation.x;
  
  heading = absoluteHeading - headingOffset;
  
  if (heading > 180) {
    heading -= 360;
  } else if (heading < -180) {
    heading += 360;
  }
}
