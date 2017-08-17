/*
 * Shim module to implement wiringPi calls from gateway and map them to Arduino SPI
 * 
 */

int spiSpeed;
int spiMode;

void wiringPiSPISetup (int channel, int speed, int mode) {
  spiSpeed = speed;

  switch(mode) {
    case 1:
      spiMode = SPI_MODE1;
      break;
    case 2:
      spiMode = SPI_MODE2;
      break;
    case 3:
      spiMode = SPI_MODE3;
      break;
    default:
      spiMode = SPI_MODE0;
      break;
  }
}


int wiringPiSPIDataRW(int channel, unsigned char *data, int len) {
  int csn = (int) (channel == 0 ? CSN0 : CSN1);

  SPI.beginTransaction(SPISettings(spiSpeed, MSBFIRST, spiMode));
  
  digitalWrite(csn, LOW);

  while (len--) {
    *data = SPI.transfer(*data);
    data++;
  }

  digitalWrite(csn, HIGH);

  SPI.endTransaction();

  return(1);
}


int wiringPiISR (int pin, int edgeType,  void (*function)(void)) {
  attachInterrupt(pin, function, edgeType);

  return(1);
}

