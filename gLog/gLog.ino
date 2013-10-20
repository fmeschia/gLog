/*
    2013-10-10
    Francesco Meschia
    
    gLog is an acceleration logger based on the OpenLog hardware and the Analog Devices ADXL345
    3-axis accelerometer. Features include:
    
    * on start-up, gLog samples the gravity vector and assumes it to be the Z axis in a cartesian
      system with X aligned with the direction of motion. All logging will be performed in this
      reference frame. In this way, gLog can be mounted on a vehicle with arbitrary positioning 
      in pitch and roll, only requiring X to point in the forward direction.
      
    * gLog logs to a microSD card inserted in the microSD slot of the OpenLog hardware board.
    
*/

#include <Wire.h>
#include <SdFat.h>
#include <EEPROM.h>
#include <I2Cdev.h>
#include <ADXL345.h>
#include <SerialPort.h>
#include <avr/sleep.h> //Needed for sleep_mode
#include <avr/power.h> //Needed for powering down perihperals such as the ADC/TWI and Timers

SerialPort<0, 780, 0> NewSerial;

ADXL345 accel;

int16_t ax, ay, az;
float g;
long time, startTime, lastTime;
long interval = 50L;
double st, ct, sp, cp;
double axr, ayr, azr;
const double alpha = 0.9;

Sd2Card card;
SdVolume volume;
SdFile currentDirectory;
SdFile workingFile;
char logFileName[13];

#define DEBUG 0
#define CALIBRATE  0
#define LOCATION_OFFSET_X               0x10
#define LOCATION_OFFSET_Y               0x11
#define LOCATION_OFFSET_Z               0x12
#define LOCATION_FILE_NUMBER_LSB	0x03
#define LOCATION_FILE_NUMBER_MSB	0x04
#define LED_PIN 13 
const byte statled1 = 5;
const byte statled2 = LED_PIN;
bool blinkState = false;

#define G_SCALE 256

//Blinking LED error codes
#define ERROR_SD_INIT	  3
#define ERROR_NEW_BAUD	  5
#define ERROR_CARD_INIT   6
#define ERROR_VOLUME_INIT 7
#define ERROR_ROOT_INIT   8
#define ERROR_FILE_OPEN   9

void blink_error(byte ERROR_TYPE) {
  while(1) {
    for(int x = 0 ; x < ERROR_TYPE ; x++) {
      digitalWrite(statled1, HIGH);
      delay(200);
      digitalWrite(statled1, LOW);
      delay(200);
    }
    delay(2000);
  }
}

void systemError(byte error_type)
{
  NewSerial.print(F("Error "));
  switch(error_type)
  {
  case ERROR_CARD_INIT:
    NewSerial.print(F("card.init")); 
    blink_error(ERROR_SD_INIT);
    break;
  case ERROR_VOLUME_INIT:
    NewSerial.print(F("volume.init")); 
    blink_error(ERROR_VOLUME_INIT);
    break;
  case ERROR_ROOT_INIT:
    NewSerial.print(F("root.init")); 
    blink_error(ERROR_ROOT_INIT);
    break;
  case ERROR_FILE_OPEN:
    NewSerial.print(F("file.open")); 
    blink_error(ERROR_SD_INIT);
    break;
  }
}

void newlog(char *new_file_name)
{
  byte msb, lsb;
  uint16_t new_file_number;

  SdFile newFile; //This will contain the file for SD writing

  //Combine two 8-bit EEPROM spots into one 16-bit number
  lsb = EEPROM.read(LOCATION_FILE_NUMBER_LSB);
  msb = EEPROM.read(LOCATION_FILE_NUMBER_MSB);

  new_file_number = msb;
  new_file_number = new_file_number << 8;
  new_file_number |= lsb;

  //If both EEPROM spots are 255 (0xFF), that means they are un-initialized (first time OpenLog has been turned on)
  //Let's init them both to 0
  if((lsb == 255) && (msb == 255))
  {
    new_file_number = 0; //By default, unit will start at file number zero
    EEPROM.write(LOCATION_FILE_NUMBER_LSB, 0x00);
    EEPROM.write(LOCATION_FILE_NUMBER_MSB, 0x00);
  }

  //The above code looks like it will forever loop if we ever create 65535 logs
  //Let's quit if we ever get to 65534
  //65534 logs is quite possible if you have a system with lots of power on/off cycles
  if(new_file_number == 65534)
  {
    //Gracefully drop out to command prompt with some error
    NewSerial.print(F("!Too many logs:1!"));
    return; //Bail!
  }

  //If we made it this far, everything looks good - let's start testing to see if our file number is the next available

  //Search for next available log spot
  //char new_file_name[] = "LOG00000.TXT";

  while(1)
  {
    sprintf_P(new_file_name, PSTR("LOG%05d.TXT"), new_file_number); //Splice the new file number into this file name

    //Try to open file, if fail (file doesn't exist), then break
    if (newFile.open(&currentDirectory, new_file_name, O_CREAT | O_EXCL | O_WRITE)) break;

    //Try to open file and see if it is empty. If so, use it.
    if (newFile.open(&currentDirectory, new_file_name, O_READ)) 
    {
      if (newFile.fileSize() == 0)
      {
        newFile.close();        // Close this existing file we just opened.
        NewSerial.print(F("Using file ")); NewSerial.println(new_file_name);
        return;  // Use existing empty file.
      }
      newFile.close(); // Close this existing file we just opened.
    }

    //Try the next number
    new_file_number++;
    if(new_file_number > 65533) //There is a max of 65534 logs
    {
      NewSerial.print(F("!Too many logs:2!"));
      return; //Bail!
    }
  }
  newFile.close(); //Close this new file we just opened

  //Record new_file number to EEPROM
  lsb = (byte)(new_file_number & 0x00FF);
  msb = (byte)((new_file_number & 0xFF00) >> 8);

  EEPROM.write(LOCATION_FILE_NUMBER_LSB, lsb); // LSB

  if (EEPROM.read(LOCATION_FILE_NUMBER_MSB) != msb)
    EEPROM.write(LOCATION_FILE_NUMBER_MSB, msb); // MSB

#if DEBUG
  NewSerial.print(F("Created new file: "));
  NewSerial.println(new_file_name);
#endif
  NewSerial.print(F("Using file ")); NewSerial.println(new_file_name);
  //  append_file(new_file_name);
  return;
}

void setup() {
  
  double ix, iy, iz;
  double ox, oy, oz;
  int8_t nox, noy, noz;
  
  pinMode(statled1, OUTPUT);
  pinMode(statled2, OUTPUT);
  
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  NewSerial.begin(9600);
  NewSerial.println(F("Hello"));
  
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
   
  //Shut off TWI, Timer2, Timer1, ADC
  ADCSRA &= ~(1<<ADEN); //Disable ADC
  ACSR = (1<<ACD); //Disable the analog comparator
  DIDR0 = 0x3F; //Disable digital input buffers on all ADC0-ADC5 pins
  DIDR1 = (1<<AIN1D)|(1<<AIN0D); //Disable digital input buffer on AIN1/0
   
  power_timer1_disable();
  power_timer2_disable();
  power_adc_disable();
   
  if (!card.init(SPI_FULL_SPEED)) systemError(ERROR_CARD_INIT);
  if (!volume.init(&card)) systemError(ERROR_VOLUME_INIT);
  currentDirectory.close(); //We close the cD before opening root. This comes from QuickStart example. Saves 4 bytes.
  if (!currentDirectory.openRoot(&volume)) systemError(ERROR_ROOT_INIT);
  newlog(logFileName);
  NewSerial.println(logFileName);
  if (!workingFile.open(&currentDirectory, logFileName, O_CREAT | O_APPEND | O_WRITE)) systemError(ERROR_FILE_OPEN);
  if (workingFile.fileSize() == 0) {
    //This is a trick to make sure first cluster is allocated - found in Bill's example/beta code
    //workingFile.write((byte)0); //Leaves a NUL at the beginning of a file
    workingFile.rewind();
    workingFile.sync();
  }  

  // initialize ADXL345 device
  accel.initialize();
  workingFile.println(F("ADXL345 initialized"));
  digitalWrite(statled1,HIGH); 
  workingFile.println(accel.testConnection() ? F("ADXL345 connection successful") : F("ADXL345 connection failed"));
  accel.setRate(0x0a);
  delay(100);
  accel.getAcceleration(&ax, &ay, &az);
  delay(100);

#if CALIBRATE
  // calibrate
  accel.setOffset(0,0,0);
  delay(100);
  average_acc(&ix, &iy, &iz, 20);
  NewSerial.print(F("Accel0 X: ")); NewSerial.println(ix);
  NewSerial.print(F("Accel0 Y: ")); NewSerial.println(iy);
  NewSerial.print(F("Accel0 Z: ")); NewSerial.println(iz); 
  ox = 0.0; oy = 0.0; oz = 64.0;
  nox = (int8_t)(ox-ix/4); noy = (int8_t)(oy-iy/4); noz = int8_t((oz-iz/4));
  EEPROM.write(LOCATION_OFFSET_X, nox);
  EEPROM.write(LOCATION_OFFSET_Y, noy);
  EEPROM.write(LOCATION_OFFSET_Z, noz);
#endif
   
  nox = EEPROM.read(LOCATION_OFFSET_X);
  noy = EEPROM.read(LOCATION_OFFSET_Y);
  noz = EEPROM.read(LOCATION_OFFSET_Z);

  accel.setOffset(nox, noy, noz);
  accel.setRange(0x03); // full 16*g range
  accel.setFullResolution(1); // 13 bit resolution

  NewSerial.print(F("New Offset X: ")); 
  NewSerial.println(nox);
  NewSerial.print(F("New Offset Y: ")); 
  NewSerial.println(noy);
  NewSerial.print(F("New Offset Z: ")); 
  NewSerial.println(noz);

  average_acc(&ix, &iy, &iz, 20);
  NewSerial.print(ix*1.0/G_SCALE); 
  NewSerial.print("\t");
  NewSerial.print(iy*1.0/G_SCALE); 
  NewSerial.print("\t");
  NewSerial.println(iz*1.0/G_SCALE); 
  double theta = atan2(-ix,iz);
  double phi = atan2(iy,sqrt(square(ix)+square(iz)));
  sp = sin(-phi); 
  cp = cos(-phi);
  st = sin(-theta); 
  ct = cos(-theta);
  workingFile.print(F("Roll  : ")); 
  workingFile.println(phi*RAD_TO_DEG);
  workingFile.print(F("Pitch  : ")); 
  workingFile.println(theta*RAD_TO_DEG);
  
  workingFile.sync();  
  digitalWrite(statled1,LOW);
  lastTime = -1L;
}

void average_acc(double *ax, double *ay, double *az, int samples) {
  *ax = 0.0;
  *ay = 0.0;
  *az = 0.0;
  int16_t ix, iy, iz;
  int i;
#if DEBUG
  NewSerial.println(F("Averaging acceleration"));
#endif
  for (i=0; i<samples; i++) {
    accel.getAcceleration(&ix, &iy, &iz);
    NewSerial.print(F("ix=")); NewSerial.print(ix); NewSerial.print(F("\t"));
    NewSerial.print(F("iy=")); NewSerial.print(iy); NewSerial.print(F("\t"));
    NewSerial.print(F("iz=")); NewSerial.print(iz); NewSerial.println(F("\t"));
    *ax += ix; *ay += iy; *az += iz;
  }
  *ax /= samples; *ay /= samples; *az /= samples;
#if DEBUG
  NewSerial.print(F("ax=")); NewSerial.print(*ax); NewSerial.print(F("\t"));
  NewSerial.print(F("ay=")); NewSerial.print(*ay); NewSerial.print(F("\t"));
  NewSerial.print(F("az=")); NewSerial.print(*az); NewSerial.println(F("\t"));
#endif
}

void loop() {
  int16_t ix, iy, iz;
  if (lastTime == -1L) {
    workingFile.println(F("Timer reset"));
    lastTime = millis();
    startTime = lastTime/interval*interval;
  }
  if ((time=millis())-lastTime >= interval) {
    lastTime = time/interval*interval;
    // read raw accel measurements from device
    
    accel.getAcceleration(&ix, &iy, &iz);
    ax = ix * alpha + (ax * (1.0 - alpha));
    ay = iy * alpha + (ay * (1.0 - alpha));
    az = iz * alpha + (az * (1.0 - alpha));

    axr = ax*ct - (-ay*sp + az*cp)*st;  
    ayr = ay*cp + az*sp;  
    azr = ax*st + (-ay*sp + az*cp)*ct;

    g = sqrt((axr/G_SCALE)*(axr/G_SCALE)+(ayr/G_SCALE)*(ayr/G_SCALE)+(azr/G_SCALE)*(azr/G_SCALE));

    workingFile.print(((lastTime-startTime)/10*10)*0.001); 
    workingFile.print(F("\t"));
    workingFile.print(ax*1.0/G_SCALE); 
    workingFile.print(F("\t"));
    workingFile.print(ay*1.0/G_SCALE); 
    workingFile.print(F("\t"));
    workingFile.print(az*1.0/G_SCALE); 
    workingFile.print(F("\t"));
    workingFile.print(axr/G_SCALE); 
    workingFile.print(F("\t"));
    workingFile.print(ayr/G_SCALE); 
    workingFile.print(F("\t"));
    workingFile.print(azr/G_SCALE); 
    workingFile.print(F("\t"));
    workingFile.println(g);

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

  }
  workingFile.sync();
}

