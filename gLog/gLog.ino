/*
    2013-10-10
    Francesco Meschia
    
    gLog is an acceleration logger based on the OpenLog hardware and the Analog Devices ADXL345
    3-axis accelerometer. Features include:
    
    * on start-up, gLog samples the gravity vector and assumes it to be the Z axis in a cartesian
      system with Y aligned with the direction of motion. All logging will be performed in this
      reference frame. In this way, gLog can be mounted on a vehicle with arbitrary positioning 
      in pitch and roll, only requiring Y to point in the forward direction.
      
    * gLog logs to a microSD card inserted in the microSD slot of the OpenLog hardware board.
    
*/

#include <Wire.h>
#include <SdFat.h>
#include <EEPROM.h>
#include <I2Cdev.h>
#include <ADXL345.h>
//#include <SerialPort.h>
#include <avr/sleep.h> //Needed for sleep_mode
#include <avr/power.h> //Needed for powering down perihperals such as the ADC/TWI and Timers

//SerialPort<0, 780, 0> NewSerial;

ADXL345 accel;

#define BUF_SIZE 50

int16_t ax, ay, az;
int16_t buf_x[BUF_SIZE], buf_y[BUF_SIZE], buf_z[BUF_SIZE];
int16_t filt_x[BUF_SIZE], filt_y[BUF_SIZE], filt_z[BUF_SIZE];
int8_t buf_ptr, buf_ptr_old;
int8_t state;
//int8_t filter_old = 0;
//int8_t buf_y_old = 0;

bool calibrate = false;
float g;
long time, startTime, lastTime;
long launchTime, zoomTime, landTime;
long interval PROGMEM = 125L;
long ZOOM_RELAX_TIME PROGMEM = 3000L;
long RELAX_TIME PROGMEM = 5000L;
long INITIAL_DELAY = 15250L;
float st, ct, sp, cp;
float axr, ayr, azr;
const float alpha PROGMEM = 0.9;
const float alpha1 PROGMEM = 0.3;

Sd2Card card;
SdVolume volume;
SdFile currentDirectory;
SdFile workingFile;
char logFileName[13];

#define STATE_PRELAUNCH 0
#define STATE_TOW       1
#define STATE_FLIGHT    2
#define STATE_LANDED    3

#define DEBUG 1
#define CALIBRATE  1
#define LOCATION_OFFSET_X               0x10
#define LOCATION_OFFSET_Y               0x11
#define LOCATION_OFFSET_Z               0x12
#define LOCATION_FILE_NUMBER_LSB	0x03
#define LOCATION_FILE_NUMBER_MSB	0x04
#define LED_PIN 13 
const byte statled1 PROGMEM = 5;
const byte statled2 PROGMEM = LED_PIN;
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

/*
extern int __bss_end;
extern void *__brkval;

int get_free_memory()
{
  int free_memory;

  if((int)__brkval == 0)
    free_memory = ((int)&free_memory) - ((int)&__bss_end);
  else
    free_memory = ((int)&free_memory) - ((int)__brkval);

  return free_memory;
}
*/

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

void systemError(byte error_type)
{
  Serial.print(F("Error "));
  switch(error_type)
  {
  case ERROR_CARD_INIT:
    Serial.print(F("card.init")); 
    blink_error(ERROR_SD_INIT);
    break;
  case ERROR_VOLUME_INIT:
    Serial.print(F("volume.init")); 
    blink_error(ERROR_VOLUME_INIT);
    break;
  case ERROR_ROOT_INIT:
    Serial.print(F("root.init")); 
    blink_error(ERROR_ROOT_INIT);
    break;
  case ERROR_FILE_OPEN:
    Serial.print(F("file.open")); 
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
    Serial.print(F("!Too many logs:1!"));
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
        Serial.print(F("Using file ")); Serial.println(new_file_name);
        return;  // Use existing empty file.
      }
      newFile.close(); // Close this existing file we just opened.
    }

    //Try the next number
    new_file_number++;
    if(new_file_number > 65533) //There is a max of 65534 logs
    {
      Serial.print(F("!Too many logs:2!"));
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
  Serial.print(F("Created new file: "));
  Serial.println(new_file_name);
#endif
  Serial.print(F("Using file ")); Serial.println(new_file_name);
  //  append_file(new_file_name);
  return;
}

void setup() {
  
  float ix, iy, iz;
  float ox, oy, oz;
  int8_t nox, noy, noz;
  
  time = 0L;
  
  pinMode(statled1, OUTPUT);
  pinMode(statled2, OUTPUT);
  
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Serial.begin(9600);
  Serial.println(F("Hello"));
  
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
   
  if (!card.init(SPI_FULL_SPEED)) {
    Serial.println(F("No SD card - performing calibration"));
    calibrate = true;
  } 

  // initialize ADXL345 device
  accel.initialize();
  //workingFile.println(F("ADXL345 initialized"));
  digitalWrite(statled1,HIGH); 
  //workingFile.println(accel.testConnection() ? F("ADXL345 connection successful") : F("ADXL345 connection failed"));
  accel.setRate(0x0a);
  delay(100);
  accel.getAcceleration(&ax, &ay, &az);
  delay(100);

  if (calibrate) {
    // calibrate
    accel.setOffset(0,0,0);
    delay(100);
    average_acc(&ix, &iy, &iz, 20);
    Serial.print(F("Accel0 X: ")); Serial.println(ix);
    Serial.print(F("Accel0 Y: ")); Serial.println(iy);
    Serial.print(F("Accel0 Z: ")); Serial.println(iz); 
    ox = 0.0; oy = 0.0; oz = 64.0;
    nox = (int8_t)(ox-ix/4); noy = (int8_t)(oy-iy/4); noz = int8_t((oz-iz/4));
    EEPROM.write(LOCATION_OFFSET_X, nox);
    EEPROM.write(LOCATION_OFFSET_Y, noy);
    EEPROM.write(LOCATION_OFFSET_Z, noz);
    blink_error(3);
  }
   
  nox = EEPROM.read(LOCATION_OFFSET_X);
  noy = EEPROM.read(LOCATION_OFFSET_Y);
  noz = EEPROM.read(LOCATION_OFFSET_Z);

  accel.setOffset(nox, noy, noz);
  accel.setRange(0x03); // full 16*g range
  accel.setFullResolution(1); // 13 bit resolution

  Serial.print(F("New Offset X: ")); 
  Serial.println(nox);
  Serial.print(F("New Offset Y: ")); 
  Serial.println(noy);
  Serial.print(F("New Offset Z: ")); 
  Serial.println(noz);

  average_acc(&ix, &iy, &iz, 20);
  digitalWrite(statled1,LOW); 
  delay(100);
  digitalWrite(statled1,HIGH); 
  Serial.print(ix*1.0/G_SCALE); 
  Serial.print("\t");
  Serial.print(iy*1.0/G_SCALE); 
  Serial.print("\t");
  Serial.println(iz*1.0/G_SCALE); 
  double phi = atan2(-ix, iz);
  double theta = atan2(iy,sqrt(square(ix)+square(iz)));
  //sp = sin(-phi); 
  //cp = cos(-phi);
  //st = sin(-theta); 
  //ct = cos(-theta);
  sp = -ix / sqrt(square(ix)+square(iz));
  cp = iz / sqrt(square(ix)+square(iz));
  st = iy / sqrt(square(ix)+square(iy)+square(iz));
  ct = sqrt(square(ix)+square(iz)) / sqrt(square(ix)+square(iy)+square(iz));
  
  delay(5000L);
  if (!volume.init(&card)) systemError(ERROR_VOLUME_INIT);
  currentDirectory.close(); //We close the cD before opening root. This comes from QuickStart example. Saves 4 bytes.
  if (!currentDirectory.openRoot(&volume)) systemError(ERROR_ROOT_INIT);
  newlog(logFileName);
  Serial.println(logFileName);
  if (!workingFile.open(&currentDirectory, logFileName, O_CREAT | O_APPEND | O_WRITE)) systemError(ERROR_FILE_OPEN);
  if (workingFile.fileSize() == 0) {
    //This is a trick to make sure first cluster is allocated - found in Bill's example/beta code
    //workingFile.write((byte)0); //Leaves a NUL at the beginning of a file
    workingFile.rewind();
    workingFile.sync();
  } 
  
  workingFile.print(F("Roll  : ")); 
  workingFile.println(phi*RAD_TO_DEG);
  workingFile.print(F("Pitch  : ")); 
  workingFile.println(theta*RAD_TO_DEG);
  
  Serial.print(F("Roll  : ")); 
  Serial.println(phi*RAD_TO_DEG);
  Serial.print(F("Pitch  : ")); 
  Serial.println(theta*RAD_TO_DEG);
  
  workingFile.sync();   
//  Serial.print(F("Free SRAM: ")); Serial.print(freeRam()); Serial.println(F(" bytes"));
  int i;
  for ( i=0; i<BUF_SIZE; i++) {
    buf_x[i] = 0;
    buf_y[i] = 0;
    buf_z[i] = 0;
  }

  Serial.print(F("Free SRAM: ")); Serial.print(freeRam()); Serial.println(F(" bytes"));
  while ((millis()-time) < INITIAL_DELAY) {
    delay(50);
  } 
  digitalWrite(statled1,LOW);
  lastTime = -1L;
}

void average_acc(float *ax, float *ay, float *az, int samples) {
  *ax = 0.0;
  *ay = 0.0;
  *az = 0.0;
  int16_t ix, iy, iz;
  int i;
#if DEBUG
  Serial.println(F("Averaging acceleration"));
#endif
  for (i=0; i<samples; i++) {
    accel.getAcceleration(&ix, &iy, &iz);
    Serial.print(F("ix=")); Serial.print(ix); Serial.print(F("\t"));
    Serial.print(F("iy=")); Serial.print(iy); Serial.print(F("\t"));
    Serial.print(F("iz=")); Serial.print(iz); Serial.println(F("\t"));
    *ax += ix; *ay += iy; *az += iz;
  }
  *ax /= samples; *ay /= samples; *az /= samples;
#if DEBUG
  Serial.print(F("ax=")); Serial.print(*ax); Serial.print(F("\t"));
  Serial.print(F("ay=")); Serial.print(*ay); Serial.print(F("\t"));
  Serial.print(F("az=")); Serial.print(*az); Serial.println(F("\t"));
#endif
}


void moving_average(int16_t *ax, int16_t *ay, int16_t *az, int16_t *sigmax, int16_t *sigmay, int16_t *sigmaz, int8_t nsamples, int8_t start=0) {
  int8_t i, ptr;
  long sumx = 0L, sumy = 0L, sumz = 0L;
  //for (i=0, ptr=buf_ptr; i>nsamples; i++) {
  //  ptr--; if (ptr < 0) ptr=BUF_SIZE-1;
  //}
  for (i=0, ptr=buf_ptr; i<nsamples; i++) {
      sumx += buf_x[ptr]; sumy += buf_y[ptr] ; sumz += buf_z[ptr];
      ptr--; if (ptr < 0) ptr=BUF_SIZE-1;
  }
  *ax = (int8_t)(sumx/nsamples); 
  *ay = (int8_t)(sumy/nsamples); 
  *az = (int8_t)(sumz/nsamples);
  sumx = 0L; sumy = 0L; sumz = 0L;
  for (i=0, ptr=buf_ptr; i<nsamples; i++) {
      sumx += (buf_x[ptr]-*ax)*(buf_x[ptr]-*ax); 
      sumy += (buf_y[ptr]-*ay)*(buf_y[ptr]-*ay);
      sumz += (buf_z[ptr]-*az)*(buf_z[ptr]-*az);
      ptr--; if (ptr < 0) ptr=BUF_SIZE-1;
  }
  *sigmax = (int16_t)(sumx/nsamples);
  *sigmay = (int16_t)(sumy/nsamples);
  *sigmaz = (int16_t)(sumz/nsamples);
}


void loop() {
  int16_t ix, iy, iz;
  int16_t avg_x, avg_y, avg_z;
  int16_t sigma_x, sigma_y, sigma_z;
  int8_t i, ptr, ptrExtr_y;
  int16_t tot, peak = 0;
  float avg_g;
  if (lastTime == -1L) {
    state = STATE_PRELAUNCH;
    workingFile.println(F("Timer reset"));
    workingFile.println(F("Time\tax\tay\taz\ta\tavg_x\tavg_y\tavg_z\tsigma2_x\tsigma2_y\tsigma2_z\tfilter_x\tfilter_y\tfilter_z"));
    lastTime = millis();
    startTime = lastTime/interval*interval;
    lastTime -= 2*interval;
  }
  if ((time=millis())-lastTime >= interval) {
    digitalWrite(statled1, LOW);
    lastTime = time/interval*interval;
    // read raw accel measurements from device
    
    accel.getAcceleration(&ix, &iy, &iz);
    ax = ix * alpha + (ax * (1.0 - alpha));
    ay = iy * alpha + (ay * (1.0 - alpha));
    az = iz * alpha + (az * (1.0 - alpha));

    //axr = ax*ct - (-ay*sp + az*cp)*st;  
    //ayr = ay*cp + az*sp;  
    //azr = ax*st + (-ay*sp + az*cp)*ct;
    
    //axr = ax*cp + sp * (ay*st + az*ct);
    //ayr = ay*ct - az*st;
    //azr = -ax*sp + cp * (ay*st + az*ct);
    
    axr =  ax*cp            + az*sp;
    ayr =  ax*sp*st + ay*ct - az*cp*st;
    azr = -ax*sp*ct + ay*st + az*cp*ct;

    
    g = sqrt((axr/G_SCALE)*(axr/G_SCALE)+(ayr/G_SCALE)*(ayr/G_SCALE)+(azr/G_SCALE)*(azr/G_SCALE));   

    workingFile.print(((lastTime-startTime)/5*5)*0.001,3); 
    workingFile.print(F("\t"));
    
    Serial.print(((lastTime-startTime)/5*5)*0.001,3); 
    Serial.print(F("\t"));
/*
    workingFile.print(ax*1.0/G_SCALE); 
    workingFile.print(F("\t"));
    workingFile.print(ay*1.0/G_SCALE); 
    workingFile.print(F("\t"));
    workingFile.print(az*1.0/G_SCALE); 
    workingFile.print(F("\t"));
*/

    avg_g = sqrt((avg_x*4.0/G_SCALE)*(avg_x*4.0/G_SCALE)+(avg_y*4.0/G_SCALE)*(avg_y*4.0/G_SCALE)+(avg_z*4.0/G_SCALE)*(avg_z*4.0/G_SCALE));
    
    buf_x[buf_ptr] = axr; //(int8_t)(axr > 327677.0 ? 127 : axr < -127.0 ? -127 : axr);
    buf_y[buf_ptr] = ayr; //(int8_t)(ayr > 127.0 ? 127 : ayr < -127.0 ? -127 : ayr);
    buf_z[buf_ptr] = azr; //(int8_t)(azr > 127.0 ? 127 : azr < -127.0 ? -127 : azr);

    filt_x[buf_ptr] = (int8_t)(alpha1*(filt_x[buf_ptr_old] + buf_x[buf_ptr] - buf_x[buf_ptr_old]));
    filt_y[buf_ptr] = (int8_t)(alpha1*(filt_y[buf_ptr_old] + buf_y[buf_ptr] - buf_y[buf_ptr_old]));
    filt_z[buf_ptr] = (int8_t)(alpha1*(filt_z[buf_ptr_old] + buf_z[buf_ptr] - buf_z[buf_ptr_old]));
    
    workingFile.print(axr/G_SCALE); 
    workingFile.print(F("\t")); 
    workingFile.print(ayr/G_SCALE); 
    workingFile.print(F("\t"));
    workingFile.print(azr/G_SCALE); 
    workingFile.print(F("\t"));
    workingFile.print(g);
    workingFile.print(F("\t"));
    
    Serial.print(axr/G_SCALE); 
    Serial.print(F("\t"));
    Serial.print(ayr/G_SCALE); 
    Serial.print(F("\t"));
    Serial.print(azr/G_SCALE); 
    Serial.print(F("\t"));
    Serial.println(g);

    buf_ptr_old = buf_ptr;
    
    moving_average(&avg_x, &avg_y, &avg_z, &sigma_x, &sigma_y, &sigma_z, 10);
    
#if DEBUG
    workingFile.print(avg_x); workingFile.print(F("\t"));
    workingFile.print(avg_y); workingFile.print(F("\t"));
    workingFile.print(avg_z); workingFile.print(F("\t"));
    workingFile.print(sigma_x); workingFile.print(F("\t"));
    workingFile.print(sigma_y); workingFile.print(F("\t"));
    workingFile.print(sigma_z); workingFile.print(F("\t"));
    workingFile.print(filt_x[buf_ptr]); workingFile.print(F("\t"));
    workingFile.print(filt_y[buf_ptr]); workingFile.print(F("\t"));
    workingFile.print(filt_z[buf_ptr]); workingFile.print(F("\t"));
#endif
    
    if (state == STATE_PRELAUNCH) {
      if (ayr/G_SCALE > 2.0) {      
        Serial.println(F("*** LAUNCH DETECTED ***"));
        workingFile.print(F("LAUNCH"));
        digitalWrite(statled1,HIGH);
        launchTime = time;
        state = STATE_TOW;
      }
    } else if (state == STATE_TOW) {
      if (time - launchTime > ZOOM_RELAX_TIME) {
        if (azr/G_SCALE > 2.0) {        
        Serial.println(F("*** ZOOM DETECTED ***"));
        workingFile.print(F("ZOOM"));
        digitalWrite(statled1,HIGH);
        zoomTime = time;
        state = STATE_FLIGHT;
        }
      }
    } else if (state == STATE_FLIGHT) {
      if (time-zoomTime > RELAX_TIME) {
     //   if (((int8_t)(avg_x - axr/4.0)) > 32) {
     //     Serial.println(F("*** LANDING DETECTED ***"));
     //     workingFile.print(F("LANDING"));
     //     digitalWrite(statled1,HIGH);
     //     state = STATE_LANDED;
     //   } else
       if (sigma_y < 5) {
          Serial.println(F("*** LANDED DETECTED ***"));
          workingFile.print(F("LANDED"));
          digitalWrite(statled1,HIGH);
          state = STATE_LANDED;
          /*
          ptr = buf_ptr - 40; if (ptr<0) ptr+=BUF_SIZE;
          filter_old = 0; buf_x_old = 0;
          for (i=0, ; i<40; i++) {
            filter = (filter_old + buf_x[ptr] - buf_x_old);
            if (filter > max_x) {
              ptrExtr_x = 40-i;
              max_x = buf_x[ptr];
            }
            ptr++; if (ptr >= BUF_SIZE) ptr=0;
            filter_old = filter;
            buf_x_old = buf_x[ptr];            
          }
          */
          
         workingFile.println(F("Buffer output:\n")); 
         for (i=0, ptr=buf_ptr; i<40; i++) {
            tot = abs(filt_x[ptr]-avg_x) + abs(filt_y[ptr]-avg_y) + abs(filt_z[ptr]-avg_z);
            workingFile.println(tot);            
            if (tot > peak) {
              ptrExtr_y = i;
              peak = tot;
            }
            ptr--; if (ptr < 0) ptr=BUF_SIZE-1;
          }
          workingFile.print(F("\t ptr_max_y=")); workingFile.print(ptrExtr_y);
        }
      }
    } 

    workingFile.println();
    buf_ptr++; if (buf_ptr>=BUF_SIZE) buf_ptr = 0;


    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

  }
  workingFile.sync();
}

