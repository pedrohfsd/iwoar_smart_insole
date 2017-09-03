// This #include statement was automatically added by the Particle IDE.
#include <SparkFun_MPL3115A2.h>

// This #include statement was automatically added by the Particle IDE.
#include <SparkFunLSM9DS1.h>

// This #include statement was automatically added by the Particle IDE.
#include "math.h"
#include "Particle.h"

char resultStr[352]; //String to store the sensor data
char resultGyro[48];
char resultAccel[48];
char resultMagn[48];
char resultAForce[48];
char resultFsr[128];
char resultBaroA[16];
char resultSonar[16];

//SONAR
const unsigned int MAX_DIST = 23200;
//SONAR Pins
int TRIG_PIN = D5;
int ECHO_PIN = D4;

//Barometer
MPL3115A2 baro = MPL3115A2();//create instance of MPL3115A2 barrometric sensor

//////////////////////////
//    UDP WIFI SETUP    //
//////////////////////////
//PARTICLE
SYSTEM_THREAD(ENABLED);

// How often to send samples in milliseconds
const unsigned long SEND_PERIOD_MS = 50;

// IP address and port of the server. Note that the node server uses two ports - one for the web browser
IPAddress serverAddress(139,82,71,156);
int serverPort = 8081;

// Finite state machine states
enum {CONNECT_STATE, SEND_DATA_STATE};

TCPClient client;
unsigned long lastSend = 0;
int state = CONNECT_STATE;

//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu;

///////////////////////
// Example I2C Setup //
///////////////////////
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M	0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG	0x6B // Would be 0x6A if SDO_AG is LOW

////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 1000 // 250 ms between prints

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

//FSR EXAMPLE : https://learn.adafruit.com/force-sensitive-resistor-fsr/using-an-fsr
//int fsrPin = 0;     // the FSR and 10K pulldown are connected to a0

void setup() {

  // Print your device IP Address via serial
  Serial.begin(9600);
  Serial.println(WiFi.localIP());
  
  //SONAR The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  
  //Initialize
  while(!baro.begin()) {
    Serial.println("MPL3115A2 not found");
    delay(1000);
  }
  //Serial.println("MPL3115A2 OK");
  //MPL3115A2 Settings
  baro.setModeAltimeter();//Set to altimeter Mode
  baro.setOversampleRate(7); // Set Oversample to the recommended 128
  baro.enableEventFlags(); //Necessary register calls to enble temp, baro ansd alt
  
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  //The above lines will only take effect AFTER calling
  //imu.begin(), which verifies communication with the IMU
  //and turns it on.
  if (!imu.begin()) {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1);
  }
}
//---------------------------------------------------------------
void loop() {
  printGyro();  // Print "G: gx, gy, gz"
  printAccel(); // Print "A: ax, ay, az"
  printMagn();   // Print "M: mx, my, mz"
  //Print FSR
  //Serial.println("PRINT FSR");
  printFSR();
  //BAROMETER
  printBaro();
  //SONAR
  printSonar();
  // Print the heading and orientation for fun!
  // Call print attitude. The LSM9DS1's magnetometer x and y
  // axes are opposite to the accelerometer, so my and mx are
  // substituted for each other.
  printAttitude(imu.ax, imu.ay, imu.az, -imu.my, -imu.mx, imu.mz);
  //SEND JSON DATA
  sendTcpData();
  delay(SEND_PERIOD_MS);
}
//---------------------------------------------------------------
void printGyro() {
    // To read from the gyroscope, you must first call the
    // readGyro() function. When this exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
    float x = imu.calcGyro(imu.gx);
    float y = imu.calcGyro(imu.gy);
    float z = imu.calcGyro(imu.gz);
    //create json result
    sprintf(resultGyro, "{\"x\":%f,\"y\":%f,\"z\":%f}", x, y, z);
}
//---------------------------------------------------------------
void printAccel() {
    // To read from the accelerometer, you must first call the
    // readAccel() function. When this exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
    float x = imu.calcAccel(imu.ax);
    float y = imu.calcAccel(imu.ay);
    float z = imu.calcAccel(imu.az);
    //create json result
    sprintf(resultAccel, "{\"x\":%f,\"y\":%f,\"z\":%f}", x, y, z);
}
//---------------------------------------------------------------
void printMagn() {
    // To read from the magnetometer, you must first call the
    // readMag() function. When this exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
    float x = imu.calcMag(imu.mx);
    float y = imu.calcMag(imu.my);
    float z = imu.calcMag(imu.mz);
    //create json result
    sprintf(resultMagn, "{\"x\":%f,\"y\":%f,\"z\":%f}", x, y, z);
}
//---------------------------------------------------------------
// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(float ax, float ay, float az, float mx, float my, float mz) {
    //char parcialResultStr[128];
    float roll = atan2(ay, az);
    float pitch = atan2(-ax, sqrt(ay * ay + az * az));
    float heading;
    
    if (my == 0)
        heading = (mx < 0) ? 180.0 : 0;
    else
        heading = atan2(mx, my);
    
    heading -= DECLINATION * M_PI / 180;
    
    if (heading > M_PI) heading -= (2 * M_PI);
    else if (heading < -M_PI) heading += (2 * M_PI);
    else if (heading < 0) heading += 2 * M_PI;
    
    // Convert everything from radians to degrees:
    heading *= 180.0 / M_PI;
    pitch *= 180.0 / M_PI;
    roll  *= 180.0 / M_PI;
    //create json result
    sprintf(resultAForce, "{\"r\":%f,\"p\":%f,\"h\":%f}", roll, pitch, heading);
}
//---------------------------------------------------------------
//GET ANALOG FSR
void printFSR () {
    // the analog reading from the FSR resistor divider
    int fsr0Reading = analogRead(0);
    //Serial.print("FSR1 ");
    //Serial.println(fsr0Reading);
    int fsr1Reading = analogRead(1);
    //Serial.print("FSR2 ");
    //Serial.println(fsr1Reading);
    int fsr2Reading = analogRead(2);
    //Serial.print("FSR3 ");
    //Serial.println(fsr2Reading);
    int fsr3Reading = analogRead(3);
    //Serial.print("FSR4 ");
    //Serial.println(fsr3Reading);
    int fsr4Reading = analogRead(4);
    //Serial.print("FSR5 ");
    //Serial.println(fsr4Reading);
    int fsr5Reading = analogRead(5);
    //Serial.print("FSR6 ");
    //Serial.println(fsr5Reading);
    //create json result
    sprintf(resultFsr, "{\"fsr0\":%d,\"fsr1\":%d,\"fsr2\":%d,\"fsr3\":%d,\"fsr4\":%d,\"fsr5\":%d}", fsr0Reading, fsr1Reading, fsr2Reading, fsr3Reading, fsr4Reading, fsr5Reading);
}
//---------------------------------------------------------------
void printBaro() {
    //float baroTemp = baro.readTempF();//get the temperature in F
    //float pascals = baro.readPressure();//get pressure in Pascals
    //float baroAltf = baro.readAltitudeFt();//get altitude in feet
    float baroAltf = baro.readAltitudeFt();
    //create json result
    sprintf(resultBaroA,"%f", baroAltf);
}
//---------------------------------------------------------------
void printSonar() {
    
    unsigned long t1;
    unsigned long t2;
    unsigned long pulse_width;
    float cm;
    float inches;
    // Hold the trigger pin high for at least 10 us
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Wait for pulse on echo pin
    while (digitalRead(ECHO_PIN) == 0 );

    // Measure how long the echo pin was held high (pulse width)
     // Note: the micros() counter will overflow after ~70 min
    t1 = micros();
    while (digitalRead(ECHO_PIN) == 1);
    t2 = micros();
    pulse_width = t2 - t1;

    // Calculate distance in centimeters and inches. The constants
    // are found in the datasheet, and calculated from the assumed speed 
    //of sound in air at sea level (~340 m/s).
    cm = pulse_width / 58.0;
    //inches = pulse_width / 148.0;

    // Print out results
    if ( pulse_width > MAX_DIST ) {
        //Serial.println("Out of range");
        sprintf(resultSonar,"%f", 0.0);
    } else {
        Serial.print(cm);
        //create json result
        sprintf(resultSonar,"%f", cm);
        //Serial.print(" cm \t");
        //Serial.print(inches);
        //Serial.println(" in");
    }
}
//---------------------------------------------------------------
void sendTcpData() {
	switch(state) {
    	case CONNECT_STATE:
    		Serial.println("connecting...");
    		if (client.connect(serverAddress, serverPort)) {
    			state = SEND_DATA_STATE;
    		} else {
    			Serial.println("connection failed");
    			delay(15000);
    		}
        break;
    	case SEND_DATA_STATE:
    		if (client.connected()) {
    			// Discard any incoming data; there shouldn't be any
    			while(client.available()) {
    				client.read();
    			}
    
    			// Send data up to the server
    			if (millis() - lastSend >= SEND_PERIOD_MS) {
    				lastSend = millis();
    
    				// analogRead returns 0 - 4095; remove the low bits so we got 0 - 255 instead.
    				//int val = analogRead(INPUT_PIN) >> 4;
    				sprintf(resultStr, "{\"g\":%s,\"a\":%s,\"m\":%s,\"af\":%s,\"fsr\":%s,\"b\":%s,\"d\":%s}", resultGyro, resultAccel, resultMagn, resultAForce, resultFsr, resultBaroA, resultSonar);
                    client.write(resultStr);
    				//client.write((unsigned char)val);
    			}
    		} else {
    			// Disconnected
    			Serial.println("disconnected...");
    			client.stop();
    			state = CONNECT_STATE;
    			delay(5000);
    		}
    	break;
	}
}
//---------------------------------------------------------------