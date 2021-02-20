#include "sensorbar.h"
#include<Servo.h>
#include "Wire.h"


#define READ 1 //what value the sensors output when over the line
#define WAIT 300  //wait time in ms for the robot to be lined up to execute a turn
#define CENTER 270 //time it takes to move the unltrasonic sensor away from the bin
#define NEXT 200 //time to spin blocks 90 degrees
#define OBJ 7 //distance in cm from the untrasonic sensor an object must be to trigger the colour sensing system
#define CSR 3  //amount of readings of each colour the colour sensor makes
#define SERVO 40  //servo turn wait time in ms
#define STRAIGHT 120 // servo angle to go straight
#define RIGHT 68 // servo angle to turn left
#define LEFT 167 //servo angle to turn right
#define SMALL_ADJ -5 //small andjustment in the servo in degrees
#define LARGE_ADJ -10 // large adjustment in the servo in degrees
#define ON 185 //on voltage
#define SUPERON 205
#define SLOW 185 //slow motor speed for adjustments
#define OFF 0 //off voltage
#define WHITE 280 //used for determining the colours for the colour sensor
#define white 2
#define trayDelay 650
#define RED 0
#define GREEN 1
#define BLUE 3
#define YELLOW 5
#define IDTHRESHOLD 400 //used for reading the inductive sensor
#define HETHRESHHOLD 550 //used for reading the hall effect sensor
#define OBJ1 35 //defines object 1
#define OBJ2 80 //defines object 2
#define OBJ3 130 //defines object 3
#define OBJ4 178 //defines object 4


const uint8_t SX1509_ADDRESS = 0x3E;  // SX1509 I2C address (00)

//all the wonderful functions
void xel(int); //normal power
void xxel(int); //turning power
void nel(int); //turns motor off
void getval(void); //reads line sensor values
int average(int colour[]); //averages the colour sensor readings
int getDistance(void); //gets the reading from the ultrasonic sensor
int colourDeterction(void); //code to determine colour and set LED
void initialize_sorting(void); //determines which blocks are what material
void detect(int A[], int B[], int C[]); //helper function for initialize_sorting
int detect1(void); //used to read the colour sensor number 2
int detect2(void); //used to read the inductive sensor
int detect3(void); //used to read the hall effect sensor
void detect4(int A[]); //used to assign if none of the sensors detected anything
void disposal(int); //responsible for disposing the blocks
void dispose_obj(int, int, int, int, int, int, int, int); //helper function for disposal
void kick(void); //code for activating the solenoid
void turn_servo(int); //gently turns the servo
void setColour(int); //used to set the colour of the LED
void fakeDispose(void); //shows colour of blocks
void alingnWheel(void);

Servo servo; //declaring servo -> we attach it to pin 6
Servo spinner; //declaring servo -> we attach it to pin 9
SensorBar mySensorBar(SX1509_ADDRESS);
int drive = 5; //drive motor amplifier pin

//colour sensor pins
int colourOut2 = 7;
int colourOut1 = 8;

//shift register pins and setup stuff

//ultrasonic sensor pins
const int trigPin = 3;
const int echoPin = 2;

//inductive sensor pin
int inductivePin = A0;

//hall effect sensor pin
int HEpin = A1;

//variables for calculating distance from unltrasonic sensor
long duration;
int distance;

int solenoid = 13;

//colour sensor pins
int S0 = 12;
int S2 = A2;
int S3 = A3;

//RBG LED pins
int LEDGREEN = 10;
int LEDBLUE = 11;

//shared pins
int S1_LEDRED = 4;

int frequency = 0; //colour sensing frequency

//colour values to determine colour
int red[CSR];
int blue[CSR];
int green[CSR];
int colour;

//general variables for reading the inductive and hall effect sensors
int sensorVal;
int readings;
int sum;
int obj;

//variable to read the sensor values to
int sen0 = 0;
int sen1 = 0;
int sen2 = 0;
int sen3 = 0;
int sen4 = 0;
int sen5 = 0;
int sen6 = 0;
int sen7 = 0;
int det1 = 0;
int det2 = 0;
int det3 = 0;
int i = 0;

int count = 0; // counter to know when to shut robot off
int objFlag = 0; // used for when the robot detects an object
int binInfront = 0; //used for detection of same bin


//arrays for the blocks
int block1[5];
int block2[5];
int block3[5];
int block4[5];

bool sen[8]; //array to hold values that come out of the sensor bar


void setup() {
  Serial.begin(115200); //enable serial monitor
  //command to run the sensor bar all the time
  mySensorBar.clearBarStrobe();

  //detect dark as high
  mySensorBar.clearInvertBits();

  //intializes the sensor bar
  uint8_t returnStatus = mySensorBar.begin();
  if (returnStatus)
  {
    Serial.println("sx1509 IC communication OK");
  }
  else
  {
    Serial.println("sx1509 IC communication FAILED!");
  }
  Serial.println();


  //setting the pins for the colour sensor
  pinMode(colourOut1, INPUT);
  pinMode(colourOut2, INPUT);

  //setting the pins for the colour sensor
  pinMode(S0, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  //setting the pins for the LED
  pinMode(LEDGREEN, OUTPUT);
  pinMode(LEDBLUE, OUTPUT);

  //setting the shared pin
  pinMode(S1_LEDRED, OUTPUT);

  //setting pins for ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //setting hall effect sensor pin
  pinMode(HEpin, INPUT);

  //setting inductive sensor pin
  //pinMode(inductivePin, INPUT_PULLUP);

  servo.attach(9); //attach the servo to pin 6
  spinner.attach(6); //attach the small spinner servo
  pinMode(drive, OUTPUT); //declare the voltage source for motor as an output

  pinMode(solenoid, OUTPUT);


  // Setting frequency-scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1_LEDRED,LOW);

  //determines which blocks are which
  
  initialize_sorting();
  fakeDispose();
  turn_servo(OBJ2);
  

  
  servo.write(STRAIGHT); //straighten the servo
  delay(500);
  xel(drive); //accelerate the motor
  Serial.print("Accelerating");
}

void loop() {
  //Gets sensor values
  getval();
  sen7 = sen[0];
  sen6 = sen[1];
  sen5 = sen[2];
  sen4 = sen[3];
  sen3 = sen[4];
  sen2 = sen[5];
  sen1 = sen[6];
  sen0 = sen[7];

  //prints readings
  Serial.print(sen0 );
  Serial.print(sen1 );
  Serial.print(sen2 );
  Serial.print(sen3 );
  Serial.print(sen4 );
  Serial.print(sen5 );
  Serial.print(sen6 );
  Serial.print(sen7 );
  Serial.println();



  //code that stops the robot after 1/2 a second
  if (count == 100)
    analogWrite(drive, 0);
  if (sen0 == OFF && sen1 == OFF && sen6 == OFF && sen7 == OFF && sen2 == OFF && sen3 == OFF && sen4 == OFF && sen5 == OFF) {
    delay(10);
    count++;
  }


  //code responsible for the autonomous driving
  if (sen0 == OFF && sen1 == OFF && sen6 == OFF && sen7 == OFF) {

    if (sen4 == READ && sen3 == READ) {
      servo.write(STRAIGHT);
      count = 0;
    }

    else if (sen4 == READ && sen5 != READ) {
      servo.write(STRAIGHT - SMALL_ADJ);
      count = 0;
    }
    else if (sen3 == READ && sen2 != READ) {
      servo.write(STRAIGHT + SMALL_ADJ);
      count = 0;
    }

    else if (sen5 == READ) {
      servo.write(STRAIGHT - LARGE_ADJ);
      count = 0;
    }
    else if (sen2 == READ) {
      servo.write(STRAIGHT + LARGE_ADJ);
      count = 0;
    }
  }

  else if (sen6 == READ || sen7 == READ) {
    delay(WAIT);
    nel(drive);
    Serial.println("Decelerating");
    servo.write(LEFT);
    Serial.println("Turning Left");
    delay(SERVO);
    xxel(drive);
    Serial.println("Accelerating");
    do {
      getval();
      sen4 = sen[3];
      sen5 = sen[2];
      Serial.println(sen4);
    }
    while (sen4 != READ || sen5 != READ);
    nel(drive);
    Serial.println("Decelerating");
    servo.write(STRAIGHT);
    Serial.println("Straightening Out");
    delay(SERVO);
    xel(drive);
    Serial.println("Accelerating");
  }
  
  else if (sen0 == READ || sen1 == READ) {
    delay(WAIT);
    nel(drive);
    Serial.println("Decelerating");
    servo.write(RIGHT);
    Serial.println("Turning Right");
    delay(SERVO);
    xxel(drive);
    Serial.println("SUPER Accelerating");
    do {
      getval();
      sen3 = sen[4];
      sen2 = sen[5];
      Serial.println(sen3);
    }
    while (sen3 != READ || sen2 != READ);
    nel(drive);
    Serial.println("Decelerating");
    servo.write(STRAIGHT);
    Serial.println("Straightening Out");
    delay(SERVO);
    xel(drive);
    Serial.println("Accelerating");
  }

  //checks the ultrasonic sensor
  distance = getDistance();

  //if object is detected set the object flag to 1
  if (distance <= OBJ && distance > 1)
    objFlag = 1;

  //confirm that the detected object is really a bin
  if (objFlag == 1) {
    distance = getDistance(); //double checks the distance
    if (distance <= OBJ && distance > 1)
      binInfront = 1;
    else {
      binInfront = 0;
      objFlag = 0;
    }
  }


  //code that enables the colour sensor and detrmines the colour
  if (objFlag == 1 && binInfront == 1) {
    colour = colourDetection();
    alingnWheel();
    xel(drive);
    delay(trayDelay);
    disposal(colour);
  }


}

void xel(int drive) {
  analogWrite(drive, ON);
}

void nel(int drive) {
  analogWrite(drive, OFF);
}

void xxel(int drive) {
  analogWrite(drive, SUPERON);
}


void getval() {
  //Gets sensor valuesvoid reader()
  int raw = mySensorBar.getRaw();
  for (i = 0; i < 8; i++)
  {
    sen[i] = (raw >> i) & 1;
  }
}


int avg = 0;

int average(int colour[]) {
  avg = 0;
  for (i = 0; i < CSR; i++) {
    avg = avg + colour[i];
  }
  avg = avg / CSR;
  return avg;
}


int getDistance(void) {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // Calculating the distance
  distance = duration * 0.034 / 2;

  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);

  //returns distance
  return distance;
}

int colourDetection(void) {
  nel(drive);

  //reading the photodiodes CSR times
  for (i = 0; i < CSR; i++) {
    // Setting red filtered photodiodes to be read
    digitalWrite(S2,LOW);
    digitalWrite(S3,LOW);
    // Reading the output frequency
    frequency = pulseIn(colourOut1, LOW);
    red[i] = frequency;
    delay(100);

    // Setting Green filtered photodiodes to be read
    digitalWrite(S2,HIGH);
    digitalWrite(S3,HIGH);
    // Reading the output frequency
    frequency = pulseIn(colourOut1, LOW);
    green[i] = frequency;
    delay(100);

    // Setting Blue filtered photodiodes to be read
    digitalWrite(S2,LOW);
    digitalWrite(S3,HIGH);
    // Reading the output frequency
    frequency = pulseIn(colourOut1, LOW);
    blue[i] = frequency;
    delay(100);
  }

  //find the average read value
  red[0] = average(red);
  blue[0] = average(blue);
  green[0] = average(green);

  if ( red[0] <= WHITE && blue[0] <= WHITE && green[0] <= WHITE) {
    Serial.println("White");
    setColour(white);
    colour = white;
  }

  else if (red[0] <= blue[0] - 10 && red[0] <= green[0] - 10 ) {
    Serial.println("Red");
    setColour(RED);
    colour = RED;
  }

  else if (blue[0] <= red[0] - 10 && blue[0] <= green[0] - 10 ) {
    Serial.println("Blue");
    setColour(BLUE);
    colour = BLUE;
  }

  else if (green[0] <= blue[0] - 10 && green[0] <= red[0] - 10 ) {
    Serial.println("Green");
    setColour(GREEN);
    colour = GREEN;
  }

  Serial.println();
  //remove the object flag
  objFlag = 0;
  binInfront = 0;
  return colour;
}

void initialize_sorting(void) {

  //detects for magnetic blocks, metal blocks, and clear blocks
  turn_servo(OBJ2);

 //blink red LED for around 10s to place cubes
  /* for(i=1800; i>0; i/2){
    setColour(0);
    delay(i);
    setColour(RED);
    delay(80);  
  }*/
  //setColour(RED);
  //delay(500);
  setColour(RED);
  delay(1000);
  setColour(YELLOW);
  delay(1000);
  setColour(GREEN);
  delay(1000);

//begin block checking
  detect(block1, block3, block4);
  Serial.println("first reading:");
  Serial.println(); 
  
  turn_servo(OBJ1);
  detect(block4, block2, block3);
  Serial.println("second reading:");
  Serial.println(); 

  turn_servo(OBJ4);
  detect(block3, block1, block2);
  Serial.println("third reading:");
  Serial.println(); 

  turn_servo(OBJ3);
  detect(block2, block4, block1);
  Serial.println("last reading:");
  Serial.println(); 

  turn_servo(OBJ2);


  //checks if blocks are the solid plastic material
  detect4(block1);
  detect4(block2);
  detect4(block3);
  detect4(block4);

  //blocks are all loaded
  block1[4] = 1;
  block2[4] = 1;
  block3[4] = 1;
  block4[4] = 1;
}

void detect(int A[], int B[], int C[]) {
  delay(200);
  det1 = detect1();
  det2 = detect2();
  det3 = detect3();

  if (det1 == READ)
    A[0] = 1;
  else
    A[0] = 0;

  if (det2 == READ)
    B[1] = 1;
  else
    B[1] = 0;

  if (det3 == READ)
    C[2] = 1;
  else
    C[2] = 0;
}


//code for block colour sensor
int detect1(void) {

  //reading the photodiodes CSR times
  for (i = 0; i < CSR; i++) {
    // Setting red filtered photodiodes to be read
    digitalWrite(S2, LOW);
    digitalWrite(S3, LOW);  
    // Reading the output frequency
    frequency = pulseIn(colourOut2, LOW);
    red[i] = frequency;
    delay(100);

    // Setting Green filtered photodiodes to be read
    digitalWrite(S2, HIGH);
    digitalWrite(S3, HIGH);
    // Reading the output frequency
    frequency = pulseIn(colourOut2, LOW);
    green[i] = frequency;
    delay(100);

    // Setting Blue filtered photodiodes to be read
    digitalWrite(S2, LOW);
    digitalWrite(S3, HIGH);
    // Reading the output frequency
    frequency = pulseIn(colourOut2, LOW);
    blue[i] = frequency;
    delay(100);
  }

  //find the average read value
  red[0] = average(red);
  blue[0] = average(blue);
  green[0] = average(green);


  if(red[0] <400 && blue[0]<400 && green[0]<400){ //the block is white
    det1 =1;
  } 
  else{
    det1=0;   
  }
  


  return det1;
}


//code for the inductive sensor
int detect2(void) {

  sum = 0;

  for (readings = 10; readings >= 0; readings --) {
    sensorVal = analogRead(inductivePin);
    delay(15);
    sum = sum + sensorVal;
    Serial.println(sensorVal);
  }
  avg = sum / 10;
  Serial.print("average is =");
  Serial.println(avg);

  if (avg >= IDTHRESHOLD) {
    Serial.println(" => NO Metal Detected");
    det2 = 0;
  }
  else {
    Serial.println(" => Metal Detected");
    det2 = 1;
  }

  return det2;
}

//code for reading the magnetic sensor
int detect3(void) {


  sum = 0;

  for (readings = 10; readings >= 0; readings --) {
    sensorVal = analogRead(HEpin);
    delay(15);
    sum = sum + sensorVal;
    Serial.println(sensorVal);
  }
  avg = sum / 10;
  Serial.print("average is =");
  Serial.println(avg);

  if (avg <= (HETHRESHHOLD + 50) && avg >= (HETHRESHHOLD - 50) ) {
    Serial.println(" => NO Magnet Detected");
    det3 = 0;
  }
  else {
    Serial.println(" => Magnet Detected");
    det3 = 1;
  }

  return det3;

}

void detect4(int A[]) {


  if (A[2] == 1) { //if its magnetic it cant be anything else
    A[0] = 0;
    A[1] = 0;
    A[3] = 0;
  }
  
 else if (A[1] == 1) { //if its metal it cant be anything else
    A[0] = 0;
    A[2] = 0;
    A[3] = 0;
  }
  else if (A[0] == 1) { //if its white but not the first two it cant be anything else
    A[1] = 0;
    A[2] = 0;
    A[3] = 0;
  }
  /*else
  { // extra check to see if the block is translucent
    //reading the photodiodes CSR times
       for (i = 0; i < CSR; i++) {
        // Setting red filtered photodiodes to be read
        digitalWrite(S2, LOW);
        digitalWrite(S3, LOW);  
        // Reading the output frequency
        frequency = pulseIn(colourOut2, LOW);
        red[i] = frequency;
        delay(100);
    
        // Setting Green filtered photodiodes to be read
        digitalWrite(S2, HIGH);
        digitalWrite(S3, HIGH);
        // Reading the output frequency
        frequency = pulseIn(colourOut2, LOW);
        green[i] = frequency;
        delay(100);
    
        // Setting Blue filtered photodiodes to be read
        digitalWrite(S2, LOW);
        digitalWrite(S3, HIGH);
        // Reading the output frequency
        frequency = pulseIn(colourOut2, LOW);
        blue[i] = frequency;
        delay(100);
      }
    
        //find the average read value
        red[0] = average(red);
        blue[0] = average(blue);
        green[0] = average(green);
    
      //Final check to confirm that the block is translucent
        if((red[0] <600 && red[0]>400) && (blue[0] <600 && blue[0]>400) && (green[0] <600 && green[0]>400)){ //the block between 400 and 600 => translucent
        A[3] = 1; //only option left it its glass
        } */
       else{
        A[3] = 1; //only option left it its glass
       }
    
   

}

void disposal(int colour) {
  if (colour == RED) {
    dispose_obj(block1[0], block1[4], block2[0], block2[4], block3[0], block3[4], block4[0], block4[4]); //dispose plastic
  }
  else if (colour == GREEN) {
    dispose_obj(block1[1], block1[4], block2[1], block2[4], block3[1], block3[4], block4[1], block4[4]); //dispose metal
  }

  else if (colour == BLUE) {
    dispose_obj(block1[3], block1[4], block2[3], block2[4], block3[3], block3[4], block4[3], block4[4]); //dispose glass
  }

  else if (colour == white) {
    dispose_obj(block1[2], block1[4], block2[2], block2[4], block3[2], block3[4], block4[2], block4[4]); //disposed mixed material (magnet)
  }

}



void dispose_obj(int obj1, int load1, int obj2, int load2, int obj3, int load3, int obj4, int load4) {
  nel(drive);

  if (obj1 == 1 && load1 == 1) {   
    turn_servo(OBJ1);
    delay(NEXT);
    kick();
    block1[4] = 0;
    delay(NEXT); 
    turn_servo(OBJ2);
    delay(NEXT);
  }
  
  else if (obj2 == 1 && load2 == 1) {
    turn_servo(OBJ2);
    delay(NEXT);
    kick();
    block2[4] = 0;
    delay(NEXT);
  }
  else if (obj3 == 1 && load3 == 1) {
    turn_servo(OBJ3);
    delay(NEXT);
    kick();
    block3[4] = 0;
    delay(NEXT);
    turn_servo(OBJ2);
    delay(NEXT);
  }
  else if (obj4 == 1 && load4 == 1) {
    turn_servo(OBJ4);
    delay(NEXT);
    kick();
    block4[4] = 0;
    delay(NEXT);
    turn_servo(OBJ2);
    delay(NEXT);
  }

  xel(drive);
}


void turn_servo(int next) {

int current = spinner.read();

 if (next >= current){
    
    for (i = current; i <= next; i++) {
      spinner.write(i);
      delay(20);
    }
  
  }
  else {
    for (i = current; i >= next; i--) {
      spinner.write(i);
      delay(20);
    }
  }

}


void kick(void) {
  digitalWrite(solenoid, HIGH);
  delay(1000);
  digitalWrite(solenoid, LOW);
}


int option;

void setColour(int option)
{
  if (option == RED)
  {
    digitalWrite(S1_LEDRED, HIGH);
    digitalWrite(LEDGREEN, LOW);
    digitalWrite(LEDBLUE, LOW);
  }

  else if (option == BLUE)
  {
    digitalWrite(S1_LEDRED, LOW);
    digitalWrite(LEDGREEN, LOW);
    digitalWrite(LEDBLUE, HIGH);
  }

  else if (option == GREEN)
  {
    digitalWrite(S1_LEDRED, LOW);
    digitalWrite(LEDGREEN, HIGH);
    digitalWrite(LEDBLUE, LOW);
  }

  else if (option == white)
  {
    digitalWrite(S1_LEDRED, HIGH);
    digitalWrite(LEDGREEN, HIGH);
    digitalWrite(LEDBLUE, HIGH);
  }
  else if (option == YELLOW)
  {
    digitalWrite(S1_LEDRED, HIGH);
    digitalWrite(LEDGREEN, HIGH);
    digitalWrite(LEDBLUE, LOW);
  }
  delay(2000);


  digitalWrite(S1_LEDRED, LOW);
  digitalWrite(LEDGREEN, LOW);
  digitalWrite(LEDBLUE, LOW);
}


void fakeDispose(void){

 int i;

 //block1
    delay(500);
    turn_servo(OBJ1);
    delay(100);
       if(block1[0]) 
        setColour(RED);
      else if(block1[1])
        setColour(GREEN);
      else if(block1[2])
        setColour(white);
      else if(block1[3])
        setColour(BLUE);


//block2
    delay(500);
    turn_servo(OBJ2);
    delay(100);
       if(block2[0]) 
        setColour(RED);
      else if(block2[1])
        setColour(GREEN);
      else if(block2[2])
        setColour(white);
      else if(block2[3])
        setColour(BLUE);
        
//block3
    delay(500);
    turn_servo(OBJ3);
    delay(100);
       if(block3[0]) 
        setColour(RED);
      else if(block3[1])
        setColour(GREEN);
      else if(block3[2])
        setColour(white);
      else if(block3[3])
        setColour(BLUE);
        
//block4
    delay(500);
    turn_servo(OBJ4);
    delay(100);
       if(block4[0]) 
        setColour(RED);
      else if(block4[1])
        setColour(GREEN);
      else if(block4[2])
        setColour(white);
      else if(block4[3])
        setColour(BLUE);   
      
}

void alingnWheel(){

getval();
  sen7 = sen[0];
  sen6 = sen[1];
  sen5 = sen[2];
  sen4 = sen[3];
  sen3 = sen[4];
  sen2 = sen[5];
  sen1 = sen[6];
  sen0 = sen[7];


  //code responsible for the autonomous driving
  if (sen0 == OFF && sen1 == OFF && sen6 == OFF && sen7 == OFF) {

    if (sen4 == READ && sen3 == READ) {
      servo.write(STRAIGHT);
      count = 0;
    }

    else if (sen4 == READ && sen5 != READ) {
      servo.write(STRAIGHT - SMALL_ADJ);
      count = 0;
    }
    else if (sen3 == READ && sen2 != READ) {
      servo.write(STRAIGHT + SMALL_ADJ);
      count = 0;
    }

    else if (sen5 == READ) {
      servo.write(STRAIGHT - LARGE_ADJ);
      count = 0;
    }
    else if (sen2 == READ) {
      servo.write(STRAIGHT + LARGE_ADJ);
      count = 0;
    }
  }

  
}