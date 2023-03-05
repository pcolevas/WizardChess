#include <Arduino.h>

#include <AccelStepper.h>
#include <MultiStepper.h>

/*/////////////////////////////////////////////////////////////////////////////
                                      VARIABLES
/////////////////////////////////////////////////////////////////////////////*/

// step size 
const int step_x = 400 ; 
const int step_y = 400 ; 

//State Variables
bool start = 1; 
bool splice = 0;
bool conv = 0; 
bool mov = 0; 

// define pins
const int stepPinA = 12;
const int dirPinA = 14;
const int stepPinB = 27;
const int dirPinB = 26;
const int limitSwitch_x = 32;
const int limitSwitch_y = 33;
const int Emag = 34 ; 

//message processing variables
const unsigned int MAX_MESSAGE_LENGTH = 12;
static char message[MAX_MESSAGE_LENGTH];
char initLet;
char initNum;
char finLet;
char finNum;
float initX;
float initY;
float finX;
float finY; 


// instantiate stepper motor objects
AccelStepper A(AccelStepper::DRIVER, stepPinA, dirPinA);
AccelStepper B(AccelStepper::DRIVER, stepPinB, dirPinB);

/*///////////////////////////////////////////////////////////////////////
                                    FUNCTIONS 
///////////////////////////////////////////////////////////////////////*/

//Stops both motors immediately
void stopBoth(){
  A.stop();
  B.stop();
}

//USED ONLY FOR INITIALIZATION//
void negX(){
  A.setSpeed(-300);
  B.setSpeed(-300);
  A.runSpeed();
  B.runSpeed();
}

//USED ONLY FOR INITIALIZATION//
void negY(){
  
  A.setSpeed(-200);
  B.setSpeed(200);
  A.runSpeed();
  B.runSpeed();
}




//Sets the current position as zero for both A and B motors
void currentPositionZero(){
   A.setCurrentPosition(0.0);
   B.setCurrentPosition(0.0);
}

// moveDeltaX and moveDeltaY ONLY complete lateral and vertical movements along respective axes 
void moveDeltaX(float delta){

   A.moveTo(delta);
   B.moveTo(delta);
   A.setSpeed(300);
   B.setSpeed(300);

   while(A.distanceToGo()!=0 && B.distanceToGo()!=0){
      A.runSpeedToPosition();
      B.runSpeedToPosition();
      }
}

void moveDeltaY(float delta){

   A.moveTo(delta);
   B.moveTo(-1.0*delta);
   A.setSpeed(300);
   B.setSpeed(300);

   while(A.distanceToGo()!=0 && B.distanceToGo()!=0){
      A.runSpeedToPosition();
      B.runSpeedToPosition();
      }
}

// Used to Home the cart at a bottom left hand corner of the board 

void initialize(){
  //uses the state variable 'start' to turn thes function on or off
  if(start){
  stopBoth();
   while(digitalRead(limitSwitch_y)){
      negY();
      }
   stopBoth();
   delay(100);
   while(digitalRead(limitSwitch_x)){
      negX();}
      stopBoth();
   currentPositionZero();
   start = 0;}  
}


void moveInitCo(float x, float y){

 if(mov){

  float targetx = x ;
  float targety = y ; 

  moveDeltaX(x); 
  currentPositionZero();
  moveDeltaY(y); 
  //arives at point (x,y) 



  
  mov = 0;
 }
}


//this is a first try at going from initialized to (x1,y1) and then to (x2, y2) 
void moveCo(float x1, float y1, float x2, float y2){

 if(mov){
 
    float deltaX = x2-x1;
    float deltaY = y2-y1;
  
    moveDeltaX(x1);
    currentPositionZero();
    moveDeltaY(y1);
    // arives at (x1,y1)
       
    delay(2000); 
    
    currentPositionZero();
    moveDeltaX(deltaX);
    currentPositionZero();
    moveDeltaY(deltaY);
    //arives at (x2, y2)
     
    delay(1000); 

    start = 1; 
  
    //go back the home point
    initialize(); 
    
    
    mov = 0;
 }
}






void convert(){
  if(conv){

      // Use these chars to convert message to corrdinates
      char A = 'A';
      char B = 'B';
      char C = 'C';
      char D = 'D';
      char E = 'E';
      char F = 'F';
      char G = 'G';
      char H = 'H';

      char one = '1';
      char two = '2';
      char three = '3';
      char four = '4';
      char five = '5';
      char six = '6';
      char seven = '7';
      char eight = '8';
     
      

      if(initLet == A){
        // initX = 700. ;
        initX = 300 + 1*step_x ;  
      }
      else if(initLet==B){
        // initX = 1100. ;
        initX = 300 + 2*step_x ;
      }
      else if(initLet==C){
        // initX = 600. ;
        initX = 300 + 3*step_x ;
      }
      else if(initLet==D){
        // initX = 800. ;
        initX = 300 + 4*step_x ;
      }
      else if(initLet==E){
        // initX = 1000. ;
        initX = 300 + 5*step_x ;
      }
      else if(initLet==F){
        // initX = 1200. ;
        initX = 300 + 6*step_x ; 
      }
      else if(initLet==G){
        // initX = 1400. ;
        initX = 300 + 7*step_x ; 
      }
      else if(initLet==H){
        // initX = 1600. ;
        initX = 300 + 8*step_x ; 
      }


      if(initNum==one){
        initY = 0. ; 
      }
      else if(initNum==two){
        // initY = 400. ;
        initY = 1*step_y ; 
      }
      else if(initNum==three){
        //initY = 600. ;
        initY = 2*step_y ;
      }
      else if(initNum==four){
        //initY = 800. ;
        initY = 3*step_y ;
      }
      else if(initNum==five){
        //initY = 1000. ;
        initY = 4*step_y ;
      }
      else if(initNum==six){
        //initY = 1200. ;
        initY = 5*step_y ;
      }
      else if(initNum==seven){
        //initY = 1400. ;
        initY = 6*step_y ;
      }
      else if(initNum==eight){
        //initY = 1600. ;
        initY = 7*step_y ;
      }

      if(finLet==A){
        // finX = 700. ; 
        finX = 300 + 1*step_x ; 
      }
      else if(finLet==B){
        //finX = 1052. ;
        finX = 300 + 2*step_x ; 
      }
      else if(finLet==C){
        //finX = 600. ;
        finX = 300 + 3*step_x ; 
      }
      else if(finLet==D){
        //finX = 800. ;
        finX = 300 + 4*step_x ; 
      }
      else if(finLet==E){
        //finX = 1000. ;
        finX = 300 + 5*step_x ; 
      }
      else if(finLet==F){
        //finX = 1200. ;
        finX = 300 + 6*step_x ; 
      }
      else if(finLet==G){
        //finX = 1400. ;
        finX = 300 + 7*step_x ; 
      }
      else if(finLet==H){
        //finX = 1600. ;
        finX = 300 + 8*step_x ; 
      }


      if(finNum==one){
        finY = 0. ; 
      }
      else if(finNum==two){
        // finY = 400. ;
        finY = 1*step_y ; 
      }
      else if(finNum==three){
        //finY = 600. ;
        finY = 2*step_y ;
      }
      else if(finNum==four){
        //finY = 800. ;
        finY = 3*step_y ;
      }
      else if(finNum==five){
        //finY = 1000. ;
        finY = 4*step_y ;
      }
      else if(finNum==six){
        //finY = 1200. ;
        finY = 5*step_y ;
      }
      else if(finNum==seven){
        //finY = 1400. ;
        finY = 6*step_y ;
      }
      else if(finNum==eight){
        //finY = 1600. ;
        finY = 7*step_y ;
      }

    Serial.print(initX);
    Serial.print('\n');
    Serial.print(initY);
    Serial.print('\n');
    Serial.print(finX);
    Serial.print('\n');
    Serial.print(finY);
    Serial.print('\n');
    
    conv = 0; 
    mov = 1; 
    
  }
}


void spliceMessage(char mes[]){

    
    if(splice){
        static unsigned int message_pos = 0;
        initLet = mes[0];
        initNum = mes[1];
        finLet = mes[6] ;
        finNum = mes[7]; 

        splice = 0 ;
        conv = 1;
    }
}

void getMessage(char mes[]){

      while (Serial.available() > 0){
         
         static unsigned int message_pos = 0;
         char inByte = Serial.read();
      
         if ( inByte != '\n' && (message_pos < MAX_MESSAGE_LENGTH - 1) ){
           mes[message_pos] = inByte;
           message_pos++;}
         else{
           mes[message_pos] = '\0';
           message_pos = 0;
           splice = 1;}
       } 
}


/*/////////////////////////////////////////////////////////////////////////////
                                      SETUP
/////////////////////////////////////////////////////////////////////////////*/


void setup() {
  Serial.begin(115200);

  //Stepper Setup
  A.setMaxSpeed(5000.0);
  B.setMaxSpeed(5000.0);
  A.setCurrentPosition(0.);
  B.setCurrentPosition(0.); 
  A.setAcceleration(5000.);
  B.setAcceleration(5000.);
 

  //Limit Switches
  pinMode(limitSwitch_x, INPUT_PULLUP);
  pinMode(limitSwitch_y, INPUT_PULLUP);

  //Delay before beginnging
  delay(3000);

  //Initialize Position  
  initialize();

}
  

/*/////////////////////////////////////////////////////////////////////////////
                                      LOOP 
/////////////////////////////////////////////////////////////////////////////*/

void loop() { 

  //Get Coordinates//
  getMessage(message);
  spliceMessage(message) ;
  convert();
  //moveInitCo(initX,initY); 
  moveCo(initX, initY, finX, finY) ; 

}


  
