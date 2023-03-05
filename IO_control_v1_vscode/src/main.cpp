#include <Arduino.h>

#include <AccelStepper.h>
#include <MultiStepper.h>

/*/////////////////////////////////////////////////////////////////////////////
                                      VARIABLES
/////////////////////////////////////////////////////////////////////////////*/
// coordinate array trial 
int coordinate_Array[] = {(700, 0), (700, 400), (700, 800), (700, 1200), (700, 1600), (700, 2000), (700, 2400), (700, 2800)

                          }; 

// step size 
const int step_x = 375 ; 
const int step_y = 375 ; 

//State Variables
bool start = 1; 
bool splice = 0;
bool conv = 0; 
bool mov = 0; 
bool LEDstateG = LOW ;
bool LEDstateR = LOW; 
bool LEDstateB = LOW;  

// define pins
const int stepPinA = 12;
const int dirPinA = 14;
const int stepPinB = 27;
const int dirPinB = 26;
const int limitSwitch_x = 32;
const int limitSwitch_y = 33;
const int Emag = 34 ; 
const int LED_Green = 13; 
const int LED_red = 25 ; 
// const int LED_Blue = 34; 

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
// Turn on LED 
void LED_G_on() { 
  
    LEDstateG = HIGH ; 
    digitalWrite(LED_Green, LEDstateG); 
  
}

void LED_G_off(){
  
    LEDstateG = LOW; 
    digitalWrite(LED_Green, LEDstateG); 
  
}

void LED_R_on() { 
  
    LEDstateR = HIGH ; 
    digitalWrite(LED_red, LEDstateR); 
  
}

void LED_R_off(){
  
    LEDstateR = LOW; 
    digitalWrite(LED_red, LEDstateR); 
  
}
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
    LED_G_on();  
    delay(2000); 
     
    currentPositionZero();
    moveDeltaX(deltaX);
    currentPositionZero();
    moveDeltaY(deltaY);
    //arives at (x2, y2)
    LED_G_off(); 
    delay(1000); 
    LED_R_on(); 
    start = 1; 
  
    //go back the home point
    initialize(); 
    LED_R_off(); 
    
    
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
         initX = 700. ;
        
      }
      else if(initLet==B){
       initX = 1075. ;
       
      }
      else if(initLet==C){
        initX = 1450. ;
        
      }
      else if(initLet==D){
        initX = 1785. ;
        
      }
      else if(initLet==E){
        initX = 2300. ;
        
      }
      else if(initLet==F){
        initX = 2700. ;
         
      }
      else if(initLet==G){
       initX = 3100. ;
         
      }
      else if(initLet==H){
       initX = 3500. ;
        
      }


      if(initNum==one){
        initY = 0. ; 
      }
      else if(initNum==two){
       initY = 400. ;
         
      }
      else if(initNum==three){
        initY = 800. ;
        
      }
      else if(initNum==four){
        initY = 1200. ;
        
      }
      else if(initNum==five){
        initY = 1600. ;
        
      }
      else if(initNum==six){
        initY = 2000. ;
        
      }
      else if(initNum==seven){
        initY = 2400. ;
        
      }
      else if(initNum==eight){
        initY = 2800. ;
        
      }

      if(finLet==A){
        finX = 700. ; 
        
      }
      else if(finLet==B){
        finX = 1075. ;
        
      }
      else if(finLet==C){
        finX = 1450. ;
        
      }
      else if(finLet==D){
        finX = 1785. ;
         
      }
      else if(finLet==E){
        finX = 2300. ;
        
      }
      else if(finLet==F){
        finX = 2700. ;
         
      }
      else if(finLet==G){
        finX = 3100. ;
         
      }
      else if(finLet==H){
       finX = 3500. ;
         
      }


      if(finNum==one){
        finY = 0. ; 
      }
      else if(finNum==two){
        finY = 400. ;
        
      }
      else if(finNum==three){
        finY = 800. ;
        
      }
      else if(finNum==four){
        finY = 1200. ;
        
      }
      else if(finNum==five){
        finY = 1600. ;
        
      }
      else if(finNum==six){
        finY = 2000. ;
        
      }
      else if(finNum==seven){
        finY = 2400. ;
        
      }
      else if(finNum==eight){
        finY = 2800. ;
        
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

  // LED pins 
 // pinMode(LED_Blue, OUTPUT); 
 pinMode(LED_Green, OUTPUT); 
 pinMode(LED_red, OUTPUT); 

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


  
