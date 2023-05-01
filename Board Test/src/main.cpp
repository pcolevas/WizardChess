#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include "driver/i2s.h" // Library of I2S routines, comes with ESP32 standard install
#include "Checkmate.h"
#include "ExactlyLike.h"
#include "Kill1.h"
#include "Kill2.h"
#include "Kill3.h"
#include "Remix.h"
#include "SorryAboutThat.h"
#include "SwordSound1.h"
#include "SwordSound2.h"
#include "ThemeSong.h"
#include "TotallyBarbaric.h"
#include "VeryClearly.h"
#include "YoureAWizardData.h"
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <WiFi.h>
#include <WebServer.h>

/*/////////////////////////////////////////////////////////////////////////////
                                      VARIABLES
/////////////////////////////////////////////////////////////////////////////*/
const float BOARD_START_X = 590.0;
const float BOARD_START_Y = 45.0;
const float SQUARE_SIZE = 360.0;
// float m = 360; (unused variable)
const int SPEED = 700;
const float HOME_X = 1860.0;
const float HOME_Y = 1275.0;
const float CAPT_XL = 50.0;
const float CAPT_XR = 3700.0;

// moveDelta factors
const int X_FACTOR = 1;
const int Y_FACTOR = -1;

// step size
const int STEP_X = 375;
const int STEP_Y = 375;

// define pins
const int STEP_PIN_A = 10;
const int DIR_PIN_A = 11;
const int STEP_PIN_B = 12;
const int DIR_PIN_B = 13;
const int LIMIT_SWITCH_X = 2;
const int LIMIT_SWITCH_Y = 1;
const int EMAG = 7;
const int LED_R = 16;
const int LED_G = 17;
const int LED_B = 18;

// message processing variables
int turnCounter = 0;

// WIFI Setup
const char *ssid = "SDNet";
const char *password = "CapstoneProject";
String message = "";
WebServer server(80);

// instantiate stepper motor objects
AccelStepper A(AccelStepper::DRIVER, STEP_PIN_A, DIR_PIN_A);
AccelStepper B(AccelStepper::DRIVER, STEP_PIN_B, DIR_PIN_B);

// Audio
static const i2s_port_t i2s_num = I2S_NUM_0; // i2s port number

struct WavHeader_Struct
{
    //   RIFF Section
    char RIFFSectionID[4]; // Letters "RIFF"
    uint32_t Size;         // Size of entire file less 8
    char RiffFormat[4];    // Letters "WAVE"

    //   Format Section
    char FormatSectionID[3]; // letters "fmt"
    uint32_t FormatSize;     // Size of format section less 8
    uint16_t FormatID;       // 1=uncompressed PCM
    uint16_t NumChannels;    // 1=mono,2=stereo
    uint32_t SampleRate;     // 44100, 16000, 8000 etc.
    uint32_t ByteRate;       // =SampleRate * Channels * (BitsPerSample/8)
    uint16_t BlockAlign;     // =Channels * (BitsPerSample/8), effectivly the size of a single sample for all chans.
    uint16_t BitsPerSample;  // 8,16,24 or 32

    // Data Section
    char DataSectionID[4]; // The letters "data"
    uint32_t DataSize;     // Size of the data that follows
} WavHeader;

// speaker settings
#define USE_I2S_SPEAKER_OUTPUT
#define I2S_SPEAKER_SERIAL_DATA GPIO_NUM_4
#define I2S_SPEAKER_SERIAL_CLOCK GPIO_NUM_5
#define I2S_SPEAKER_LEFT_RIGHT_CLOCK GPIO_NUM_6

// Shutdown line if you have this wired up or -1 if you don't
#define I2S_SPEAKER_SD_PIN -1

static const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 44100, // Note, this will be changed later
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // high interrupt priority
    .dma_buf_count = 8,                       // 8 buffers
    .dma_buf_len = 1024,                      // 1K per buffer, so 8K of buffer space
    .use_apll = 0,
    .tx_desc_auto_clear = true,
    .fixed_mclk = -1};

// These are the physical wiring connections to our I2S decoder board/chip from the esp32, there are other connections
// required for the chips mentioned at the top (but not to the ESP32), please visit the page mentioned at the top for
// further information regarding these other connections.

static const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SPEAKER_SERIAL_CLOCK,
    .ws_io_num = I2S_SPEAKER_LEFT_RIGHT_CLOCK,
    .data_out_num = I2S_SPEAKER_SERIAL_DATA,
    .data_in_num = I2S_PIN_NO_CHANGE};

/*///////////////////////////////////////////////////////////////////////
                                    HEADERS
///////////////////////////////////////////////////////////////////////*/
void handlePost();
void ledOff();
void ledOn(bool red, bool green, bool blue);
void stopBoth();
void currentPositionZero();
void initialize();
void moveDelta(int bFactor, float delta);
void moveDiagonally(float deltX, float deltY);
void move(float startX, float startY, float endX, float endY);
void move(float deltX, float deltY);
void captChessMove(float startX, float startY, float endX, float endY);
void chessMove(float startX, float startY, float endX, float endY, bool isCapture);
void spliceMessage();
void ledMessage();
bool validWAVData(WavHeader_Struct *Wav);
void dumpWAVHeader(WavHeader_Struct *Wav);
void play(const unsigned char *WavFile);
/*///////////////////////////////////////////////////////////////////////
                                    FUNCTIONS
///////////////////////////////////////////////////////////////////////*/

void handlePost()
{
    if (server.method() == HTTP_POST)
    {
        message = server.arg("message");
        Serial.print(message);
        server.send(200);
    }
}

// disable ALL led
void ledOff()
{
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_B, LOW);
}

// turn on LEDs corresponding to arguments
void ledOn(bool red, bool green, bool blue)
{
    if (red)
        digitalWrite(LED_R, HIGH);
    if (green)
        digitalWrite(LED_G, HIGH);
    if (blue)
        digitalWrite(LED_B, HIGH);
}

// Stops both motors immediately
void stopBoth()
{
    A.stop();
    B.stop();
}

// Sets the current position as zero for both A and B motors
void currentPositionZero()
{
    A.setCurrentPosition(0.0);
    B.setCurrentPosition(0.0);
}

void initialize()
{
    stopBoth();
    while (digitalRead(LIMIT_SWITCH_Y))
    {
        A.setSpeed(-SPEED);
        B.setSpeed(SPEED);
        A.runSpeed();
        B.runSpeed();
    }

    stopBoth();
    delay(100);
    while (digitalRead(LIMIT_SWITCH_X))
    {
        A.setSpeed(-SPEED);
        B.setSpeed(-SPEED);
        A.runSpeed();
        B.runSpeed();
    }

    stopBoth();
    currentPositionZero();
}
// moveDelta ONLY complete lateral and vertical movements along respective axes
// moveDelta(X_FACTOR, delta) indicates B.moveTo(delta)
// moveDelta(Y_FACTOR, delta) indicates B.moveTo(-delta)
void moveDelta(int bFactor, float delta)
{
    A.moveTo(delta);
    B.moveTo(bFactor * delta);
    A.setSpeed(SPEED);
    B.setSpeed(SPEED);

    while (A.distanceToGo() != 0 || B.distanceToGo() != 0)
    {
        A.runSpeedToPosition();
        B.runSpeedToPosition();
    }
}

void moveDiagonally(float deltX, float deltY)
{
    float delta = sqrt((sq(deltX)) + (sq(deltY)));
    float diagonalFactor = 1.45;

    if (deltX < 0)
        diagonalFactor *= -1;
    if (((deltX > 0) && (deltY > 0)) || ((deltX < 0) && (deltY < 0)))
    {
        A.moveTo(diagonalFactor * delta);
        A.setSpeed(SPEED);

        while (A.distanceToGo() != 0)
            A.runSpeedToPosition();
    }
    else
    {
        B.moveTo(diagonalFactor * delta);
        B.setSpeed(SPEED);

        while (B.distanceToGo() != 0)
            B.runSpeedToPosition();
    }
}

// overloaded move function using deltas instead of location literals
void move(float startX, float startY, float endX, float endY)
{
    float deltX = endX - startX;
    float deltY = endY - startY;

    currentPositionZero();
    moveDelta(Y_FACTOR, deltY);
    currentPositionZero();
    moveDelta(X_FACTOR, deltX);
    currentPositionZero();
    // arives at point (x,y)
}

void move(float deltX, float deltY)
{
    currentPositionZero();
    moveDelta(Y_FACTOR, deltY);
    currentPositionZero();
    moveDelta(X_FACTOR, deltX);
    currentPositionZero();
    // arives at point (x,y)
}

void captChessMove(float startX, float startY, float endX, float endY)
{
    ledOff();
    ledOn(1, 0, 0);

    const int distanceToJail = 170;

    float captPosY;
    if (endY != BOARD_START_Y + 7 * SQUARE_SIZE)
        captPosY = endY + distanceToJail;
    else
        captPosY = endY - distanceToJail;

    float captureSide;
    if (turnCounter % 2 == 0)
        captureSide = CAPT_XR;
    else
        captureSide = CAPT_XL;

    move(HOME_X, HOME_Y, endX, endY);
    digitalWrite(EMAG, HIGH); // magnetOn
    delay(500);

    move(endX, endY, captureSide, captPosY);
    delay(100);
    digitalWrite(EMAG, LOW); // magnetOff

    // move back to piece being moved
    move(captureSide, captPosY, startX, startY);
}

void chessMove(float startX, float startY, float endX, float endY, bool isCapture)
{
    if (isCapture)
        captChessMove(startX, startY, endX, endY);
    else
        move(HOME_X, HOME_Y, startX, startY);

    float deltX = endX - startX;
    float deltY = endY - startY;
    float xhalfstep = deltX / 2;
    float yhalfstep = deltY / 2;

    ledOn(1, 1, 1);
    digitalWrite(EMAG, HIGH); // magnetOn
    delay(500);

    bool isDiag = 0;
    int xChange = 0;
    int yChange = 0;

    if (abs((deltX)) == abs(deltY)) // diagonal move
    {
        moveDiagonally(deltX, deltY);
        isDiag = 1;
    }
    else if ((!deltX) || (!deltY)) // horizontal or vertical move
    {
        if (!deltX){

            moveDelta(Y_FACTOR, deltY);
            
        }
        if (!deltY){

            moveDelta(X_FACTOR, deltX);
         
        }
    }
    else if (abs(deltX) / SQUARE_SIZE == 1 && abs(deltY) / SQUARE_SIZE == 2) // knight move type 1
    {
        xChange = 50 * (deltX / SQUARE_SIZE);

        moveDelta(X_FACTOR, xhalfstep);
        // calls overloaded move function using deltas instead of location literals
        move(xhalfstep + xChange, deltY);
    }
    else // knight move type 2
    {
        yChange = 50 * (deltY / SQUARE_SIZE);

        // calls overloaded move function using deltas instead of location literals
        move(deltX, yhalfstep);
        moveDelta(Y_FACTOR, yhalfstep + yChange);
    }

    delay(500);
    digitalWrite(EMAG, LOW); // magnetOff
    ledOn(1, 1, 1);

    if ((turnCounter % 4 == 0 && turnCounter != 0) || isDiag)
    {
        initialize();
        move(HOME_X, HOME_Y);
    }
    else
        move(endX + xChange, endY + yChange, HOME_X, HOME_Y);


    ledOn(1, 1, 1);

    if(isCapture){
        int random1 = rand() % 5;
    switch (random1){
        case 0:
            play(Kill1);
            break;
        case 1:
            play(Kill2);
        case 2:
            play(Kill3);
        case 3:
            play(SorryAboutThat);
        case 4:
            play(TotallyBarbaric); 
        default:
            break;
        }
    }
    else{
        int random2 = rand() %4;
    switch (random2){
        case 0:
            play(Yer_A_Wizard_Data);
            break;
        case 1:
            play(SwordSound2);
            break;
        case 2:
            play(SwordSound1);
            break;
        case 3:
            play(SorryAboutThat);
            break;
        default:
            break;
        }
    }

    turnCounter++;
}

void spliceMessage()
{
    static unsigned int message_pos = 0;

    float xArray[8];
    float yArray[8];

    for (int i = 0; i < 8; i++)
    {
        xArray[i] = BOARD_START_X + (i * SQUARE_SIZE);
        yArray[i] = BOARD_START_Y + (i * SQUARE_SIZE);
    }

    // convert to integers
    // message[0] - 'A' will give 0 for A, 1 for B, ...
    int transX1 = message[0] - 'A';
    int transY1 = message[1] - '1';
    int transX2 = message[2] - 'A';
    int transY2 = message[3] - '1';

    // convertCapt: isCapture is 1 iff message[4] is '1', else 0
    bool isCapture = (message[4] == '1');

    // useArray
    float initX = xArray[transX1];
    float initY = yArray[transY1];
    float finX = xArray[transX2];
    float finY = yArray[transY2];

    chessMove(initX, initY, finX, finY, isCapture);
}

void ledMessage()
{
    bool redCode = message[0] == '1';
    bool greenCode = message[1] == '1';
    bool blueCode = message[2] == '1';

    ledOff();
    ledOn(redCode, greenCode, blueCode);

    //if (redCode)
     //   play(Exactly_Like_WizChess_Data);
    //else if (greenCode || blueCode)
      //  play(Yer_A_Wizard_Data);
}

bool validWAVData(WavHeader_Struct *Wav)
{
    if (memcmp(Wav->RIFFSectionID, "RIFF", 4) != 0)
    {
        Serial.print("Invlaid data - Not RIFF format");
        return false;
    }
    if (memcmp(Wav->RiffFormat, "WAVE", 4) != 0)
    {
        Serial.print("Invlaid data - Not Wave file");
        return false;
    }
    if (memcmp(Wav->FormatSectionID, "fmt", 3) != 0)
    {
        Serial.print("Invlaid data - No format section found");
        return false;
    }
    if (memcmp(Wav->DataSectionID, "data", 4) != 0)
    {
        Serial.print("Invlaid data - data section not found");
        return false;
    }
    if (Wav->FormatID != 1)
    {
        Serial.print("Invlaid data - format Id must be 1");
        return false;
    }
    if (Wav->FormatSize != 16)
    {
        Serial.print("Invlaid data - format section size must be 16.");
        return false;
    }
    if ((Wav->NumChannels != 1) & (Wav->NumChannels != 2))
    {
        Serial.print("Invlaid data - only mono or stereo permitted.");
        return false;
    }
    if (Wav->SampleRate > 48000)
    {
        Serial.print("Invlaid data - Sample rate cannot be greater than 48000");
        return false;
    }
    if (Wav->BitsPerSample != 16)
    {
        Serial.print("Invlaid data - Only 16 bits per sample permitted.");
        return false;
    }
    return true;
}

void dumpWAVHeader(WavHeader_Struct *Wav)
{
    if (memcmp(Wav->RIFFSectionID, "RIFF", 4) != 0)
    {
        Serial.print("Not a RIFF format file - ");
        Serial.print(Wav->RIFFSectionID);
        return;
    }
    if (memcmp(Wav->RiffFormat, "WAVE", 4) != 0)
    {
        Serial.print("Not a WAVE file - ");
        Serial.print(Wav->RiffFormat);
        return;
    }
    if (memcmp(Wav->FormatSectionID, "fmt", 3) != 0)
    {
        Serial.print("fmt ID not present - ");
        Serial.print(Wav->FormatSectionID);
        return;
    }
    if (memcmp(Wav->DataSectionID, "data", 4) != 0)
    {
        Serial.print("data ID not present - ");
        Serial.print(Wav->DataSectionID);
        return;
    }

    // All looks good, dump the data
    Serial.print("Total size :");
    Serial.println(Wav->Size);
    Serial.print("Format section size :");
    Serial.println(Wav->FormatSize);
    Serial.print("Wave format :");
    Serial.println(Wav->FormatID);
    Serial.print("Channels :");
    Serial.println(Wav->NumChannels);
    Serial.print("Sample Rate :");
    Serial.println(Wav->SampleRate);
    Serial.print("Byte Rate :");
    Serial.println(Wav->ByteRate);
    Serial.print("Block Align :");
    Serial.println(Wav->BlockAlign);
    Serial.print("Bits Per Sample :");
    Serial.println(Wav->BitsPerSample);
    Serial.print("Data Size :");
    Serial.println(Wav->DataSize);
}

void play(const unsigned char *WavFile)
{
    unsigned const char *TheData;
    uint32_t DataIdx = 0;            // index offset into "TheData" for current  data t send to I2S
    memcpy(&WavHeader, WavFile, 44); // Copy the header part of the wav data into our structure
    dumpWAVHeader(&WavHeader);       // Dump the header data to serial, optional!
    if (validWAVData(&WavHeader))
    {
        i2s_set_sample_rates(i2s_num, WavHeader.SampleRate); // set sample rate
        TheData = WavFile + 44;                              // set to start of data

        while (DataIdx <= WavHeader.DataSize)
        {
            uint8_t Mono[4];           // This holds the data we actually send to the I2S if mono sound
            const unsigned char *Data; // Points to the data we are going to send
            size_t BytesWritten;       // Returned by the I2S write routine, we are not interested in it

            // The WAV Data could be mono or stereo but always 16 bit, that's a data size of 2 byte or 4 bytes
            // Unfortunatly I2S only allows stereo, so to send mono we have to send the mono sample on both left and right
            // channels. It's a bit of a faf really!
            if (WavHeader.NumChannels == 1) // mono
            {
                Mono[0] = *(TheData + DataIdx); // copy the sample to both left and right samples, this is left
                Mono[1] = *(TheData + DataIdx + 1);
                Mono[2] = *(TheData + DataIdx); // Same data to the right channel
                Mono[3] = *(TheData + DataIdx + 1);
                Data = Mono;
            }
            else // stereo
                Data = TheData + DataIdx;

            i2s_write(i2s_num, Data, 4, &BytesWritten, portMAX_DELAY);
            DataIdx += WavHeader.BlockAlign; // increase the data index to next next sample
        }
    }
}

/*/////////////////////////////////////////////////////////////////////////////
                                      SETUP
/////////////////////////////////////////////////////////////////////////////*/

void setup()
{
    Serial.begin(115200);

    i2s_driver_install(i2s_num, &i2s_config, 0, NULL); // ESP32 will allocated resources to run I2S
    i2s_set_pin(i2s_num, &pin_config);

    IPAddress ip(192, 168, 10, 69);       // Set the desired IP address here
    IPAddress gateway(192, 168, 10, 144); // Set the gateway address here
    IPAddress subnet(255, 255, 255, 0);   // Set the subnet mask here

    // WIFI Setup
    //  Connect to Wi-Fi network with SSID and password
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }

    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    WiFi.config(ip, gateway, subnet);
    server.on("/post", handlePost);
    server.begin();

    // Stepper Setup
    A.setMaxSpeed(5000.0);
    B.setMaxSpeed(5000.0);
    A.setCurrentPosition(0.);
    B.setCurrentPosition(0.);
    A.setAcceleration(1500.);
    B.setAcceleration(1500.);

    // LED pins
    pinMode(LED_G, OUTPUT);
    pinMode(LED_B, OUTPUT);
    pinMode(LED_R, OUTPUT);

    // Magnet Pin
    pinMode(EMAG, OUTPUT);

    // Limit Switches
    pinMode(LIMIT_SWITCH_X, INPUT_PULLUP);
    pinMode(LIMIT_SWITCH_Y, INPUT_PULLUP);

    // Initialize Position
    ledOn(1, 1, 1);
    initialize();
    move(HOME_X, HOME_Y);
    play(ExactlyLike);
    play(Remix); 
    play(VeryClearly);
}

/*/////////////////////////////////////////////////////////////////////////////
                                      LOOP
/////////////////////////////////////////////////////////////////////////////*/

void loop()
{
    server.handleClient();

    if (message.length() == 6 || message.length() == 5)
        spliceMessage();
    else if (message.length() == 3)
        ledMessage();

    message = "";
}