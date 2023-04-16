//------------------------------------------------------------------------------------------------------------------------
// Play a WAV file from memory, stored in the File called WavData.h within this directory
// This version supports mono or stereo sound but must be 16 bit sound, any sample speed
// Will display the wav file stats before playing
//
// Boring copyright/usage information:
//    (c) XTronical, www.xtronical.com
//    Use as you wish for personal or monatary gain, or to rule the world (if that sort of thing spins your bottle)
//    However you use it, no warrenty is provided etc. etc. It is not listed as fit for any purpose you perceive
//    It may damage your house, steal your lover, drink your beers and more.
//    
// For more information and wiring for the specific chips mentioned please visit:
//    http://www.xtronical.com/I2SAudio
//
//------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------------------
// Includes
    #include <Arduino.h>
    #include <stdio.h>
    #include <stdlib.h>
    #include "driver/i2s.h"                       // Library of I2S routines, comes with ESP32 standard install
    #include "YoureAWizardData.h"
    #include "ThemeSongData.h"
    #include "ExactlyLikeWizardChessData.h"                          // The Wav file stored in memory, should be in folder with this file
//------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------------------
//  Global Variables/objects    
    static const i2s_port_t i2s_num = I2S_NUM_0;  // i2s port number

    struct WavHeader_Struct
    {
      //   RIFF Section    
      char RIFFSectionID[4];      // Letters "RIFF"
      uint32_t Size;              // Size of entire file less 8
      char RiffFormat[4];         // Letters "WAVE"
      
      //   Format Section    
      char FormatSectionID[4];    // letters "fmt"
      uint32_t FormatSize;        // Size of format section less 8
      uint16_t FormatID;          // 1=uncompressed PCM
      uint16_t NumChannels;       // 1=mono,2=stereo
      uint32_t SampleRate;        // 44100, 16000, 8000 etc.
      uint32_t ByteRate;          // =SampleRate * Channels * (BitsPerSample/8)
      uint16_t BlockAlign;        // =Channels * (BitsPerSample/8), effectivly the size of a single sample for all chans.
      uint16_t BitsPerSample;     // 8,16,24 or 32
    
      // Data Section
      char DataSectionID[4];      // The letters "data"
      uint32_t DataSize;          // Size of the data that follows
    }WavHeader;
    

// speaker settings
#define USE_I2S_SPEAKER_OUTPUT
#define I2S_SPEAKER_SERIAL_CLOCK GPIO_NUM_12
#define I2S_SPEAKER_LEFT_RIGHT_CLOCK GPIO_NUM_11
#define I2S_SPEAKER_SERIAL_DATA GPIO_NUM_13
// Shutdown line if you have this wired up or -1 if you don't
#define I2S_SPEAKER_SD_PIN -1

//------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------------------
// I2S configuration structures

static const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 44100,                            // Note, this will be changed later
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,       // high interrupt priority
    .dma_buf_count = 8,                             // 8 buffers
    .dma_buf_len = 1024,                            // 1K per buffer, so 8K of buffer space
    .use_apll=0,
    .tx_desc_auto_clear= true, 
    .fixed_mclk=-1    
};

// These are the physical wiring connections to our I2S decoder board/chip from the esp32, there are other connections
// required for the chips mentioned at the top (but not to the ESP32), please visit the page mentioned at the top for
// further information regarding these other connections.

static const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SPEAKER_SERIAL_CLOCK,
    .ws_io_num = I2S_SPEAKER_LEFT_RIGHT_CLOCK,
    .data_out_num = I2S_SPEAKER_SERIAL_DATA,
    .data_in_num = I2S_PIN_NO_CHANGE};

//------------------------------------------------------------------------------------------------------------------------


void PrintData(const char* Data,uint8_t NumBytes)
{
    for(uint8_t i=0;i<NumBytes;i++)
      Serial.print(Data[i]); 
      Serial.println();  
}

bool ValidWavData(WavHeader_Struct* Wav)
{
  
  if(memcmp(Wav->RIFFSectionID,"RIFF",4)!=0) 
  {    
    Serial.print("Invlaid data - Not RIFF format");
    return false;        
  }
  if(memcmp(Wav->RiffFormat,"WAVE",4)!=0)
  {
    Serial.print("Invlaid data - Not Wave file");
    return false;           
  }
  if(memcmp(Wav->FormatSectionID,"fmt",3)!=0) 
  {
    Serial.print("Invlaid data - No format section found");
    return false;       
  }
  if(memcmp(Wav->DataSectionID,"data",4)!=0) 
  {
    Serial.print("Invlaid data - data section not found");
    return false;      
  }
  if(Wav->FormatID!=1) 
  {
    Serial.print("Invlaid data - format Id must be 1");
    return false;                          
  }
  if(Wav->FormatSize!=16) 
  {
    Serial.print("Invlaid data - format section size must be 16.");
    return false;                          
  }
  if((Wav->NumChannels!=1)&(Wav->NumChannels!=2))
  {
    Serial.print("Invlaid data - only mono or stereo permitted.");
    return false;   
  }
  if(Wav->SampleRate>48000) 
  {
    Serial.print("Invlaid data - Sample rate cannot be greater than 48000");
    return false;                       
  }
  if(Wav->BitsPerSample!=16) 
  {
    Serial.print("Invlaid data - Only 16 bits per sample permitted.");
    return false;                        
  }
  return true;
}

void DumpWAVHeader(WavHeader_Struct* Wav)
{
  if(memcmp(Wav->RIFFSectionID,"RIFF",4)!=0)
  {
    Serial.print("Not a RIFF format file - ");    
    PrintData(Wav->RIFFSectionID,4);
    return;
  } 
  if(memcmp(Wav->RiffFormat,"WAVE",4)!=0)
  {
    Serial.print("Not a WAVE file - ");  
    PrintData(Wav->RiffFormat,4);  
    return;
  }  
  if(memcmp(Wav->FormatSectionID,"fmt",3)!=0)
  {
    Serial.print("fmt ID not present - ");
    PrintData(Wav->FormatSectionID,3);      
    return;
  } 
  if(memcmp(Wav->DataSectionID,"data",4)!=0)
  {
    Serial.print("data ID not present - "); 
    PrintData(Wav->DataSectionID,4);
    return;
  }  
  // All looks good, dump the data
  Serial.print("Total size :");Serial.println(Wav->Size);
  Serial.print("Format section size :");Serial.println(Wav->FormatSize);
  Serial.print("Wave format :");Serial.println(Wav->FormatID);
  Serial.print("Channels :");Serial.println(Wav->NumChannels);
  Serial.print("Sample Rate :");Serial.println(Wav->SampleRate);
  Serial.print("Byte Rate :");Serial.println(Wav->ByteRate);
  Serial.print("Block Align :");Serial.println(Wav->BlockAlign);
  Serial.print("Bits Per Sample :");Serial.println(Wav->BitsPerSample);
  Serial.print("Data Size :");Serial.println(Wav->DataSize);
}

void play(const unsigned char *WavFile){
  unsigned const char* TheData;
  uint32_t DataIdx=0;                           // index offset into "TheData" for current  data t send to I2S
  memcpy(&WavHeader,WavFile,44);                     // Copy the header part of the wav data into our structure
  DumpWAVHeader(&WavHeader);                          // Dump the header data to serial, optional!
  if(ValidWavData(&WavHeader))
  {
    i2s_set_sample_rates(i2s_num, WavHeader.SampleRate);      //set sample rate 
    TheData=WavFile+44;                                     // set to start of data  

    while (DataIdx<=WavHeader.DataSize){
      uint8_t Mono[4];                             // This holds the data we actually send to the I2S if mono sound
      const unsigned char *Data;                   // Points to the data we are going to send                                              
      size_t BytesWritten;                         // Returned by the I2S write routine, we are not interested in it

      // The WAV Data could be mono or stereo but always 16 bit, that's a data size of 2 byte or 4 bytes  
      // Unfortunatly I2S only allows stereo, so to send mono we have to send the mono sample on both left and right
      // channels. It's a bit of a faf really!
      if(WavHeader.NumChannels==1)     // mono
      {
        Mono[0]=*(TheData+DataIdx);                 // copy the sample to both left and right samples, this is left
        Mono[1]=*(TheData+DataIdx+1);      
        Mono[2]=*(TheData+DataIdx);                 // Same data to the right channel
        Mono[3]=*(TheData+DataIdx+1);     
        Data=Mono;    
      }
      else                            // stereo
        Data=TheData+DataIdx;
      
      i2s_write(i2s_num,Data,4,&BytesWritten,portMAX_DELAY); 
      DataIdx+=WavHeader.BlockAlign;                            // increase the data index to next next sample
    }
  }
}

void setup() {
  Serial.begin(115200);
  i2s_driver_install(i2s_num, &i2s_config, 0, NULL);        // ESP32 will allocated resources to run I2S
  i2s_set_pin(i2s_num, &pin_config);                        // Tell it the pins you will be using
}

void loop()
{   
  delay(2000); 
  play(Exactly_Like_WizChess_Data);
  delay(2000); 
  play(Yer_A_Wizard_Data);
  delay(2000); 
  play(HP_Theme_Data);            
}