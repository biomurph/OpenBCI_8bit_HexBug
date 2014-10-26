/*
 * 
 *  >>>> THIS CODE DESIGNED FOR OBCI_8bit board <<<<
 *  >>>>>>>>  INCORPORATES HEX BUG CONTROLLER MADE BY CHIP AUDETTE  <<<<<<<<<<<
 *  https://github.com/chipaudette/OpenBCI/tree/variant_hexBugControl_visualEntrainment/Arduino
 *
 * This code is written to target an ATmega328P with UNO bootloader. 
 * Adjust as needed if you are testing on different hardware.
 *
 *
 * Made by Joel Murphy, Luke Travis, Conor Russomanno Summer, 2014. 
 * SDcard code is based on RawWrite example in SDFat library 
 * ASCII commands are received on the serial port to configure and control
 * Serial protocol uses '+' immediately before and after the command character
 * We call this the 'burger' protocol. the '+' re the buns. Example:
 * To begin streaming data, this code needs to see '+b+' on the serial port.
 * The included OpenBCI_8bit_Host code is designed to insert the '+' characters.
 * Any PC or mobile device should just send the command character. 
 * OpenBCI_8bit_Host will do the rest. You're welcome.
 *
 * This software is provided as-is with no promise of workability
 * Use at your own risk.
 *
 */

#include <EEPROM.h>
#include <SPI.h>
 #include <SdFat.h>   // not using SD. could be an option later
 #include <SdFatUtil.h>
#include "OpenBCI_04.h"  


//------------------------------------------------------------------------------
//  << SD CARD BUSINESS >> has bee taken out. See OBCI_SD_LOG_CMRR 
//  SD_SS on pin 7 defined in OpenBCI library
//------------------------------------------------------------------------------
//  << OpenBCI BUSINESS >>
#define N_CHANNELS_PER_OPENBCI (8)  //number of channels on a single OpenBCI board
#define MAX_N_CHANNELS (N_CHANNELS_PER_OPENBCI)   //how many channels are available in hardware
//#define MAX_N_CHANNELS (2*N_CHANNELS_PER_OPENBCI)   //how many channels are available in hardware...use this for daisy-chained board
int nActiveChannels = MAX_N_CHANNELS;   //how many active channels would I like?
OpenBCI OBCI; //Uses SPI bus and pins to say data is ready.  Uses Pins 13,12,11,10,9,8,4
// #define MAX_N_CHANNELS (8)  //must be less than or equal to length of channelData in ADS1299 object!!
//int nActiveChannels = 8;   //how many active channels would I like?
byte gainCode = ADS_GAIN24;   //how much gain do I want. adjustable
byte inputType = ADSINPUT_NORMAL;   //here's the normal way to setup the channels. adjustable
boolean is_running = false;    // this flag is set in serialEvent on reciept of ascii prompt
boolean startBecauseOfSerial = false; // not sure this is needed?
byte sampleCounter = 0;
char leadingChar;
// these are used to receive individual channel settings from PC
char currentChannel;    // keep track of what channel we're loading settings for
boolean getChannelSettings = false; // used to receive channel settings command
int channelSettingsCounter; // used to retrieve channel settings from serial port
// these are all subject to the radio requirements: 31byte max packet length (radio maxPacketLength - 1 for checkSum)
#define OUTPUT_NOTHING (0)  // quiet
#define OUTPUT_BINARY (2)  // normal transfer mode
#define OUTPUT_BINARY_SYNTHETIC (3)  // needs portage
#define OUTPUT_BINARY_4CHAN (4)  // needs portage, maybe get rid of it...?
#define OUTPUT_BINARY_WITH_ACCEL (8)  // needs testing
int outputType;

//------------------------------------------------------------------------------
//  << LIS3DH Accelerometer Business >>
//  LIS3DH_SS on pin 5 defined in OpenBCI library
// int axisData[3];  // holds X, Y, Z accelerometer data MOVED TO LIBRARY-JOEL
boolean xyzAvailable = false;
boolean useAccel = false;
//------------------------------------------------------------------------------
//add code for the hex bug
#include "HexBug.h"
//define the pins to use for Hex Bug Control
HexBug_t hexBug(A1,A5,A3,A4,A2); //Ground,Forward,Left, Right, Fire
//------------------------------------------------------------------------------
  
//------------------------------------------------------------------------------

void setup(void) {

  Serial.begin(115200);
  
  
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  
  delay(1000);
 
  OBCI.initialize();  
  Serial.print(F("OpenBCI V3 Stream Data To Dongle\nSetting ADS1299 Channel Values\n"));
//  setup channels on the ADS
// for (int chan=1; chan <= nActiveChannels; chan++) {
//   OBCI.activateChannel(chan); // 
// }
//setup the electrode impedance detection parameters
  OBCI.configure_Zdetect(LOFF_MAG_6NA, LOFF_FREQ_31p2HZ);

  
  Serial.print(F("ADS1299 Device ID: 0x")); Serial.println(OBCI.getADS_ID(),HEX);
  Serial.print(F("LIS3DH Device ID: 0x")); Serial.println(OBCI.getAccelID(),HEX);
  OBCI.printAllRegisters(); //print state of all registers ADS and LIS3DH

  // tell the controlling program that we're ready to start!
  Serial.println(F("Press '?' to query and print ADS1299 register settings again")); //read it straight from flash
  Serial.println(F("Press 1-8 to disable EEG Channels, q-i to enable (all enabled by default)"));
  Serial.println(F("Press 'b' to begin streaming data, press 's' to stop..."));  
  Serial.print(F("Free RAM: ")); Serial.println(FreeRam()); // how much RAM?
}



void loop() {
    
  //update the HexBug
  //Serial.println("loop: hexBug.update() 1");
  hexBug.update();
  
  if(is_running){
    
      while(!(OBCI.isDataAvailable())){   // watch the DRDY pin
        // delayMicroseconds(10); // don't delay!
      }

      OBCI.updateChannelData(); // retrieve the ADS channel data 8x3 bytes
      //Apply  filers to the data here if desired. FILTERS NEEDS int CONVERSION
      if(OBCI.useAccel && OBCI.LIS3DH_DataReady()){
        OBCI.getAccelData();    // fresh axis data goes into the X Y Z 
      }
      OBCI.sendChannelData(sampleCounter);  // 
      sampleCounter++;  
      hexBug.update();
  }

} // end of loop


// some variables to help find 'burger' commands
int plusCounter = 0;
char testChar;
unsigned long commandTimer;

void serialEvent(){
  while(Serial.available()){      
    char inChar = (char)Serial.read();
    
    if(plusCounter == 1){  // if we have received the first 'bun'
      testChar = inChar;   // this might be the 'patty'
      plusCounter++;       // get ready to look for another 'bun'
      commandTimer = millis();  // don't wait too long!
    }
  
    if(inChar == '+'){  // if we see a 'bun' on the serial
      plusCounter++;    // make a note of it
      if(plusCounter == 3){  // looks like we got a command character
        if(millis() - commandTimer < 5){  // if it's not too late,
          if(getChannelSettings){
            loadChannelSettings(testChar);
          }else{
            getCommand(testChar);    // decode the command
          }
        }
        plusCounter = 0;  // get ready for the next one
      }
    }
  }
}
    
    
void getCommand(char token){
    switch (token){
// TURN CHANNELS ON/OFF COMMANDS
      case '1':
        changeChannelState_maintainRunningState(1,DEACTIVATE); break;
      case '2':
        changeChannelState_maintainRunningState(2,DEACTIVATE); break;
      case '3':
        changeChannelState_maintainRunningState(3,DEACTIVATE); break;
      case '4':
        changeChannelState_maintainRunningState(4,DEACTIVATE); break;
      case '5':
        changeChannelState_maintainRunningState(5,DEACTIVATE); break;
      case '6':
        changeChannelState_maintainRunningState(6,DEACTIVATE); break;
      case '7':
        changeChannelState_maintainRunningState(7,DEACTIVATE); break;
      case '8':
        changeChannelState_maintainRunningState(8,DEACTIVATE); break;
      case 'q':
        changeChannelState_maintainRunningState(1,ACTIVATE); break;
      case 'w':
        changeChannelState_maintainRunningState(2,ACTIVATE); break;
      case 'e':
        changeChannelState_maintainRunningState(3,ACTIVATE); break;
      case 'r':
        changeChannelState_maintainRunningState(4,ACTIVATE); break;
      case 't':
        changeChannelState_maintainRunningState(5,ACTIVATE); break;
      case 'y':
        changeChannelState_maintainRunningState(6,ACTIVATE); break;
      case 'u':
        changeChannelState_maintainRunningState(7,ACTIVATE); break;
      case 'i':
        changeChannelState_maintainRunningState(8,ACTIVATE); break;
        
//TURN IMPEDANCE DETECTION ON AND OFF
      case '!':
        changeChannel_Zdetect_maintainRunningState(1,ACTIVATE,PCHAN); break;
      case '@':
        changeChannel_Zdetect_maintainRunningState(2,ACTIVATE,PCHAN); break;
      case '#':
        changeChannel_Zdetect_maintainRunningState(3,ACTIVATE,PCHAN); break;
      case '$':
        changeChannel_Zdetect_maintainRunningState(4,ACTIVATE,PCHAN); break;
      case '%':
        changeChannel_Zdetect_maintainRunningState(5,ACTIVATE,PCHAN); break;
      case '^':
        changeChannel_Zdetect_maintainRunningState(6,ACTIVATE,PCHAN); break;
      case '&':
        changeChannel_Zdetect_maintainRunningState(7,ACTIVATE,PCHAN); break;
      case '*':
        changeChannel_Zdetect_maintainRunningState(8,ACTIVATE,PCHAN); break;
      case 'Q':
        changeChannel_Zdetect_maintainRunningState(1,DEACTIVATE,PCHAN); break;
      case 'W':
        changeChannel_Zdetect_maintainRunningState(2,DEACTIVATE,PCHAN); break;
      case 'E':
        changeChannel_Zdetect_maintainRunningState(3,DEACTIVATE,PCHAN); break;
      case 'R':
        changeChannel_Zdetect_maintainRunningState(4,DEACTIVATE,PCHAN); break;
      case 'T':
        changeChannel_Zdetect_maintainRunningState(5,DEACTIVATE,PCHAN); break;
      case 'Y':
        changeChannel_Zdetect_maintainRunningState(6,DEACTIVATE,PCHAN); break;
      case 'U':
        changeChannel_Zdetect_maintainRunningState(7,DEACTIVATE,PCHAN); break;
      case 'I':
        changeChannel_Zdetect_maintainRunningState(8,DEACTIVATE,PCHAN); break;
       case 'A':
        changeChannel_Zdetect_maintainRunningState(1,ACTIVATE,NCHAN); break;
      case 'S':
        changeChannel_Zdetect_maintainRunningState(2,ACTIVATE,NCHAN); break;
      case 'D':
        changeChannel_Zdetect_maintainRunningState(3,ACTIVATE,NCHAN); break;
      case 'F':
        changeChannel_Zdetect_maintainRunningState(4,ACTIVATE,NCHAN); break;
      case 'G':
        changeChannel_Zdetect_maintainRunningState(5,ACTIVATE,NCHAN); break;
      case 'H':
        changeChannel_Zdetect_maintainRunningState(6,ACTIVATE,NCHAN); break;
      case 'J':
        changeChannel_Zdetect_maintainRunningState(7,ACTIVATE,NCHAN); break;
      case 'K':
        changeChannel_Zdetect_maintainRunningState(8,ACTIVATE,NCHAN); break;
      case 'Z':
        changeChannel_Zdetect_maintainRunningState(1,DEACTIVATE,NCHAN); break;
      case 'X':
        changeChannel_Zdetect_maintainRunningState(2,DEACTIVATE,NCHAN); break;
      case 'C':
        changeChannel_Zdetect_maintainRunningState(3,DEACTIVATE,NCHAN); break;
      case 'V':
        changeChannel_Zdetect_maintainRunningState(4,DEACTIVATE,NCHAN); break;
      case 'B':
        changeChannel_Zdetect_maintainRunningState(5,DEACTIVATE,NCHAN); break;
      case 'N':
        changeChannel_Zdetect_maintainRunningState(6,DEACTIVATE,NCHAN); break;
      case 'M':
        changeChannel_Zdetect_maintainRunningState(7,DEACTIVATE,NCHAN); break;
      case '<':
        changeChannel_Zdetect_maintainRunningState(8,DEACTIVATE,NCHAN); break; 
        
     
//TEST SIGNAL CONTROL COMMANDS
      case '0':
        activateAllChannelsToTestCondition(ADSINPUT_SHORTED,ADSTESTSIG_NOCHANGE,ADSTESTSIG_NOCHANGE); break;
      case '-':
        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_1X,ADSTESTSIG_PULSE_SLOW); break;
      case '=':
        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_1X,ADSTESTSIG_PULSE_FAST); break;
      case 'p':
        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_2X,ADSTESTSIG_DCSIG); break;
      case '[':
        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_2X,ADSTESTSIG_PULSE_SLOW); break;
      case ']':
        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_2X,ADSTESTSIG_PULSE_FAST); break;

//OUTPUT SELECT AND CHANNEL SETTING COMMANDS
      case 'x':  // get ready to receive new settins for a channel
        if(!is_running) {Serial.println(F("ready to accept new channel settings"));}
        channelSettingsCounter = 0;
        getChannelSettings = true; break;
      case 'a':  // update the ADS with all new channel settings
        if(!is_running) {Serial.println(F("updating channel settings"));}
        writeChannelSettingsToADS(); break;
      case 'd':  // reset all channel settings to default
        if(!is_running) {Serial.println(F("updating channel settings do default"));}
        setChannelsToDefaultSetting(); break;
      case 'n':  // start streaming with accelerometer data included
        useAccel = true;
        startRunning(OUTPUT_BINARY_WITH_ACCEL);
        startBecauseOfSerial = is_running;
        break;
      case 'b':
        startRunning(OUTPUT_BINARY);
        OBCI.useAccel = false;
        startBecauseOfSerial = is_running;
        break;
      case 'v':
      // something
        break;
     case 's':
        stopRunning();
        OBCI.useAccel = false;
        startBecauseOfSerial = is_running;  // looking for a good use for these booleans
        break;
     // case 'x':
     //    toggleRunState(OUTPUT_BINARY_SYNTHETIC);
     //    useAccel = false;
     //    startBecauseOfSerial = is_running;
     //    if (is_running) Serial.println(F("OBCI: Starting synthetic..."));
     //    break;
     case '?':
        //print state of all registers
        printRegisters(); break;
      default:
      hexBug.parseCommandCharacter(token);
        break;
      }
  }// end of getCommand


void loadChannelSettings(char c){
  
  if(channelSettingsCounter == 0){  // if it's the first time here, this byte is the channel number to set
    currentChannel = c - '1'; // we just got the channel to load settings into (shift number down for array usage)
    Serial.print(F("loading settings for channel ")); Serial.println(currentChannel+1,DEC);
    channelSettingsCounter++;
    return;
  }
//  setting bytes are in order: POWER_DOWN, GAIN_SET, INPUT_TYPE_SET, BIAS_SET, SRB2_SET, SRB1_SET
  Serial.print(F("load setting ")); Serial.print(channelSettingsCounter-1);
  Serial.print(F(" with ")); Serial.println(c);
  c -= '0';
  if(channelSettingsCounter-1 == GAIN_SET){ c <<= 4; }
  OBCI.ADSchannelSettings[currentChannel][channelSettingsCounter-1] = c;
  channelSettingsCounter++;
  if(channelSettingsCounter == 7){  // 1 currentChannel, plus 6 channelSetting parameters
    Serial.print(F("done receiving settings for channel "));Serial.println(currentChannel+1,DEC);
    getChannelSettings = false;
  }
}

void writeChannelSettingsToADS(){
  boolean is_running_when_called = is_running;
  int cur_outputType = outputType; 
  stopRunning();                   //must stop running to change channel settings
  OBCI.updateChannelSettings();    // change the channel settings
  if (is_running_when_called == true) {
    startRunning(cur_outputType);  //restart, if it was running before
  }
}

void setChannelsToDefaultSetting(){
  boolean is_running_when_called = is_running;
  int cur_outputType = outputType; 
  stopRunning();  //must stop running to change channel settings
  OBCI.setChannelsToDefault();   // default channel settings
  if (is_running_when_called == true) {
    startRunning(cur_outputType);  //restart, if it was running before
  }
}

boolean stopRunning(void) {
  if(is_running == true){
    OBCI.stopStreaming();                    // stop the data acquisition  //
    is_running = false;
    return is_running;
  }
}

boolean startRunning(int OUT_TYPE) {
  if(is_running == false){
    outputType = OUT_TYPE;
    OBCI.startStreaming();    //start the data acquisition NOT BUILT include accel if needed
    is_running = true;
  }
    return is_running;
}


int changeChannelState_maintainRunningState(int chan, int start)
{
  boolean is_running_when_called = is_running;
  int cur_outputType = outputType;
  
  //must stop running to change channel settings
  stopRunning();
  if (start == true) {
    if(is_running_when_called == false){
      Serial.print(F("Activating channel "));
      Serial.println(chan);
    }
    OBCI.activateChannel(chan);
  } else {
    if(is_running_when_called == false){
      Serial.print(F("Deactivating channel "));
      Serial.println(chan);
    }
    OBCI.deactivateChannel(chan);
  }
  //restart, if it was running before
  if (is_running_when_called == true) {
    startRunning(cur_outputType);
  }
}

// CALLED WHEN COMMAND CHARACTER IS SEEN ON THE SERIAL PORT
int activateAllChannelsToTestCondition(int testInputCode, byte amplitudeCode, byte freqCode)
{
  boolean is_running_when_called = is_running;
  int cur_outputType = outputType;
  
  //must stop running to change channel settings
  stopRunning();
  //set the test signal to the desired state
  OBCI.configureInternalTestSignal(amplitudeCode,freqCode);    
  //loop over all channels to change their state
  for (int Ichan=1; Ichan <= 8; Ichan++) {
    OBCI.activateChannel(Ichan,gainCode,testInputCode,false);  //Ichan must be [1 8]...it does not start counting from zero
  }
  //restart, if it was running before
  if (is_running_when_called == true) {
    startRunning(cur_outputType);
  }
}

int changeChannel_Zdetect_maintainRunningState(int chan, int start, int code_P_N_Both)
{
  boolean is_running_when_called = is_running;
  int cur_outputType = outputType;
  
  //must stop running to change channel settings
  stopRunning();
  if (start == true) {
    Serial.print(F("Activating channel "));
    Serial.print(chan);
    Serial.println(F(" Lead-Off Detection"));
    OBCI.changeChannel_Zdetect(chan,ON,code_P_N_Both);
  } else {
    Serial.print(F("Deactivating channel "));
    Serial.print(chan);
    Serial.println(F(" Lead-Off Detection"));
    OBCI.changeChannel_Zdetect(chan,OFF,code_P_N_Both);
  }
  
  //restart, if it was running before
  if (is_running_when_called == true) {
    startRunning(cur_outputType);
  }
}

void printRegisters(){
  if(is_running == false){
    // print the ADS and LIS3DH registers
    OBCI.printAllRegisters();
  }
}



// end

