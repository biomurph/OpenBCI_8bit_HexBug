

#include "ADS1299.h"

//void ADS1299::initialize(int _CS){
  void ADS1299::initialize(){
//    DRDY = _DRDY;
//    CS = _CS;
//    RST = _RST;
// recommended power up sequence requiers Tpor (~32mS)			
    delay(50);				
    pinMode(ADS_RST,OUTPUT);
    digitalWrite(ADS_RST,LOW);
    delayMicroseconds(4);	// toggle reset pin
    digitalWrite(ADS_RST,HIGH);
    delayMicroseconds(20);	// recommended to wait 18 Tclk before using device (~8uS);    
// initalize the  data ready chip select and reset pins:
    pinMode(ADS_DRDY, INPUT);
    pinMode(CS, OUTPUT); digitalWrite(CS,HIGH); 
    pinMode(ADS_SS, OUTPUT); digitalWrite(ADS_SS,HIGH);	
    delay(100);
	
    resetADS();
    // DEFAULT CHANNEL SETTINGS FOR ADS
    defaultChannelSettings[POWER_DOWN] = NO;        // on = NO, off = YES
    defaultChannelSettings[GAIN_SET] = ADS_GAIN24;     // Gain setting
    defaultChannelSettings[INPUT_TYPE_SET] = ADSINPUT_NORMAL;// input muxer setting
    defaultChannelSettings[BIAS_SET] = YES;    // add this channel to bias generation
    defaultChannelSettings[SRB2_SET] = NO;       // connect this P side to SRB2
    defaultChannelSettings[SRB1_SET] = NO;        // don't use SRB1
    for(int i=0; i<8; i++){
      for(int j=0; j<6; j++){
        channelSettings[i][j] = defaultChannelSettings[j];
      }
    }
    writeChannelSettings();
    
    verbosity = false;      // when verbosity is true, there will be Serial feedback
};


// ADS Slave Select
void ADS1299::csLow(void)
{
  SPI.setDataMode(SPI_MODE1);
  digitalWrite(ADS_SS, LOW);
  
}

void ADS1299::csHigh(void)
{
  digitalWrite(ADS_SS, HIGH);
  SPI.setDataMode(SPI_MODE0);
}

//SPI communication method
byte ADS1299::xfer(byte _data) {
  cli();
    SPDR = _data;
    while (!(SPSR & _BV(SPIF)));
  sei();
    return SPDR;
}


//reset all the ADS1299's settings.  Call however you'd like.  Stops all data acquisition
void ADS1299::resetADS(void)
{
  RESET();             // send RESET command to default all registers
  SDATAC();            // exit Read Data Continuous mode to communicate with ADS
  
  delay(100);
    
  // turn off all channels
  for (int chan=1; chan <= 8; chan++) {
    deactivateChannel(chan);
  } 
  
};

void ADS1299::writeChannelSettings(void){
  byte setting;
  boolean use_SRB1 = false;
//proceed...first, disable any data collection
  SDATAC(); delay(1);      // exit Read Data Continuous mode to communicate with ADS

  for(byte i=0; i<8; i++){ // write 8 channel settings
    setting = 0x00;
    if(channelSettings[i][POWER_DOWN] == YES) setting |= 0x80;
    setting |= channelSettings[i][GAIN_SET]; // gain
    setting |= channelSettings[i][INPUT_TYPE_SET]; // input code
    if(channelSettings[i][SRB2_SET] == YES) setting |= 0x08; // close this SRB2 switch
    WREG(CH1SET+i, setting);  // write this channel's register settings
    
      // add or remove from inclusion in BIAS generation
      setting = RREG(BIAS_SENSP);       //get the current P bias settings
      if(channelSettings[i][BIAS_SET] == YES){
        bitSet(setting,i);    //set this channel's bit to add it to the bias generation
      }else{
        bitClear(setting,i);  // clear this channel's bit to remove from bias generation
      }
      WREG(BIAS_SENSP,setting); delay(1); //send the modified byte back to the ADS
      setting = RREG(BIAS_SENSN);       //get the current N bias settings
      if(channelSettings[i][BIAS_SET] == YES){
        bitSet(setting,i);    //set this channel's bit to add it to the bias generation
      }else{
        bitClear(setting,i);  // clear this channel's bit to remove from bias generation
      }
      WREG(BIAS_SENSN,setting); delay(1); //send the modified byte back to the ADS
      
    if(channelSettings[i][SRB1_SET] == YES){
      use_SRB1 = true;
    }
  }
    if(use_SRB1){
      WREG(MISC1,0x20);     // close all SRB1 swtiches
    }
    WREG(CONFIG3,0b11101100); delay(1);
}
//  deactivate the given channel...note: stops data colleciton to issue its commands
//  N is the channel number: 1-8
void ADS1299::deactivateChannel(int N)
{
  byte reg, setting;
  
  //check the inputs
  if ((N < 1) || (N > 8)) return;
  
  //proceed...first, disable any data collection
  SDATAC(); delay(1);      // exit Read Data Continuous mode to communicate with ADS

  //shut down the channel
  N = constrain(N-1,0,7);  //subtracts 1 so that we're counting from 0, not 1
  reg = CH1SET+(byte)N;						// select the current channel
  setting = RREG(reg); delay(1);	// get the current channel settings
  bitSet(setting,7);  						// set bit7 to shut down channel
  if (channelSettings[N][SRB2_SET] == YES) bitClear(setting,3);  	// clear bit3 to disclude from SRB2 if used
  WREG(reg,setting); delay(1);	    // write the new value to disable the channel
  
  //remove the channel from the bias generation...
  reg = BIAS_SENSP; 					 	// set up to disconnect the P inputs from Bias generation
  setting = RREG(reg); delay(1);	//get the current bias settings
  bitClear(setting,N);          				//clear this channel's bit to remove from bias generation
  WREG(reg,setting); delay(1); 	//send the modified byte back to the ADS

  reg = BIAS_SENSN; 						// set up to disconnect the N inputs from Bias generation
  setting = RREG(reg); delay(1);	//get the current bias settings
  bitClear(setting,N);          				//clear this channel's bit to remove from bias generation
  WREG(reg,setting); delay(1);  	//send the modified byte back to the ADS
}; 

//  Active a channel using stored channelSettings[][]  
//  N is 1 through 8
void ADS1299::activateChannel(int N) 
{
	byte reg, setting;
   //check the inputs
  if ((N < 1) || (N > 8)) return;
  N = constrain(N-1,0,7);  //shift down by one
  //proceed...first, disable any data collection
  SDATAC(); delay(1);      // exit Read Data Continuous mode to communicate with ADS
    setting = 0x00;
    setting |= channelSettings[N][GAIN_SET]; // gain
    setting |= channelSettings[N][INPUT_TYPE_SET]; // input code
    if(channelSettings[N][SRB2_SET] == YES) bitSet(setting,3); // close this SRB2 switch
    WREG(CH1SET+N, setting);
    // add or remove from inclusion in BIAS generation
      setting = RREG(BIAS_SENSP);       //get the current P bias settings
      if(channelSettings[N][BIAS_SET] == YES){
        bitSet(setting,N);    //set this channel's bit to add it to the bias generation
      }else{
        bitClear(setting,N);  // clear this channel's bit to remove from bias generation
      }
      WREG(BIAS_SENSP,setting); delay(1); //send the modified byte back to the ADS
      setting = RREG(BIAS_SENSN);       //get the current N bias settings
      if(channelSettings[N][BIAS_SET] == YES){
        bitSet(setting,N);    //set this channel's bit to add it to the bias generation
      }else{
        bitClear(setting,N);  // clear this channel's bit to remove from bias generation
      }
      WREG(BIAS_SENSN,setting); delay(1); //send the modified byte back to the ADS
      
    setting = 0x00;
    if(channelSettings[N][SRB1_SET] == YES){
      setting = 0x20;
    }
    WREG(MISC1,setting);     // close all SRB1 swtiches
  //Finalize the bias setup...activate buffer and use internal reference for center of bias creation, datasheet PDF p42
  WREG(CONFIG3,0b11101100); delay(1);   // THIS COULD MOVE??
};

// activate a channel and specify the gainCode and inputCode to use. this is holdover from V2 code
void ADS1299::activateChannel(int N,byte gainCode,byte inputCode, boolean useInBias) 
{
  byte reg, setting;
   //check the inputs
  if ((N < 1) || (N > 8)) return;
  N = constrain(N-1,0,7);  //shift down by one for ease of use
  SDATAC(); delay(1);      // exit Read Data Continuous mode to communicate with ADS
  //active the channel using the given gain.
  //see ADS1299 datasheet, PDF p44
  setting = 0b00000000;  						//left-most zero (bit 7) is to activate the channel
  gainCode = gainCode & 0b01110000;  			//bitwise AND to get just the bits we want and set the rest to zero
  setting = setting | gainCode; 					//bitwise OR to set just the gain bits high or low and leave the rest
  inputCode = inputCode & 0b00000111;  		//bitwise AND to get just the bits we want and set the rest to zero
  setting = setting | inputCode; 					//bitwise OR to set just the gain bits high or low and leave the rest

  if (channelSettings[N][SRB2_SET] == YES) setting |= 0b00001000;  	//set the SRB2 flag if you plan to use it
  WREG(CH1SET+(byte)N,setting); delay(1);

  // add or remove from inclusion in BIAS generation
      setting = RREG(BIAS_SENSP);       //get the current P bias settings
      if(useInBias){
        bitSet(setting,N);    //set this channel's bit to add it to the bias generation
      }else{
        bitClear(setting,N);  // clear this channel's bit to remove from bias generation
      }
      WREG(BIAS_SENSP,setting); delay(1); //send the modified byte back to the ADS
      setting = RREG(BIAS_SENSN);       //get the current N bias settings
      if(useInBias){
        bitSet(setting,N);    //set this channel's bit to add it to the bias generation
      }else{
        bitClear(setting,N);  // clear this channel's bit to remove from bias generation
      }
      WREG(BIAS_SENSN,setting); delay(1); //send the modified byte back to the ADS
  setting = 0x00;
  if(channelSettings[N][SRB1_SET] == YES){
    setting = 0x20;
  }
  WREG(MISC1,setting);     // close all SRB1 swtiches
  //Finalize the bias setup...activate buffer and use internal reference for center of bias creation, datasheet PDF p42
  WREG(CONFIG3,0b11101100); delay(1);   // THIS COULD MOVE TO THE INITIALIZATION
};


// Start continuous data acquisition
void ADS1299::startADS(void)
{
    RDATAC(); 
  delay(1);   // enter Read Data Continuous mode
    START();        // start the data acquisition
  delay(1);
    isRunning = true;
}
  
// Query to see if data is available from the ADS1299...return TRUE is data is available
boolean ADS1299::isDataAvailable(void)
{
  return (!(digitalRead(ADS_DRDY)));
}
  
// Get ADS channel data when DRDY goes low
void ADS1299::updateChannelData(){
    byte inByte;
    int byteCounter = 0;
    csLow();        //  open SPI
    for(int i=0; i<3; i++){ 
      inByte = xfer(0x00);    //  read status register (1100 + LOFF_STATP + LOFF_STATN + GPIO[7:4])
      stat = (stat << 8) | inByte;
    }
    for(int i = 0; i<8; i++){
      for(int j=0; j<3; j++){   //  read 24 bits of channel data in 8 3 byte chunks
        inByte = xfer(0x00);
        bit24ChannelData[byteCounter] = inByte;  // raw data gets streamed to the radio
        byteCounter++;
        // THIS INT ARRAY IS LEGACY
        channelData[i] = (channelData[i]<<8) | inByte;
      }
    }
    csHigh();       //  close SPI
// <<<<<<<<<<<<<<<   THIS INT CONVERSION IS LEGACY   >>>>>>>>>>>>>>>>>>>
// need to convert 24bit to 32bit if using a filter
  for(int i=0; i<8; i++){     // convert 3 byte 2's compliment to 4 byte 2's compliment 
    if(bitRead(channelData[i],23) == 1){  
      channelData[i] |= 0xFF000000;
    }else{
      channelData[i] &= 0x00FFFFFF;
    }
  }
// <<<<<<<<<<<<<<<   THIS INT CONVERSION IS LEGACY  >>>>>>>>>>>>>>>>>>>>
}

//write as binary each channel's 3 bytes of data
void ADS1299::writeADSchannelData(void)
{
  //print rawChannelData array
  for (int i = 0; i < 24; i++)
  {
    Serial.write(bit24ChannelData[i]); 
  }
}

// Stop the continuous data acquisition
void ADS1299::stopADS(void)
{
    STOP();
  delay(1);       // start the data acquisition
    SDATAC();
  delay(1);       // exit Read Data Continuous mode to communicate with ADS
    isRunning = false;
}

// SRB1 is used when connecting to the P channel inputs
//void ADS1299::setSRB1(boolean desired_state) {
//	if (desired_state) {
//		WREG(MISC1,0b00100000); delay(1);  //ADS1299 datasheet, PDF p46
//	} else {
//		WREG(MISC1,0b00000000); delay(1);  //ADS1299 datasheet, PDF p46
//	}
//}


void ADS1299::changeChannelLeadOffDetection(int N, int code_OFF_ON, int code_P_N_Both)
{
  byte reg, setting;
	
  //check the inputs
  if ((N < 1) || (N > 8)) return;
  N = constrain(N-1,0,7);  //shift down by one
  
  //proceed...first, disable any data collection
  ADS1299::SDATAC(); delay(1);      // exit Read Data Continuous mode to communicate with ADS

  if ((code_P_N_Both == PCHAN) || (code_P_N_Both == BOTHCHAN)) {
  	  //shut down the lead-off signal on the positive side
  	  reg = LOFF_SENSP;  //are we using the P inptus or the N inputs?
  	  setting = ADS1299::RREG(reg); //get the current lead-off settings
  	  if (code_OFF_ON == OFF) {
  	  	  bitClear(setting,N);                   //clear this channel's bit
  	  } else {
  	  	  bitSet(setting,N); 			  //clear this channel's bit
  	  }
  	  ADS1299::WREG(reg,setting); delay(1);  //send the modified byte back to the ADS
  }
  
  if ((code_P_N_Both == NCHAN) || (code_P_N_Both == BOTHCHAN)) {
  	  //shut down the lead-off signal on the negative side
  	  reg = LOFF_SENSN;  //are we using the P inptus or the N inputs?
  	  setting = ADS1299::RREG(reg); //get the current lead-off settings
  	  if (code_OFF_ON == OFF) {
  	  	  bitClear(setting,N);                   //clear this channel's bit
  	  } else {
  	  	  bitSet(setting,N); 			  //clear this channel's bit
  	  }           //set this channel's bit
  	  ADS1299::WREG(reg,setting); delay(1);  //send the modified byte back to the ADS
  }
}; 

void ADS1299::configureLeadOffDetection(byte amplitudeCode, byte freqCode)
{
	amplitudeCode &= 0b00001100;  //only these two bits should be used
	freqCode &= 0b00000011;  //only these two bits should be used
	
	//get the current configuration of he byte
	byte reg, setting;
	reg = LOFF;
	setting = ADS1299::RREG(reg); //get the current bias settings
	
	//reconfigure the byte to get what we want
	setting &= 0b11110000;  //clear out the last four bits
	setting |= amplitudeCode;  //set the amplitude
	setting |= freqCode;    //set the frequency
	
	//send the config byte back to the hardware
	ADS1299::WREG(reg,setting); delay(1);  //send the modified byte back to the ADS
	
}

//Configure the test signals that can be inernally generated by the ADS1299
void ADS1299::configureInternalTestSignal(byte amplitudeCode, byte freqCode)
{
	if (amplitudeCode == ADSTESTSIG_NOCHANGE) amplitudeCode = (RREG(CONFIG2) & (0b00000100));
	if (freqCode == ADSTESTSIG_NOCHANGE) freqCode = (RREG(CONFIG2) & (0b00000011));
	freqCode &= 0b00000011;  		//only the last two bits are used
	amplitudeCode &= 0b00000100;  	//only this bit is used
	byte message = 0b11010000 | freqCode | amplitudeCode;  //compose the code
	
	WREG(CONFIG2,message); delay(1);
	
}
 
//only use SRB1 if all use_SRB2 are set to false
//boolean ADS1299::use_SRB1(void) {
//	for (int Ichan=0; Ichan < OPENBCI_NCHAN; Ichan++) {
//		if (use_SRB2[Ichan]) {
//			return false;
//		}
//	}
//	return true;
//}

//  <<<<<<  SYSTEM COMMANDS  >>>>>>
void ADS1299::WAKEUP() {
    csLow(); 
    xfer(_WAKEUP);
    csHigh(); 
    delayMicroseconds(3);     //must wait 4 tCLK cycles before sending another command (Datasheet, pg. 35)
}

void ADS1299::STANDBY() {   // only allowed to send WAKEUP after sending STANDBY
    csLow();
    xfer(_STANDBY);
    csHigh();
}

void ADS1299::RESET() {     // reset all the registers to default settings
    csLow();
    xfer(_RESET);
    delayMicroseconds(12);    //must wait 18 tCLK cycles to execute this command (Datasheet, pg. 35)
    csHigh();
}

void ADS1299::START() {     //start data conversion 
    csLow();
    xfer(_START);
    csHigh();
}

void ADS1299::STOP() {      //stop data conversion
    csLow();
    xfer(_STOP);
    csHigh();
}

void ADS1299::RDATAC() {
    csLow();
    xfer(_RDATAC);
    csHigh();
  delayMicroseconds(3);   
}
void ADS1299::SDATAC() {
    csLow();
    xfer(_SDATAC);
    csHigh();
  delayMicroseconds(3);   //must wait 4 tCLK cycles after executing this command (Datasheet, pg. 37)
}

void ADS1299::RDATA() {         //  use in Stop Read Continuous mode when DRDY goes low
  byte inByte;            //  to read in one sample of the channels
    csLow();        //  open SPI
    xfer(_RDATA);         //  send the RDATA command
  stat = xfer(0x00);        //  read status register (1100 + LOFF_STATP + LOFF_STATN + GPIO[7:4])
  for(int i = 0; i<8; i++){
    for(int j=0; j<3; j++){   //  read in the status register and new channel data
      inByte = xfer(0x00);
      channelData[i] = (channelData[i]<<8) | inByte;
    }
  }
  csHigh();       //  close SPI
  
  for(int i=0; i<8; i++){
    if(bitRead(channelData[i],23) == 1){  // convert 3 byte 2's compliment to 4 byte 2's compliment
      channelData[i] |= 0xFF000000;
    }else{
      channelData[i] &= 0x00FFFFFF;
    }
  }
}

// REGISTER READ/WRITE COMMANDS
byte ADS1299::RREG(byte _address) {   //  reads ONE register at _address
    byte opcode1 = _address + 0x20;   //  RREG expects 001rrrrr where rrrrr = _address
    csLow();        //  open SPI
    xfer(opcode1);          //  opcode1
    xfer(0x00);           //  opcode2
    regData[_address] = xfer(0x00);//  update mirror location with returned byte
    csHigh();       //  close SPI 
  if (verbosity){           //  verbosity output
    printRegisterName(_address);
    printHex(_address);
    Serial.print(F(", "));
    printHex(regData[_address]);
    Serial.print(F(", "));
    for(byte j = 0; j<8; j++){
      Serial.print(bitRead(regData[_address], 7-j));
      if(j!=7) Serial.print(F(", "));
    }
    
    Serial.println();
  }
  return regData[_address];     // return requested register value
}

// Read more than one register starting at _address
void ADS1299::RREGS(byte _address, byte _numRegistersMinusOne) {
//  for(byte i = 0; i < 0x17; i++){
//    regData[i] = 0;         //  reset the regData array
//  }
    byte opcode1 = _address + 0x20;   //  RREG expects 001rrrrr where rrrrr = _address
    csLow();        //  open SPI
    xfer(opcode1);          //  opcode1
    xfer(_numRegistersMinusOne);  //  opcode2
    for(int i = 0; i <= _numRegistersMinusOne; i++){
        regData[_address + i] = xfer(0x00);   //  add register byte to mirror array
    }
    csHigh();       //  close SPI
  if(verbosity){            //  verbosity output
    for(int i = 0; i<= _numRegistersMinusOne; i++){
      printRegisterName(_address + i);
      printHex(_address + i);
      Serial.print(F(", "));
      printHex(regData[_address + i]);
      Serial.print(F(", "));
      for(int j = 0; j<8; j++){
        Serial.print(bitRead(regData[_address + i], 7-j));
        if(j!=7) Serial.print(F(", "));
      }
      Serial.println();
      delay(30);
    }
    }
}

void ADS1299::WREG(byte _address, byte _value) {  //  Write ONE register at _address
    byte opcode1 = _address + 0x40;   //  WREG expects 010rrrrr where rrrrr = _address
    csLow();        //  open SPI
    xfer(opcode1);          //  Send WREG command & address
    xfer(0x00);           //  Send number of registers to read -1
    xfer(_value);         //  Write the value to the register
    csHigh();       //  close SPI
  regData[_address] = _value;     //  update the mirror array
  if(verbosity){            //  verbosity output
    Serial.print(F("Register "));
    printHex(_address);
    Serial.println(F(" modified."));
  }
}

void ADS1299::WREGS(byte _address, byte _numRegistersMinusOne) {
    byte opcode1 = _address + 0x40;   //  WREG expects 010rrrrr where rrrrr = _address
    csLow();        //  open SPI
    xfer(opcode1);          //  Send WREG command & address
    xfer(_numRegistersMinusOne);  //  Send number of registers to read -1 
  for (int i=_address; i <=(_address + _numRegistersMinusOne); i++){
    xfer(regData[i]);     //  Write to the registers
  } 
  digitalWrite(CS,HIGH);        //  close SPI
  if(verbosity){
    Serial.print(F("Registers "));
    printHex(_address); Serial.print(F(" to "));
    printHex(_address + _numRegistersMinusOne);
    Serial.println(F(" modified"));
  }
}

void ADS1299::printDeviceID(void)
{
    boolean wasRunning;
    boolean prevverbosityState = verbosity;
    if (isRunning){ stopADS(); wasRunning = true;}
        verbosity = true;
        getDeviceID();
        verbosity = prevverbosityState;
    if (wasRunning){ startADS(); }
        
}

byte ADS1299::getDeviceID() {     // simple hello world com check
  byte data = RREG(0x00);
  if(verbosity){            // verbosity otuput
    Serial.print(F("Device ID "));
    printHex(data); 
        Serial.println();
  }
  return data;
}

//print out the state of all the control registers
void ADS1299::printAllRegisters(void)   
{
    boolean wasRunning = false;
    boolean prevverbosityState = verbosity;
    if (isRunning){ stopADS(); wasRunning = true; }
        verbosity = true;           // set up for verbosity output
        RREGS(0x00,0x17);       // read out the first registers
//        delay(10);              // stall to let all that data get read by the PC
//        RREGS(0x11,0x17-0x11);  // read out the rest
        verbosity = prevverbosityState;
    if (wasRunning){ startADS(); }
}

// String-Byte converters for RREG and WREG
void ADS1299::printRegisterName(byte _address) {
    if(_address == ID){ // CHANGE THIS TO SWITCH/CASE
        Serial.print(F("ID, "));
    }
    else if(_address == CONFIG1){
        Serial.print(F("CONFIG1, "));
    }
    else if(_address == CONFIG2){
        Serial.print(F("CONFIG2, "));
    }
    else if(_address == CONFIG3){
        Serial.print(F("CONFIG3, "));
    }
    else if(_address == LOFF){
        Serial.print(F("LOFF, "));
    }
    else if(_address == CH1SET){
        Serial.print(F("CH1SET, "));
    }
    else if(_address == CH2SET){
        Serial.print(F("CH2SET, "));
    }
    else if(_address == CH3SET){
        Serial.print(F("CH3SET, "));
    }
    else if(_address == CH4SET){
        Serial.print(F("CH4SET, "));
    }
    else if(_address == CH5SET){
        Serial.print(F("CH5SET, "));
    }
    else if(_address == CH6SET){
        Serial.print(F("CH6SET, "));
    }
    else if(_address == CH7SET){
        Serial.print(F("CH7SET, "));
    }
    else if(_address == CH8SET){
        Serial.print(F("CH8SET, "));
    }
    else if(_address == BIAS_SENSP){
        Serial.print(F("BIAS_SENSP, "));
    }
    else if(_address == BIAS_SENSN){
        Serial.print(F("BIAS_SENSN, "));
    }
    else if(_address == LOFF_SENSP){
        Serial.print(F("LOFF_SENSP, "));
    }
    else if(_address == LOFF_SENSN){
        Serial.print(F("LOFF_SENSN, "));
    }
    else if(_address == LOFF_FLIP){
        Serial.print(F("LOFF_FLIP, "));
    }
    else if(_address == LOFF_STATP){
        Serial.print(F("LOFF_STATP, "));
    }
    else if(_address == LOFF_STATN){
        Serial.print(F("LOFF_STATN, "));
    }
    else if(_address == GPIO){
        Serial.print(F("GPIO, "));
    }
    else if(_address == MISC1){
        Serial.print(F("MISC1, "));
    }
    else if(_address == MISC2){
        Serial.print(F("MISC2, "));
    }
    else if(_address == CONFIG4){
        Serial.print(F("CONFIG4, "));
    }
}

// Used for printing HEX in verbosity feedback mode
void ADS1299::printHex(byte _data){
  Serial.print(F("0x"));
    if(_data < 0x10) Serial.print(F("0"));
    Serial.print(_data, HEX);
}
