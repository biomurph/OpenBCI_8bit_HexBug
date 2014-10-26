

#include "OpenBCI_04.h"

void OpenBCI::initialize(void){
  pinMode(SD_SS,OUTPUT); digitalWrite(SD_SS,HIGH);  // de-select the SD card if it's there
  // do other SD card setup here?
  initialize_ads();
  initialize_accel();
}

void OpenBCI::initialize_ads(void) {
  ads.initialize();  // adjust here to test daisy module!
  for(int i=0; i<8; i++){
    for(int j=0; j<6; j++){
      ADSchannelSettings[i][j] = ads.channelSettings[i][j];
    }
  }
}

void OpenBCI::initialize_accel(void) {
  accel.initialize();
}

void OpenBCI::enable_accel(void) {
  accel.enable_accel();
}

void OpenBCI::disable_accel(void){
  accel.disable_accel();
}

byte OpenBCI::getAccelID(void){
  return accel.getDeviceID();
}

boolean OpenBCI::LIS3DH_DataReady(void){
  return (accel.LIS3DH_DataReady());
}

void OpenBCI::getAccelData(void){
  accel.LIS3DH_updateAxisData();
}


int OpenBCI::getX(void){ return accel.getX(); }
int OpenBCI::getY(void){ return accel.getY(); }
int OpenBCI::getZ(void){ return accel.getZ(); }



// ADS FUNCTIONS
void OpenBCI::printAllRegisters(void) {
  Serial.println("\nADS Registers:");
  ads.printAllRegisters();
  delay(100);
  Serial.println("LIS3DH Registers:");
  delay(100);
  accel.readAllRegs();
}

byte OpenBCI::getADS_ID(void){
  return ads.getDeviceID();
}

void OpenBCI::setChannelsToDefault(void){
  ADSchannelSettings[0][POWER_DOWN] = NO;        // NO = on, YES = off
  ADSchannelSettings[0][GAIN_SET] = ADS_GAIN24;     // Gain setting
  ADSchannelSettings[0][INPUT_TYPE_SET] = ADSINPUT_NORMAL;// input muxer setting
  ADSchannelSettings[0][BIAS_SET] = YES;    // add this channel to bias generation
  ADSchannelSettings[0][SRB2_SET] = YES;       // connect this P side to SRB2
  ADSchannelSettings[0][SRB1_SET] = NO;        // don't use SRB1
  for(int i=1; i<8; i++){
    for(int j=0; j<6; j++){
      ADSchannelSettings[i][j] = ADSchannelSettings[0][j];
    }
  }
  updateChannelSettings();
}

void OpenBCI::setChannelsToEMG(void){
  ADSchannelSettings[0][POWER_DOWN] = NO;        // NO = on, YES = off
  ADSchannelSettings[0][GAIN_SET] = ADS_GAIN24;     // Gain setting
  ADSchannelSettings[0][INPUT_TYPE_SET] = ADSINPUT_NORMAL;// input muxer setting
  ADSchannelSettings[0][BIAS_SET] = NO;    // add this channel to bias generation
  ADSchannelSettings[0][SRB2_SET] = NO;       // connect this P side to SRB2
  ADSchannelSettings[0][SRB1_SET] = NO;        // don't use SRB1
  for(int i=1; i<8; i++){
    for(int j=0; j<6; j++){
      ADSchannelSettings[i][j] = ADSchannelSettings[0][j];
    }
  }
  updateChannelSettings();
}

void OpenBCI::setChannelsToECG(void){
  ADSchannelSettings[0][POWER_DOWN] = NO;        // NO = on, YES = off
  ADSchannelSettings[0][GAIN_SET] = ADS_GAIN24;     // Gain setting
  ADSchannelSettings[0][INPUT_TYPE_SET] = ADSINPUT_NORMAL;// input muxer setting
  ADSchannelSettings[0][BIAS_SET] = YES;    // add this channel to bias generation
  ADSchannelSettings[0][SRB2_SET] = YES;       // connect this P side to SRB2
  ADSchannelSettings[0][SRB1_SET] = NO;        // don't use SRB1
  for(int i=1; i<8; i++){
    for(int j=0; j<6; j++){
      ADSchannelSettings[i][j] = ADSchannelSettings[0][j];
    }
  }
  updateChannelSettings();
}

void OpenBCI::updateChannelSettings(void){
  for(int i=0; i<8; i++){
    for(int j=0; j<6; j++){
      ads.channelSettings[i][j] = ADSchannelSettings[i][j];
    }
  }
  writeADSchannelSettings();
//  ads.writeChannelSettings();
}

void OpenBCI::writeADSchannelSettings(void){
  ads.writeChannelSettings();
}

void OpenBCI::activateChannel(int chan){
  ads.activateChannel(chan);
}

void OpenBCI::activateChannel(int chan, byte gainCode, byte inputType, boolean useInBias){
  ads.activateChannel(chan, gainCode, inputType, useInBias);
}

void OpenBCI::deactivateChannel(int N){
  ads.deactivateChannel(N); 
}

void OpenBCI::startStreaming(void){
  ads.startADS();
  if(useAccel){accel.enable_accel();}
}

void OpenBCI::stopStreaming(void){
  ads.stopADS();
  accel.disable_accel();
}

void OpenBCI::reset_ads(void){
  ads.resetADS();
}

boolean OpenBCI::isDataAvailable(void){
  return ads.isDataAvailable();
}

void OpenBCI::updateChannelData(void){
  ads.updateChannelData();
}

void OpenBCI::setSRB1(boolean desired_state){
  ads.setSRB1(desired_state);
}

void OpenBCI::sendChannelData(byte sampleNumber){
  Serial.write(sampleNumber); // 1 byte
  ads.writeADSchannelData();  // 24 bytes
  accel.writeLIS3DHdata();    // 6 bytes
}

long OpenBCI::getChannel(int chan){
  return ads.channelData[chan];
}


void OpenBCI::putChannel(int chan, long val){
  ads.channelData[chan] = val;
}

void OpenBCI::update24bitData(){
  int indexCounter = 0;
  for(int i=0; i<8; i++){
    for(int j=2; j>=0; j--){
      ads.bit24ChannelData[indexCounter] = byte(ads.channelData[i] >> (8*j));
      indexCounter++;
    }
  }
}

// Electrode-Off Detection Functions

void OpenBCI::changeChannel_Zdetect(int N, int code_OFF_ON, int code_P_N_Both){
  ads.changeChannelLeadOffDetection(N, code_OFF_ON, code_P_N_Both);
}

void OpenBCI::configure_Zdetect(byte amplitudeCode, byte freqCode){
  ads.configureLeadOffDetection(amplitudeCode, freqCode);
}


//Configure the test signals that can be inernally generated by the ADS1299
void OpenBCI::configureInternalTestSignal(byte amplitudeCode, byte freqCode)
{
  ads.configureInternalTestSignal(amplitudeCode, freqCode);
}

