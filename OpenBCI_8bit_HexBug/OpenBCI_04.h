

#ifndef _____OpenBCI_04__
#define _____OpenBCI_04__

#include <SdFat.h>
#include <SdFatUtil.h>

#include "ADS1299.h"
#include "LIS3DH.h"
#include "Definitions.h"

class OpenBCI {

  public:
  
    ADS1299 ads;
    LIS3DH accel;
//    SdFat card;
    ADS1299 daisy;
  
    // VARIABLES
//    long channelData[9];
    char ADSchannelSettings[8][6];
    boolean useAccel;
    int outputType;  // we have a few output types

    // BOARD WIDE FUNCTIONS
    void initialize(void);
    void updateChannelSettings(void);
    void writeChannelSettings(void);
    void setChannelsToDefault(void);
    void setChannelsToEMG(void);
    void setChannelsToECG(void);
    void sendChannelData(byte);
    void startStreaming(void);
    void stopStreaming(void);

    //  ADS1299 FUNCITONS
    void initialize_ads(void);
    void printAllRegisters(void); 
    void writeADSchannelSettings(void); 
    void activateChannel(int);
    void activateChannel(int, byte, byte, boolean);
    void deactivateChannel(int);
    void configureInternalTestSignal(byte,byte);
    void configure_Zdetect(byte,byte);
    void changeChannel_Zdetect(int,int, int);
    void reset_ads(void);
    boolean isDataAvailable(void);
    void updateChannelData(void);
    void setSRB1(boolean);
    byte getADS_ID(void);
    long getChannel(int);
    void putChannel(int, long);
    void update24bitData(void);
    
    //  ACCELEROMETER FUNCIONS
    void initialize_accel(void);
    void disable_accel(void);
    void enable_accel(void);
    byte getAccelID(void);
    void getAccelData(void);
    boolean LIS3DH_DataReady(void);
    int getX(void);
    int getY(void);
    int getZ(void);
    
    
    
};

#endif
