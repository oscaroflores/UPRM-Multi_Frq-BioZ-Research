#ifndef BIOZ_H
#define BIOZ_H
#include "spiFunctions.h"
void setMode(int mode);
int calcBioZ(uint8_t buf[], imu_data_t *data);
void setMdiv(int val);
void GSRsettings();
void BIAsettings();
double getSampleInterval();
uint32_t getRefClkHz();
extern uint32_t start_time_ms;
void setFreq(int freq);
int getMdiv(void);
uint32_t getRefClkHz();
int getKDiv();
int getDACOSR();
double getBiozFreq();
#endif