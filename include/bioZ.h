#include <stdint.h>
#include <stdbool.h>
void setMode(int mode);
int calcBioZ(uint8_t buf[], bool freqLogged);
void setMdiv(int val);
void GSRsettings();
void BIAsettings();
uint32_t getSampleInterval();
uint32_t getRefClkHz();
extern uint32_t start_time_ms;
void setFreq(int freq);
int getMdiv(void);
uint32_t getRefClkHz();
int getKDiv();
int getDACOSR();
double getBiozFreq();
double getBiozOhmCoeff(void);
void setBiozOhmCoeff(double c);
double getBiozOhmCoeffCached(void);
