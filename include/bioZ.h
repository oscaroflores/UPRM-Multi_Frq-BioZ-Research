
void setMode(int mode);
int calcBioZ(uint8_t buf[], double freqLogged);
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
