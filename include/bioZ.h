
void setMode(int mode);
int calcBioZ(uint8_t buf[], uint32_t timestamp_us);
void setMdiv(int val);
void GSRsettings();
void SFBIAsettings();
uint32_t getSampleInterval();
uint32_t getRefClkHz();
extern uint32_t start_time_ms;