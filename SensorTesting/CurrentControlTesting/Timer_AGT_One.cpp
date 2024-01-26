#include "Timer_AGT_One.h"

TimerAGTOne Timer1;   // preinstantiate.

uint16_t TimerAGTOne::reloadSetting = 0xFFFF;
agt_cntsrc_t TimerAGTOne::srcbits = PCLKB;
uint8_t TimerAGTOne::divbits = 0;
uint8_t TimerAGTOne::eventLinkIndex = 0;
void (*TimerAGTOne::isrCallback)() = TimerAGTOne::isrDefaultUnused;

void TimerAGTOne::internalCallback(){
  // Reset the interrupt link and the flag for the timer
  resetEventLink(eventLinkIndex);
  R_AGT1->AGTCR &= ~(R_AGT0_AGTCR_TUNDF_Msk);
  isrCallback();
}