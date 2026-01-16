#ifndef INITIALISER_PHASE2
#define INITIALISER_PHASE2

#include "PHASE2_LIBRARIES.h"
#include "PRE_DEFINE.H"

extern gptimer_handle_t gptimer;
extern uint64_t count;

void init_acc_pos(acc_pos *f);
void gpio_configure();


#endif