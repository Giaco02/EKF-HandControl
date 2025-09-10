#ifndef __HC05LIB_H__
#define __HC05LIB_H__

#include "ch.h"
#include "hal.h"
#include <string.h>
#include "chprintf.h"

typedef struct {
  char * command;
  char * expectedOutput;
} ATCommand;

void HC05GetAddress(BaseSequentialStream *HC05, char * HC05Addr);
const char* HC05ApplyConfig(BaseSequentialStream *HC05, const ATCommand *cmdPtr);
uint8_t HC05IsAlive(BaseSequentialStream *HC05);

#endif /* __HC05LIB_H__ */
