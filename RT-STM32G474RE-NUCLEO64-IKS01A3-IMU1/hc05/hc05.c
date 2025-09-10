#include "hc05.h"
#include "chprintf.h"

static char responseBuffer[64] = {0};

static uint8_t HC05CheckCommandOutput(BaseSequentialStream *HC05,
                                      const char *cmd, const char *val) {
  memset(responseBuffer, 0, 64);
  if (val) {
  
    chprintf(HC05, "AT+%s=%s\r\n", cmd, val);
    sdReadTimeout((SerialDriver* )HC05, (uint8_t* ) responseBuffer, 63,
                  TIME_MS2I(500));

    chprintf(HC05, "AT+%s?\r\n", cmd);
    sdReadTimeout((SerialDriver* )HC05, (uint8_t* ) responseBuffer, 63,
                  TIME_MS2I(500));

    char *valPtr;
    valPtr = strstr((const char*)responseBuffer, ":");
    if (valPtr == NULL) {
      valPtr = strstr((const char*)responseBuffer, "=");
    }

    if (!strcmp(cmd, "BIND")) {
      *(responseBuffer + 10) = ',';
      *(responseBuffer + 13) = ',';
    }
    if (strncasecmp(valPtr + 1, val, strlen(val))) {
      return 1;
    }
  }

  else {
    chprintf(HC05, "AT+%s\r\n", cmd);
    sdReadTimeout((SerialDriver* )HC05, (uint8_t* ) responseBuffer, 63,
                  TIME_MS2I(500));
  }
  return 0;
}

const char* HC05ApplyConfig(BaseSequentialStream *HC05, const ATCommand *cmdPtr) {
  while (cmdPtr->command) {
    if (HC05CheckCommandOutput(HC05, cmdPtr->command, cmdPtr->expectedOutput)) {
      return cmdPtr->command;
    }
    cmdPtr++;
  }
  return NULL;
}

void HC05GetAddress(BaseSequentialStream *HC05, char *HC05Addr) {
  chprintf(HC05, "AT+ADDR?\r\n");
  sdReadTimeout((SerialDriver* )HC05, (uint8_t* )responseBuffer, 63,
                TIME_MS2I(500));
  strncpy(HC05Addr, responseBuffer + 6, 14);
  HC05Addr[4] = ',';
  HC05Addr[7] = ',';
}

uint8_t HC05IsAlive(BaseSequentialStream *HC05) {
  memset(responseBuffer, 0, 64);
  chprintf(HC05, "AT\r\n");
  sdReadTimeout((SerialDriver* )HC05, (uint8_t* ) responseBuffer, 4,
                TIME_MS2I(500));
  return strncmp(responseBuffer, "OK\r\n", 4) == 0;
}
