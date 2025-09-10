#include "ch.h"
#include "hal.h"

#include "chprintf.h"
#include "chscanf.h"
#include "hc05/hc05.h"

MUTEX_DECL(serial_mtx);
MUTEX_DECL(ble_mtx);

#define DEBUG                   0

#define PWM_TIMER_FREQUENCY     20000 //20KHz
#define PWM_PERIOD              1000

static volatile uint8_t front       = 0;
static volatile uint8_t vel_front   = 0;
static volatile uint8_t gir         = 0;
static volatile uint8_t vel_gir     = 0;

// -------- PWM -------//
static PWMConfig pwmcfg = {
  PWM_TIMER_FREQUENCY,
  PWM_PERIOD,
  NULL,
  {
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL}
  },
  0,
  0,
  0
};

#define WA_PWM_SIZE 512
THD_WORKING_AREA(waPWM, WA_PWM_SIZE);
THD_FUNCTION( thdPWM, arg ) {
  (void)arg;
  chRegSetThreadName("pwm");

  palSetPadMode(GPIOB, 4, PAL_MODE_ALTERNATE(2)); //D5       T3-Ch1
  palSetPadMode(GPIOC, 7, PAL_MODE_ALTERNATE(2)); //D9       T3-C2

  palSetPadMode(GPIOC, 8, PAL_MODE_ALTERNATE(2)); //PC8      T3-C3
  palSetPadMode(GPIOC, 9, PAL_MODE_ALTERNATE(2)); //PC9      T3-C4

  pwmStart(&PWMD3, &pwmcfg);
  pwmEnableChannel(&PWMD3, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, 0));
  pwmEnableChannel(&PWMD3, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, 0));

  pwmEnableChannel(&PWMD3, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, 0));
  pwmEnableChannel(&PWMD3, 3, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, 0));


  uint16_t base_speed = 0, speed_reduction = 0;
  uint16_t left_speed = 0, right_speed = 0;

  while(true){
    base_speed = vel_front * 700;
    speed_reduction = (vel_gir * base_speed)/15;

    // gir == 1
    if(gir == 0){ // AVANTI RECTO
      pwmEnableChannel(&PWMD3, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, front * base_speed));
      pwmEnableChannel(&PWMD3, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, (1-front) * base_speed));

      pwmEnableChannel(&PWMD3, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, front * base_speed));
      pwmEnableChannel(&PWMD3, 3, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, (1-front) * base_speed));
    }

    else if ( gir == 1){ // DIESTRA
      left_speed = base_speed + (speed_reduction/2);
      right_speed = base_speed - (speed_reduction/2);

      pwmEnableChannel(&PWMD3, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, front * right_speed));
      pwmEnableChannel(&PWMD3, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, (1-front) * right_speed));

      pwmEnableChannel(&PWMD3, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, front * left_speed));
      pwmEnableChannel(&PWMD3, 3, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, (1-front) * left_speed));

    }

    else if (gir == 2){ //SINISTRA
      left_speed = base_speed - (speed_reduction/2);
      right_speed = base_speed + (speed_reduction/2);

      pwmEnableChannel(&PWMD3, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, front * right_speed));
      pwmEnableChannel(&PWMD3, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, (1-front) * right_speed));

      pwmEnableChannel(&PWMD3, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, front * left_speed));
      pwmEnableChannel(&PWMD3, 3, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, (1-front) * left_speed));
    }

    chThdSleepMilliseconds(30);
  }
}

// -------- BLE -------//
BaseSequentialStream *chp = (BaseSequentialStream*)&SD2;
BaseSequentialStream *HC05Slave = (BaseSequentialStream*)&SD1;

int str_to_int(char *str) {
    int result = 0;
    while (*str >= '0' && *str <= '9') {
        result = result * 10 + (*str - '0');
        str++;
    }
    return result;
}


#define WA_SERIAL_SIZE 256
THD_WORKING_AREA(waSerial, WA_SERIAL_SIZE);
THD_FUNCTION( thdSerial, arg ) {
  (void)arg;
  chRegSetThreadName("serial");

  /* Port configuration. */
  palSetPadMode(GPIOC, 4, PAL_MODE_ALTERNATE(7));   //PC4 - TX
  palSetPadMode(GPIOC, 5, PAL_MODE_ALTERNATE(7));   //PC5 - RX

  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));   //TERMINAL PC
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));

  sdStart(&SD1, NULL);
  sdStart(&SD2, NULL);

  chMtxLock(&serial_mtx);
  chprintf(chp,"Waiting 5 seconds for the modules to pair...\r\n");
  chMtxUnlock(&serial_mtx);

  chThdSleepMilliseconds(5000);

  chMtxLock(&serial_mtx);
  chprintf(chp, "Beginning the communication...\r\n");
  chMtxUnlock(&serial_mtx);

  int d1, d2, d3, d4;
  while (true) {
    char buf[16] = {0};

    sdGet((SerialDriver* )HC05Slave);
    size_t n = sdReadTimeout((SerialDriver* )HC05Slave, (uint8_t* ) buf, 15, TIME_MS2I(20));

#if DEBUG
    chMtxLock(&serial_mtx);
    chprintf( chp, "[DBG]: %s\r\n", buf );
    chMtxUnlock(&serial_mtx);
#endif

    if (n >= 4) {
      chMtxLock(&serial_mtx);
      chprintf( chp, "Enter", buf );
      chMtxUnlock(&serial_mtx);

      d1 = buf[0] - '0';
      d2 = buf[1] - '0';
      d3 = buf[2] - '0';
      d4 = buf[3] - '0';

      if(d1 == 1){
        front = 1;
      }
      else if(d1 == 9 || d1 == 0){
        front = 0;
      }

      if(d2 >= 0 && d2 <= 9){
        vel_front = d2;
      }

      if(d3 == 1){
        gir = 1;
      }
      else if(d3 == 9 || d3 == 0){
        gir = 0;
      }
      else if(d3 == 2){
        gir = 2;
      }

      if(d4 >= 0 && d4 <= 9){
        vel_gir = d4;
      }
    }
    chThdSleepMilliseconds(10);
  }
  sdStop(&SD1);
  sdStop(&SD2);
  chThdSleepMilliseconds(500);
}




int main(void) {

  halInit();
  chSysInit();

  chThdCreateStatic(waSerial, sizeof(waSerial), NORMALPRIO, thdSerial,NULL);
  chThdCreateStatic(waPWM, sizeof(waPWM), NORMALPRIO - 1, thdPWM,NULL);



  while (true) {
    palToggleLine(LINE_LED_GREEN);
    chThdSleepMilliseconds(1000);
  }

  pwmDisableChannel(&PWMD3, 0);
  pwmStop(&PWMD3);
}
