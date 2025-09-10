#include "ch.h"
#include "hal.h"

#include "chprintf.h"
#include "lsm6dso.h"
#include "lsm303agr.h"
#include "hc05/hc05.h"

// AGGIUNTA: EKF
#include "EKF.h"

#define DEBUG       0

MUTEX_DECL(serial_mutex);
static volatile bool sem_kalman = false;

// Parametri controllo PWM
#define DEAD_ZONE_ROLL                      20.0f    // ±10 gradi zona morta roll
#define DEAD_ZONE_PITCH                     30.0f    // ±10 gradi zona morta pitch
#define MAX_ROLL                            50.0f    // ±60 gradi hard cap roll
#define MAX_PITCH                           100.0f    // ±30 gradi hard cap pitch
#define ALPHA_FILTER                        0.5f     // parametro filtro complementare (0.0-1.0)

#define cls(chp)                            chprintf(chp, "\033[2J\033[1;1H")
#define MAX_AXIS_NUMBER                     3U
#define M_PI                                3.14159265f
#define BUTTON_STRAIGHT          PAL_LINE(GPIOA, 5U)     //D13

/* Array for data storage. */
static float cooked[MAX_AXIS_NUMBER];
/* Axis identifiers. */
// static char axis_id[MAX_AXIS_NUMBER] = {'X', 'Y', 'Z'};


static BaseSequentialStream* chp = (BaseSequentialStream*)&SD2;
static BaseSequentialStream *HC05Master = (BaseSequentialStream*)&SD1; //buffer della scheda


// MEMS
static const I2CConfig i2ccfg = {
  STM32_TIMINGR_PRESC(8U)  |            /* 72MHz/9 = 8MHz I2CCLK.           */
  STM32_TIMINGR_SCLDEL(3U) | STM32_TIMINGR_SDADEL(3U) |
  STM32_TIMINGR_SCLH(3U)   | STM32_TIMINGR_SCLL(9U),
  0,
  0
};
/*===========================================================================*/
/* LSM6DSO related.                                                          */
/*===========================================================================*/
/* LSM6DSO Driver: This object represent an LSM6DSO instance */
/* Generic I2C configuration for every MEMS. */

static  LSM6DSODriver LSM6DSOD1;

static const LSM6DSOConfig lsm6dsocfg = {
  .i2cp               = &I2CD1,
  .i2ccfg             = &i2ccfg,
  .slaveaddress       = LSM6DSO_SAD_VCC,
  .accsensitivity     = NULL,
  .accbias            = NULL,
  .accfullscale       = LSM6DSO_ACC_FS_2G,
  .accoutdatarate     = LSM6DSO_ACC_ODR_104Hz,
  .gyrosensitivity    = NULL,
  .gyrobias           = NULL,
  .gyrofullscale      = LSM6DSO_GYRO_FS_500DPS,
  .gyrooutdatarate    = LSM6DSO_GYRO_ODR_104Hz,
};


static LSM303AGRDriver LSM303AGRD1;

static const LSM303AGRConfig lsm303agrcfg = {
  .i2cp                 = &I2CD1,
  .i2ccfg               = &i2ccfg,
  .accsensitivity       = NULL,
  .accbias              = NULL,
  .accfullscale         = LSM303AGR_ACC_FS_4G,
  .accodr               = LSM303AGR_ACC_ODR_100Hz,
  .compsensitivity      = NULL,
  .compbias             = NULL,
  .compodr              = LSM303AGR_COMP_ODR_100HZ,
};
/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/
typedef struct {
    float roll;
    float pitch;
}angles_degree;

static volatile char control_data[4] = {0, 0, 0, 0};

ekf_t ekf;


// Variabili globali per il filtro complementare
static float filtered_roll = 0.0f;
static float filtered_pitch = 0.0f;
static bool first_sample = true;

int get_sign(float value) {
    return (value >= 0.0f) ? 1 : 0;
}

int get_sign_pitch(float value) {
    if (value == 0.0f) {
        return 0;
    } else if (value < 0.0f) {
        return 1;
    } else {
        return 2;
    }
}


// Funzione per applicare zona morta
float apply_deadzone(float angle, float deadzone) {
    if (angle > deadzone) {
        return angle - deadzone;
    } else if (angle < -deadzone) {
        return angle + deadzone;
    } else {
        return 0.0f;
    }
}

// Funzione per applicare hard cap
float apply_hardcap(float angle, float max_angle) {
    if (angle > max_angle) {
        return max_angle;
    } else if (angle < -max_angle) {
        return -max_angle;
    } else {
        return angle;
    }
}

// Funzione per scalare da -max_angle a +max_angle verso 0-10
int scale_to_pwm(float angle, float max_angle) {
    // Usa solo l'intensità (valore assoluto)
    float abs_angle = (angle < 0) ? -angle : angle;

    // Normalizza da 0 : max_angle → 0 : 1
    float normalized = abs_angle / max_angle;
    if (normalized > 1.0f) normalized = 1.0f; // non oltre 1.0

    // Scala da 0:1 → 1:10
    float scaled = normalized * 9.0f;

    // Arrotonda
    int pwm_value = (int)(scaled + 0.5f);

    // Limita a [1,10]
    if (pwm_value < 1) pwm_value = 0;
    if (pwm_value > 9) pwm_value = 9;

    return pwm_value;
}


#define WA_SERIAL_SIZE 256
THD_WORKING_AREA(waSerial, WA_SERIAL_SIZE);
THD_FUNCTION(thdSerial, arg) {
  (void)arg;
  chRegSetThreadName("serial");

  /* Port configuration. */
  palSetPadMode(GPIOC, 4, PAL_MODE_ALTERNATE(7));   //PC4 - TX
  palSetPadMode(GPIOC, 5, PAL_MODE_ALTERNATE(7));   //PC5 - RX
  palSetLineMode(BUTTON_STRAIGHT, PAL_MODE_INPUT_PULLDOWN); //button



  /* Starting Serial Drivers #1, #2 and #3*/
  sdStart(&SD1, NULL);

  chprintf(
      chp,
      "Waiting 5 seconds for the modules to pair...\r\n");
  chThdSleepMilliseconds(5000);
  chprintf(chp, "Beginning module data communication...\r\n");
  while (true) {
      // Copia locale per sicurezza (evita che venga modificato mentre lo stampi)
      char local_data[4];
      local_data[0] = control_data[0];
      local_data[1] = control_data[1];
      local_data[2] = control_data[2];
      local_data[3] = control_data[3];

      // Invia su Bluetooth (HC05Master)
      if (palReadLine(BUTTON_STRAIGHT) == PAL_HIGH ) {
        chprintf(HC05Master, "a%d900\r",local_data[0]);
      }
      else{
        chprintf(HC05Master, "a%d%d%d%d\r",
               local_data[0], local_data[1], local_data[2], local_data[3]); //TODO ARNAU CHANGED THAT maybe fails; Was perfect

      }

#if DEBUG
      // Debug su PC
      chMtxLock(&serial_mutex);
      chprintf(chp, "TX -> RollSign:%d RollPWM:%d PitchSign:%d PitchPWM:%d\r\n",
               local_data[0], local_data[1], local_data[2], local_data[3]);
      chMtxUnlock(&serial_mutex);
#endif

      chThdSleepMilliseconds(30);
  }
  sdStop(&SD1);
  sdStop(&SD2);
  chThdSleepMilliseconds(500);
}

static THD_WORKING_AREA(waConversion, 512);
static THD_FUNCTION(thdConversion, arg) {
  angles_degree* input = (angles_degree*)arg;

  chRegSetThreadName("angle_conversion");

  while(true){
    float current_roll = input->roll;
    float current_pitch = input->pitch;
    // Copia gli angoli localmente per evitare problemi di concorrenza
    // 1. Applica zona morta
    float roll_deadzone = apply_deadzone(current_roll, DEAD_ZONE_ROLL);
    float pitch_deadzone = apply_deadzone(current_pitch, DEAD_ZONE_PITCH);

    // Salva i segni degli angoli ORIGINALI (prima di qualsiasi elaborazione)
    int original_sign_roll = get_sign(roll_deadzone);
    int original_sign_pitch = get_sign_pitch(pitch_deadzone);

    // 2. Applica hard cap
    float roll_capped = apply_hardcap(roll_deadzone, MAX_ROLL);
    float pitch_capped = apply_hardcap(pitch_deadzone, MAX_PITCH);

    // 3. Filtro complementare (mix tra valore precedente e nuovo)
    if (first_sample) {
        filtered_roll = roll_capped;
        filtered_pitch = pitch_capped;
        first_sample = false;
    } else {
        // ALPHA_FILTER * valore_precedente + (1 - ALPHA_FILTER) * valore_nuovo
        filtered_roll = ALPHA_FILTER * filtered_roll + (1.0f - ALPHA_FILTER) * roll_capped;
        filtered_pitch = ALPHA_FILTER * filtered_pitch + (1.0f - ALPHA_FILTER) * pitch_capped;
    }

    // 4. Scala a valori PWM 1-10
    int pwm_roll = scale_to_pwm(filtered_roll, MAX_ROLL);
    int pwm_pitch = scale_to_pwm(filtered_pitch, MAX_PITCH);


    // 5. Salva tutti i dati nell'array char[4]
    control_data[0] = (char)original_sign_roll;   // segno roll
    control_data[1] = (char)pwm_roll;             // valore roll (0-9)
    control_data[2] = (char)original_sign_pitch;  // segno pitch
    control_data[3] = (char)pwm_pitch;            // valore pitch (0-9)

     chThdSleepMilliseconds(30);
  }
}


static void button_cb(void *arg) {
  (void)arg;

  chSysLockFromISR();

  palToggleLine(LINE_LED_GREEN);
  sem_kalman = true;

  chSysUnlockFromISR();
}


/*
 * Application entry point.
 */
int main(void) {

  halInit();
  chSysInit();


  palSetLineMode(PAL_LINE(GPIOB, 9U), PAL_MODE_ALTERNATE(4) |
                    PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_OSPEED_HIGHEST |
                    PAL_STM32_PUPDR_PULLUP);
  palSetLineMode(PAL_LINE(GPIOB, 8U), PAL_MODE_ALTERNATE(4) |
                    PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_OSPEED_HIGHEST |
                    PAL_STM32_PUPDR_PULLUP);

  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));   //TERMINAL PC
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));


  sdStart(&SD2,NULL);

  /* MEMS Driver Objects Initialization.*/
  lsm6dsoObjectInit(&LSM6DSOD1);
  lsm303agrObjectInit(&LSM303AGRD1);

  /* Activates all the MEMS related drivers.*/
  lsm6dsoStart(&LSM6DSOD1, &lsm6dsocfg);
  lsm303agrStart(&LSM303AGRD1, &lsm303agrcfg);

  float euler_ekf[3];

  angles_degree angles;


  angles.pitch = 0.0;
  angles.roll = 0.0;

  // Read compas
  lsm303agrCompassReadCooked(&LSM303AGRD1, cooked);
  //  chMtxLock(&angle_mutex);
  EKF_init(&ekf, cooked[0], cooked[1], cooked[2], 0.1f, 1.0f, 1.0f);
  //  chMtxUnlock(&angle_mutex);


  chThdCreateStatic( waConversion, sizeof(waConversion), NORMALPRIO, thdConversion, &angles);
  chThdCreateStatic(waSerial, sizeof(waSerial), NORMALPRIO - 1, thdSerial,
                    NULL);

  palEnableLineEvent( LINE_BUTTON, PAL_EVENT_MODE_RISING_EDGE);
  palSetLineCallback( LINE_BUTTON, button_cb, (void *) &angles);

    while (true) {

        // Leggi accelerometro (m/s²)
        lsm6dsoAccelerometerReadCooked(&LSM6DSOD1, cooked);
        float ax = cooked[0];
        float ay = cooked[1];
        float az = cooked[2];

        // Leggi giroscopio (dps → rad/s)
        lsm6dsoGyroscopeReadCooked(&LSM6DSOD1, cooked);
        float p = cooked[0] * (M_PI / 180.0f);
        float q = cooked[1] * (M_PI / 180.0f);
        float r = cooked[2] * (M_PI / 180.0f);

        // Leggi magnetometro (unità grezze coerenti con ref iniziale)
        lsm303agrCompassReadCooked(&LSM303AGRD1, cooked);
        float mx = cooked[0];
        float my = cooked[1];
        float mz = cooked[2];

        // Aggiorna EKF
//        chMtxLock(&angle_mutex);
        if(sem_kalman){
          chThdSleepMilliseconds(500);
          lsm303agrCompassReadCooked(&LSM303AGRD1, cooked);
          EKF_init(&ekf, cooked[0], cooked[1], cooked[2], 0.1f, 1.0f, 1.0f);

          sem_kalman = false;

        }
        else{
          EKF_update(&ekf, euler_ekf, ax, ay, az, p, q, r, mx, my, mz, 0.01f);
        }
        float rpy[2];  // rpy[0]=roll, rpy[1]=pitch
        EKFquaternionToTilt(ekf.x, rpy);
//        chMtxUnlock(&angle_mutex);

        float roll_deg  = rpy[0] ;
        float pitch_deg = rpy[1] ;
        angles.roll = roll_deg;
        angles.pitch = pitch_deg;

        chThdSleepMilliseconds(10); // ~100 Hz
    }
}
