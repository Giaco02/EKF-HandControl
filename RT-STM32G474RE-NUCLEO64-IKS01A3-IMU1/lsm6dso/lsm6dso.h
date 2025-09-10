/*
    ChibiOS - Copyright (C) 2016..2024 Giovanni Francesco Comune

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

/**
 * @file    lsm6dso.h
 * @brief   LSM6DSO MEMS interface module header.
 *
 * @addtogroup LSM6DSO
 * @ingroup EX_ST
 * @{
 */
#ifndef _LSM6DSO_H_
#define _LSM6DSO_H_

#define DRV_LITTLE_ENDIAN 1234
#define DRV_BYTE_ORDER    DRV_LITTLE_ENDIAN

#include "ex_accelerometer.h"
#include "ex_gyroscope.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Version identification
 * @{
 */
/**
 * @brief   LSM6DSO driver version string.
 */
#define EX_LSM6DSO_VERSION                  "1.0.1"

/**
 * @brief   LSM6DSO driver version major number.
 */
#define EX_LSM6DSO_MAJOR                    1

/**
 * @brief   LSM6DSO driver version minor number.
 */
#define EX_LSM6DSO_MINOR                    0

/**
 * @brief   LSM6DSO driver version patch number.
 */
#define EX_LSM6DSO_PATCH                    1
/** @} */

/**
 * @brief   LSM6DSO accelerometer subsystem characteristics.
 * @note    Sensitivity is expressed as milli-G/LSB whereas
 *          1 milli-G = 0.00980665 m/s^2.
 * @note    Bias is expressed as milli-G.
 *
 * @{
 */
#define LSM6DSO_ACC_NUMBER_OF_AXES          3U

#define LSM6DSO_ACC_2G                      2.0f
#define LSM6DSO_ACC_4G                      4.0f
#define LSM6DSO_ACC_8G                      8.0f
#define LSM6DSO_ACC_16G                     16.0f

#define LSM6DSO_ACC_SENS_2G                 0.061f
#define LSM6DSO_ACC_SENS_4G                 0.122f
#define LSM6DSO_ACC_SENS_8G                 0.244f
#define LSM6DSO_ACC_SENS_16G                0.488f

#define LSM6DSO_ACC_BIAS                    0.0f
/** @} */

/**
 * @brief   L3GD20 gyroscope system characteristics.
 * @note    Sensitivity is expressed as DPS/LSB whereas DPS stand for Degree
 *          per second [�/s].
 * @note    Bias is expressed as DPS.
 *
 * @{
 */
#define LSM6DSO_GYRO_NUMBER_OF_AXES         3U

#define LSM6DSO_GYRO_125DPS                 125.0f
#define LSM6DSO_GYRO_250DPS                 250.0f
#define LSM6DSO_GYRO_500DPS                 500.0f
#define LSM6DSO_GYRO_1000DPS                1000.0f
#define LSM6DSO_GYRO_2000DPS                2000.0f

#define LSM6DSO_GYRO_SENS_125DPS            0.004375f
#define LSM6DSO_GYRO_SENS_250DPS            0.008750f
#define LSM6DSO_GYRO_SENS_500DPS            0.017500f
#define LSM6DSO_GYRO_SENS_1000DPS           0.035000f
#define LSM6DSO_GYRO_SENS_2000DPS           0.070000f

#define LSM6DSO_GYRO_BIAS                   0.0f
/** @} */

/**
 * @name   LSM6DSO communication interfaces related bit masks
 * @{
 */
#define LSM6DSO_DI_MASK                     0xFF
#define LSM6DSO_DI(n)                       (1 << n)
#define LSM6DSO_AD_MASK                     0x7F
#define LSM6DSO_AD(n)                       (1 << n)
#define LSM6DSO_MS                          (1 << 7)
/** @} */

/**
 * @name   LSM6DSO register addresses
 * @{
 */
#define LSM6DSO_AD_FUNC_CFG_ACCESS          0x01
#define LSM6DSO_AD_SENSOR_SYNC_TIME_FRAME   0x04
#define LSM6DSO_AD_SENSOR_SYNC_RES_RATIO    0x05
#define LSM6DSO_AD_FIFO_CTRL1               0x06
#define LSM6DSO_AD_FIFO_CTRL2               0x07
#define LSM6DSO_AD_FIFO_CTRL3               0x08
#define LSM6DSO_AD_FIFO_CTRL4               0x09
#define LSM6DSO_AD_FIFO_CTRL5               0x0A
#define LSM6DSO_AD_DRDY_PULSE_CFG_G         0x0B
#define LSM6DSO_AD_INT1_CTRL                0x0D
#define LSM6DSO_AD_INT2_CTRL                0x0E
#define LSM6DSO_AD_WHO_AM_I                 0x0F
#define LSM6DSO_AD_CTRL1_XL                 0x10
#define LSM6DSO_AD_CTRL2_G                  0x11
#define LSM6DSO_AD_CTRL3_C                  0x12
#define LSM6DSO_AD_CTRL4_C                  0x13
#define LSM6DSO_AD_CTRL5_C                  0x14
#define LSM6DSO_AD_CTRL6_C                  0x15
#define LSM6DSO_AD_CTRL7_G                  0x16
#define LSM6DSO_AD_CTRL8_XL                 0x17
#define LSM6DSO_AD_CTRL9_XL                 0x18
#define LSM6DSO_AD_CTRL10_C                 0x19
#define LSM6DSO_AD_MASTER_CONFIG            0x1A
#define LSM6DSO_AD_WAKE_UP_SRC              0x1B
#define LSM6DSO_AD_TAP_SRC                  0x1C
#define LSM6DSO_AD_D6D_SRC                  0x1D
#define LSM6DSO_AD_STATUS_REG               0x1E
#define LSM6DSO_AD_OUT_TEMP_L               0x20
#define LSM6DSO_AD_OUT_TEMP_H               0x21
#define LSM6DSO_AD_OUTX_L_G                 0x22
#define LSM6DSO_AD_OUTX_H_G                 0x23
#define LSM6DSO_AD_OUTY_L_G                 0x24
#define LSM6DSO_AD_OUTY_H_G                 0x25
#define LSM6DSO_AD_OUTZ_L_G                 0x26
#define LSM6DSO_AD_OUTZ_H_G                 0x27
#define LSM6DSO_AD_OUTX_L_XL                0x28
#define LSM6DSO_AD_OUTX_H_XL                0x29
#define LSM6DSO_AD_OUTY_L_XL                0x2A
#define LSM6DSO_AD_OUTY_H_XL                0x2B
#define LSM6DSO_AD_OUTZ_L_XL                0x2C
#define LSM6DSO_AD_OUTZ_H_XL                0x2D
#define LSM6DSO_AD_SENSORHUB1_REG           0x2E
#define LSM6DSO_AD_SENSORHUB2_REG           0x2F
#define LSM6DSO_AD_SENSORHUB3_REG           0x30
#define LSM6DSO_AD_SENSORHUB4_REG           0x31
#define LSM6DSO_AD_SENSORHUB5_REG           0x32
#define LSM6DSO_AD_SENSORHUB6_REG           0x33
#define LSM6DSO_AD_SENSORHUB7_REG           0x34
#define LSM6DSO_AD_SENSORHUB8_REG           0x35
#define LSM6DSO_AD_SENSORHUB9_REG           0x36
#define LSM6DSO_AD_SENSORHUB10_REG          0x37
#define LSM6DSO_AD_SENSORHUB11_REG          0x38
#define LSM6DSO_AD_SENSORHUB12_REG          0x39
#define LSM6DSO_AD_FIFO_STATUS1             0x3A
#define LSM6DSO_AD_FIFO_STATUS2             0x3B
#define LSM6DSO_AD_FIFO_STATUS3             0x3C
#define LSM6DSO_AD_FIFO_STATUS4             0x3D
#define LSM6DSO_AD_FIFO_DATA_OUT_L          0x3E
#define LSM6DSO_AD_FIFO_DATA_OUT_H          0x3F
#define LSM6DSO_AD_TIMESTAMP0_REG           0x40
#define LSM6DSO_AD_TIMESTAMP1_REG           0x41
#define LSM6DSO_AD_TIMESTAMP2_REG           0x42
#define LSM6DSO_AD_STEP_TIMESTAMP_L         0x49
#define LSM6DSO_AD_STEP_TIMESTAMP_H         0x4A
#define LSM6DSO_AD_STEP_COUNTER_L           0x4B
#define LSM6DSO_AD_STEP_COUNTER_H           0x4C
#define LSM6DSO_AD_SENSORHUB13_REG          0x4D
#define LSM6DSO_AD_SENSORHUB14_REG          0x4E
#define LSM6DSO_AD_SENSORHUB15_REG          0x4F
#define LSM6DSO_AD_SENSORHUB16_REG          0x50
#define LSM6DSO_AD_SENSORHUB17_REG          0x51
#define LSM6DSO_AD_SENSORHUB18_REG          0x52
#define LSM6DSO_AD_FUNC_SRC1                0x53
#define LSM6DSO_AD_FUNC_SRC2                0x54
#define LSM6DSO_AD_WRIST_TILT_IA            0x55
#define LSM6DSO_TAP_CFG0                    0x56
#define LSM6DSO_AD_TAP_CFG                  0x58
#define LSM6DSO_AD_TAP_THS_6D               0x59
#define LSM6DSO_AD_INT_DUR2                 0x5A
#define LSM6DSO_AD_WAKE_UP_THS              0x5B
#define LSM6DSO_AD_WAKE_UP_DUR              0x5C
#define LSM6DSO_AD_FREE_FALL                0x5D
#define LSM6DSO_AD_MD1_CFG                  0x5E
#define LSM6DSO_AD_MD2_CFG                  0x5F
#define LSM6DSO_AD_MASTER_CMD_CODE          0x60
#define LSM6DSO_AD_SENS_SYNC_SPI_ERROR_CODE 0x61
#define LSM6DSO_I3C_BUS_AVB                 0x62
#define LSM6DSO_EMB_FUNC_SRC                0x64
#define LSM6DSO_AD_OUT_MAG_RAW_X_L          0x66
#define LSM6DSO_AD_OUT_MAG_RAW_X_H          0x67
#define LSM6DSO_AD_OUT_MAG_RAW_Y_L          0x68
#define LSM6DSO_AD_OUT_MAG_RAW_Y_H          0x69
#define LSM6DSO_AD_OUT_MAG_RAW_Z_L          0x6A
#define LSM6DSO_AD_OUT_MAG_RAW_Z_H          0x6B
#define LSM6DSO_AD_X_OFS_USR                0x73
#define LSM6DSO_AD_Y_OFS_USR                0x74
#define LSM6DSO_AD_Z_OFS_USR                0x75
#define LSM6DSO_PEDO_CMD_REG                0x183
#define LSM6DSO_EMB_FUNC_EN_A               0x04
#define LSM6DSO_EMB_FUNC_EN_B               0x05
#define LSM6DSO_STEP_COUNTER_L              0x62
#define LSM6DSO_PAGE_RW                     0x17
#define LSM6DSO_PAGE_SEL                    0x02
#define LSM6DSO_PAGE_ADDRESS                0x08
#define LSM6DSO_PAGE_VALUE                  0x09

/** @} */

/**
 * @name    LSM6DSO_AD_CTRL1_XL register bits definitions
 * @{
 */
#define LSMDSL_CTRL1_XL_BW0_XL              (1 << 0)
#define LSMDSL_CTRL1_XL_LPF1_BW_SEL         (1 << 1)
#define LSMDSL_CTRL1_XL_FS_MASK             0x0C
#define LSMDSL_CTRL1_XL_FS_XL0              (1 << 2)
#define LSMDSL_CTRL1_XL_FS_XL1              (1 << 3)
#define LSMDSL_CTRL1_XL_ODR_XL0             (1 << 4)
#define LSMDSL_CTRL1_XL_ODR_XL1             (1 << 5)
#define LSMDSL_CTRL1_XL_ODR_XL2             (1 << 6)
#define LSMDSL_CTRL1_XL_ODR_XL3             (1 << 7)
/** @} */

/**
 * @name    LSM6DSO_AD_CTRL2_G register bits definitions
 * @{
 */
#define LSMDSL_CTRL2_G_FS_MASK              0x0E
#define LSMDSL_CTRL2_G_FS_125               (1 << 1)
#define LSMDSL_CTRL2_G_FS_G0                (1 << 2)
#define LSMDSL_CTRL2_G_FS_G1                (1 << 3)
#define LSMDSL_CTRL2_G_ODR_G0               (1 << 4)
#define LSMDSL_CTRL2_G_ODR_G1               (1 << 5)
#define LSMDSL_CTRL2_G_ODR_G2               (1 << 6)
#define LSMDSL_CTRL2_G_ODR_G3               (1 << 7)
/** @} */

/**
 * @name    LSM6DSO_AD_CTRL3_C register bits definitions
 * @{
 */
#define LSMDSL_CTRL3_C_SW_RESET             (1 << 0)
#define LSMDSL_CTRL3_C_BLE                  (1 << 1)
#define LSMDSL_CTRL3_C_IF_INC               (1 << 2)
#define LSMDSL_CTRL3_C_SIM                  (1 << 3)
#define LSMDSL_CTRL3_C_PP_OD                (1 << 4)
#define LSMDSL_CTRL3_C_H_LACTIVE            (1 << 5)
#define LSMDSL_CTRL3_C_BDU                  (1 << 6)
#define LSMDSL_CTRL3_C_BOOT                 (1 << 7)
/** @} */

/**
 * @name    LSM6DSO_AD_CTRL4_C register bits definitions
 * @{
 */
#define LSMDSL_CTRL4_C_NOT_USED_01          (1 << 0)
#define LSMDSL_CTRL4_C_LPF1_SEL_G           (1 << 1)
#define LSMDSL_CTRL4_C_I2C_DISABLE          (1 << 2)
#define LSMDSL_CTRL4_C_DRDY_MASK            (1 << 3)
#define LSMDSL_CTRL4_C_DEN_DRDY_IN          (1 << 4)
#define LSMDSL_CTRL4_C_INT2_ON_INT          (1 << 5)
#define LSMDSL_CTRL4_C_SLEEP                (1 << 6)
#define LSMDSL_CTRL4_C_DEN_XL_EN            (1 << 7)
/** @} */

/**
 * @name    LSM6DSO_AD_CTRL5_C register bits definitions
 * @{
 */
#define LSMDSL_CTRL5_C_ST0_XL               (1 << 0)
#define LSMDSL_CTRL5_C_ST1_XL               (1 << 1)
#define LSMDSL_CTRL5_C_ST0_G                (1 << 2)
#define LSMDSL_CTRL5_C_ST1_G                (1 << 3)
#define LSMDSL_CTRL5_C_DEN_LH               (1 << 4)
#define LSMDSL_CTRL5_C_ROUNDING0            (1 << 5)
#define LSMDSL_CTRL5_C_ROUNDING1            (1 << 6)
#define LSMDSL_CTRL5_C_ROUNDING2            (1 << 7)
/** @} */

/**
 * @name    LSM6DSO_AD_CTRL6_C register bits definitions
 * @{
 */
#define LSMDSL_CTRL6_C_FTYPE_0              (1 << 0)
#define LSMDSL_CTRL6_C_FTYPE_1              (1 << 1)
#define LSMDSL_CTRL6_C_USR_OFF_W            (1 << 3)
#define LSMDSL_CTRL6_C_XL_HM_MODE           (1 << 4)
#define LSMDSL_CTRL6_C_LVL2_EN              (1 << 5)
#define LSMDSL_CTRL6_C_LVL_EN               (1 << 6)
#define LSMDSL_CTRL6_C_TRIG_EN              (1 << 7)
/** @} */

/**
 * @name    LSM6DSO_AD_CTRL7_G register bits definitions
 * @{
 */
#define LSMDSL_CTRL7_G_ROUNDING_ST          (1 << 2)
#define LSMDSL_CTRL7_G_HPM0_G               (1 << 4)
#define LSMDSL_CTRL7_G_HPM1_G               (1 << 5)
#define LSMDSL_CTRL7_G_HP_EN_G              (1 << 6)
#define LSMDSL_CTRL7_G_G_HM_MODE            (1 << 7)
/** @} */

/**
 * @name    LSM6DSO_AD_CTRL8_XL register bits definitions
 * @{
 */
#define LSMDSL_CTRL8_XL_LOW_PASS_ON         (1 << 0)
#define LSMDSL_CTRL8_XL_HP_SLOPE_XL         (1 << 2)
#define LSMDSL_CTRL8_XL_INPUT_COMPO         (1 << 3)
#define LSMDSL_CTRL8_XL_HP_REF_MODE         (1 << 4)
#define LSMDSL_CTRL8_XL_HPCF_XL0            (1 << 5)
#define LSMDSL_CTRL8_XL_HPCF_XL1            (1 << 6)
#define LSMDSL_CTRL8_XL_LPF2_XL_EN          (1 << 7)
/** @} */

/**
 * @name    LSM6DSO_AD_CTRL9_XL register bits definitions
 * @{
 */
#define LSMDSL_CTRL9_XL_SOFT_EN             (1 << 2)
#define LSMDSL_CTRL9_XL_DEN_XL_G            (1 << 4)
#define LSMDSL_CTRL9_XL_DEN_Z               (1 << 5)
#define LSMDSL_CTRL9_XL_DEN_Y               (1 << 6)
#define LSMDSL_CTRL9_XL_DEN_X               (1 << 7)
/** @} */

/**
 * @name    LSM6DSO_AD_CTRL10_C register bits definitions
 * @{
 */
#define LSMDSL_CTRL10_C_SIGN_MOTION         (1 << 0)
#define LSMDSL_CTRL10_C_PEDO_RST_ST         (1 << 1)
#define LSMDSL_CTRL10_C_FUNC_EN             (1 << 2)
#define LSMDSL_CTRL10_C_TILT_EN             (1 << 3)
#define LSMDSL_CTRL10_C_PEDO_EN             (1 << 4)
#define LSMDSL_CTRL10_C_TIMER_EN            (1 << 5)
#define LSMDSL_CTRL10_C_WRIST_TILT          (1 << 7)
/** @} */

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sw_reset                 : 1;
  uint8_t not_used_01              : 1;
  uint8_t if_inc                   : 1;
  uint8_t sim                      : 1;
  uint8_t pp_od                    : 1;
  uint8_t h_lactive                : 1;
  uint8_t bdu                      : 1;
  uint8_t boot                     : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t boot                     : 1;
  uint8_t bdu                      : 1;
  uint8_t h_lactive                : 1;
  uint8_t pp_od                    : 1;
  uint8_t sim                      : 1;
  uint8_t if_inc                   : 1;
  uint8_t not_used_01              : 1;
  uint8_t sw_reset                 : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dso_ctrl3_c_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t pd_dis_int1              : 1;
  uint8_t not_used_01              : 2;
  uint8_t i3c_bus_avb_sel          : 2;
  uint8_t not_used_02              : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 3;
  uint8_t i3c_bus_avb_sel          : 2;
  uint8_t not_used_01              : 2;
  uint8_t pd_dis_int1              : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dso_i3c_bus_avb_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t i3c_disable              : 1;
  uint8_t den_lh                   : 1;
  uint8_t den_xl_g                 : 2;   /* den_xl_en + den_xl_g */
  uint8_t den_z                    : 1;
  uint8_t den_y                    : 1;
  uint8_t den_x                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t den_x                    : 1;
  uint8_t den_y                    : 1;
  uint8_t den_z                    : 1;
  uint8_t den_xl_g                 : 2;   /* den_xl_en + den_xl_g */
  uint8_t den_lh                   : 1;
  uint8_t i3c_disable              : 1;
  uint8_t not_used_01              : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dso_ctrl9_xl_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t lpf2_xl_en               : 1;
  uint8_t fs_xl                    : 2;
  uint8_t odr_xl                   : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t odr_xl                   : 4;
  uint8_t fs_xl                    : 2;
  uint8_t lpf2_xl_en               : 1;
  uint8_t not_used_01              : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dso_ctrl1_xl_t;

#define LSM6DSO_CTRL2_G                      0x11U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t fs_g                     : 3; /* fs_125 + fs_g */
  uint8_t odr_g                    : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t odr_g                    : 4;
  uint8_t fs_g                     : 3; /* fs_125 + fs_g */
  uint8_t not_used_01              : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dso_ctrl2_g_t;

typedef enum
{
  LSM6DSO_FF_TSH_156mg = 0,
  LSM6DSO_FF_TSH_219mg = 1,
  LSM6DSO_FF_TSH_250mg = 2,
  LSM6DSO_FF_TSH_312mg = 3,
  LSM6DSO_FF_TSH_344mg = 4,
  LSM6DSO_FF_TSH_406mg = 5,
  LSM6DSO_FF_TSH_469mg = 6,
  LSM6DSO_FF_TSH_500mg = 7,
} lsm6dso_ff_ths_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t low_pass_on_6d           : 1;
  uint8_t xl_fs_mode               : 1;
  uint8_t hp_slope_xl_en           : 1;
  uint8_t fastsettl_mode_xl        : 1;
  uint8_t hp_ref_mode_xl           : 1;
  uint8_t hpcf_xl                  : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t hpcf_xl                  : 3;
  uint8_t hp_ref_mode_xl           : 1;
  uint8_t fastsettl_mode_xl        : 1;
  uint8_t hp_slope_xl_en           : 1;
  uint8_t xl_fs_mode               : 1;
  uint8_t low_pass_on_6d           : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dso_ctrl8_xl_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sleep_dur                : 4;
  uint8_t wake_ths_w               : 1;
  uint8_t wake_dur                 : 2;
  uint8_t ff_dur                   : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ff_dur                   : 1;
  uint8_t wake_dur                 : 2;
  uint8_t wake_ths_w               : 1;
  uint8_t sleep_dur                : 4;
#endif /* DRV_BYTE_ORDER */
} lsm6dso_wake_up_dur_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ff_ths                   : 3;
  uint8_t ff_dur                   : 5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ff_dur                   : 5;
  uint8_t ff_ths                   : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dso_free_fall_t;

typedef enum
{
  LSM6DSO_ALL_INT_PULSED            = 0,
  LSM6DSO_BASE_LATCHED_EMB_PULSED   = 1,
  LSM6DSO_BASE_PULSED_EMB_LATCHED   = 2,
  LSM6DSO_ALL_INT_LATCHED           = 3,
} lsm6dso_lir_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 5;
  uint8_t page_rw                  : 2;  /* page_write + page_read */
  uint8_t emb_func_lir             : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t emb_func_lir             : 1;
  uint8_t page_rw                  : 2;  /* page_write + page_read */
  uint8_t not_used_01              : 5;
#endif /* DRV_BYTE_ORDER */
} lsm6dso_page_rw_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 4;
  uint8_t page_sel                 : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t page_sel                 : 4;
  uint8_t not_used_01              : 4;
#endif /* DRV_BYTE_ORDER */
} lsm6dso_page_sel_t;

typedef struct
{
  uint8_t page_addr                : 8;
} lsm6dso_page_address_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 6;
uint8_t reg_access               :
  2; /* shub_reg_access + func_cfg_access */
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
uint8_t reg_access               :
  2; /* shub_reg_access + func_cfg_access */
  uint8_t not_used_01              : 6;
#endif /* DRV_BYTE_ORDER */
} lsm6dso_func_cfg_access_t;

typedef enum
{
  LSM6DSO_USER_BANK           = 0,
  LSM6DSO_SENSOR_HUB_BANK     = 1,
  LSM6DSO_EMBEDDED_FUNC_BANK  = 2,
} lsm6dso_reg_access_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t lir                      : 1;
  uint8_t tap_z_en                 : 1;
  uint8_t tap_y_en                 : 1;
  uint8_t tap_x_en                 : 1;
  uint8_t slope_fds                : 1;
  uint8_t sleep_status_on_int      : 1;
  uint8_t int_clr_on_read          : 1;
  uint8_t not_used_01              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t int_clr_on_read          : 1;
  uint8_t sleep_status_on_int      : 1;
  uint8_t slope_fds                : 1;
  uint8_t tap_x_en                 : 1;
  uint8_t tap_y_en                 : 1;
  uint8_t tap_z_en                 : 1;
  uint8_t lir                      : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dso_tap_cfg0_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01             : 3;
  uint8_t is_step_det             : 1;
  uint8_t is_tilt                 : 1;
  uint8_t is_sigmot               : 1;
  uint8_t not_used_02             : 1;
  uint8_t is_fsm_lc               : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t is_fsm_lc               : 1;
  uint8_t not_used_02             : 1;
  uint8_t is_sigmot               : 1;
  uint8_t is_tilt                 : 1;
  uint8_t is_step_det             : 1;
  uint8_t not_used_01             : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dso_emb_func_status_mainpage_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sens_hub_endop          : 1;
  uint8_t not_used_01             : 2;
  uint8_t slave0_nack             : 1;
  uint8_t slave1_nack             : 1;
  uint8_t slave2_nack             : 1;
  uint8_t slave3_nack             : 1;
  uint8_t wr_once_done            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t wr_once_done            : 1;
  uint8_t slave3_nack             : 1;
  uint8_t slave2_nack             : 1;
  uint8_t slave1_nack             : 1;
  uint8_t slave0_nack             : 1;
  uint8_t not_used_01             : 2;
  uint8_t sens_hub_endop          : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dso_status_master_mainpage_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t is_fsm1                 : 1;
  uint8_t is_fsm2                 : 1;
  uint8_t is_fsm3                 : 1;
  uint8_t is_fsm4                 : 1;
  uint8_t is_fsm5                 : 1;
  uint8_t is_fsm6                 : 1;
  uint8_t is_fsm7                 : 1;
  uint8_t is_fsm8                 : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t is_fsm8                 : 1;
  uint8_t is_fsm7                 : 1;
  uint8_t is_fsm6                 : 1;
  uint8_t is_fsm5                 : 1;
  uint8_t is_fsm4                 : 1;
  uint8_t is_fsm3                 : 1;
  uint8_t is_fsm2                 : 1;
  uint8_t is_fsm1                 : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dso_fsm_status_a_mainpage_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t is_fsm9                 : 1;
  uint8_t is_fsm10                : 1;
  uint8_t is_fsm11                : 1;
  uint8_t is_fsm12                : 1;
  uint8_t is_fsm13                : 1;
  uint8_t is_fsm14                : 1;
  uint8_t is_fsm15                : 1;
  uint8_t is_fsm16                : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t is_fsm16                : 1;
  uint8_t is_fsm15                : 1;
  uint8_t is_fsm14                : 1;
  uint8_t is_fsm13                : 1;
  uint8_t is_fsm12                : 1;
  uint8_t is_fsm11                : 1;
  uint8_t is_fsm10                : 1;
  uint8_t is_fsm9                 : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dso_fsm_status_b_mainpage_t;

typedef struct
{
  uint8_t diff_fifo                : 8;
} lsm6dso_fifo_status1_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t diff_fifo                : 2;
  uint8_t not_used_01              : 1;
  uint8_t over_run_latched         : 1;
  uint8_t counter_bdr_ia           : 1;
  uint8_t fifo_full_ia             : 1;
  uint8_t fifo_ovr_ia              : 1;
  uint8_t fifo_wtm_ia              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fifo_wtm_ia              : 1;
  uint8_t fifo_ovr_ia              : 1;
  uint8_t fifo_full_ia             : 1;
  uint8_t counter_bdr_ia           : 1;
  uint8_t over_run_latched         : 1;
  uint8_t not_used_01              : 1;
  uint8_t diff_fifo                : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dso_fifo_status2_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ff_ia                    : 1;
  uint8_t wu_ia                    : 1;
  uint8_t single_tap               : 1;
  uint8_t double_tap               : 1;
  uint8_t d6d_ia                   : 1;
  uint8_t sleep_change_ia          : 1;
  uint8_t not_used_01              : 1;
  uint8_t timestamp_endcount       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t timestamp_endcount       : 1;
  uint8_t not_used_01              : 1;
  uint8_t sleep_change_ia          : 1;
  uint8_t d6d_ia                   : 1;
  uint8_t double_tap               : 1;
  uint8_t single_tap               : 1;
  uint8_t wu_ia                    : 1;
  uint8_t ff_ia                    : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dso_all_int_src_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t z_wu                     : 1;
  uint8_t y_wu                     : 1;
  uint8_t x_wu                     : 1;
  uint8_t wu_ia                    : 1;
  uint8_t sleep_state              : 1;
  uint8_t ff_ia                    : 1;
  uint8_t sleep_change_ia          : 1;
  uint8_t not_used_01              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t sleep_change_ia          : 1;
  uint8_t ff_ia                    : 1;
  uint8_t sleep_state              : 1;
  uint8_t wu_ia                    : 1;
  uint8_t x_wu                     : 1;
  uint8_t y_wu                     : 1;
  uint8_t z_wu                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dso_wake_up_src_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xlda                     : 1;
  uint8_t gda                      : 1;
  uint8_t tda                      : 1;
  uint8_t not_used_01              : 5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 5;
  uint8_t tda                      : 1;
  uint8_t gda                      : 1;
  uint8_t xlda                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dso_status_reg_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t z_tap                    : 1;
  uint8_t y_tap                    : 1;
  uint8_t x_tap                    : 1;
  uint8_t tap_sign                 : 1;
  uint8_t double_tap               : 1;
  uint8_t single_tap               : 1;
  uint8_t tap_ia                   : 1;
  uint8_t not_used_02              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 1;
  uint8_t tap_ia                   : 1;
  uint8_t single_tap               : 1;
  uint8_t double_tap               : 1;
  uint8_t tap_sign                 : 1;
  uint8_t x_tap                    : 1;
  uint8_t y_tap                    : 1;
  uint8_t z_tap                    : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dso_tap_src_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xl                       : 1;
  uint8_t xh                       : 1;
  uint8_t yl                       : 1;
  uint8_t yh                       : 1;
  uint8_t zl                       : 1;
  uint8_t zh                       : 1;
  uint8_t d6d_ia                   : 1;
  uint8_t den_drdy                 : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t den_drdy                 : 1;
  uint8_t d6d_ia                   : 1;
  uint8_t zh                       : 1;
  uint8_t zl                       : 1;
  uint8_t yh                       : 1;
  uint8_t yl                       : 1;
  uint8_t xh                       : 1;
  uint8_t xl                       : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dso_d6d_src_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t tap_ths_z                : 5;
  uint8_t sixd_ths                 : 2;
  uint8_t d4d_en                   : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t d4d_en                   : 1;
  uint8_t sixd_ths                 : 2;
  uint8_t tap_ths_z                : 5;
#endif /* DRV_BYTE_ORDER */
} lsm6dso_tap_ths_6d_t;

typedef struct
{
  uint8_t drdy_xl          :  1; /* Accelerometer data ready */
  uint8_t drdy_g           :  1; /* Gyroscope data ready */
  uint8_t drdy_temp        :  1; /* Temperature data ready */
uint8_t den_flag         :
  1; /* external trigger level recognition (DEN) */
uint8_t timestamp        :
  1; /* timestamp overflow (1 = int2 pin disable) */
  uint8_t free_fall        :  1; /* free fall event */
  uint8_t wake_up          :  1; /* wake up event */
  uint8_t wake_up_z        :  1; /* wake up on Z axis event */
  uint8_t wake_up_y        :  1; /* wake up on Y axis event */
  uint8_t wake_up_x        :  1; /* wake up on X axis event */
  uint8_t single_tap       :  1; /* single-tap event */
  uint8_t double_tap       :  1; /* double-tap event */
  uint8_t tap_z            :  1; /* single-tap on Z axis event */
  uint8_t tap_y            :  1; /* single-tap on Y axis event */
  uint8_t tap_x            :  1; /* single-tap on X axis event */
  uint8_t tap_sign         :  1; /* sign of tap event (0-pos / 1-neg) */
uint8_t six_d            :
  1; /* orientation change (6D/4D detection) */
uint8_t six_d_xl         :
  1; /* X-axis low 6D/4D event (under threshold) */
uint8_t six_d_xh         :
  1; /* X-axis high 6D/4D event (over threshold) */
uint8_t six_d_yl         :
  1; /* Y-axis low 6D/4D event (under threshold) */
uint8_t six_d_yh         :
  1; /* Y-axis high 6D/4D event (over threshold) */
uint8_t six_d_zl         :
  1; /* Z-axis low 6D/4D event (under threshold) */
uint8_t six_d_zh         :
  1; /* Z-axis high 6D/4D event (over threshold) */
uint8_t sleep_change     :
  1; /* Act/Inact (or Vice-versa) status changed */
uint8_t sleep_state      :
  1; /* Act/Inact status flag (0-Act / 1-Inact) */
  uint8_t step_detector    :  1; /* Step detected */
  uint8_t tilt             :  1; /* Relative tilt event detected */
uint8_t sig_mot          :
  1; /* "significant motion" event detected */
uint8_t fsm_lc           :
  1; /* fsm long counter timeout interrupt event */
  uint8_t fsm1             :  1; /* fsm 1 interrupt event */
  uint8_t fsm2             :  1; /* fsm 2 interrupt event */
  uint8_t fsm3             :  1; /* fsm 3 interrupt event */
  uint8_t fsm4             :  1; /* fsm 4 interrupt event */
  uint8_t fsm5             :  1; /* fsm 5 interrupt event */
  uint8_t fsm6             :  1; /* fsm 6 interrupt event */
  uint8_t fsm7             :  1; /* fsm 7 interrupt event */
  uint8_t fsm8             :  1; /* fsm 8 interrupt event */
  uint8_t fsm9             :  1; /* fsm 9 interrupt event */
  uint8_t fsm10            :  1; /* fsm 10 interrupt event */
  uint8_t fsm11            :  1; /* fsm 11 interrupt event */
  uint8_t fsm12            :  1; /* fsm 12 interrupt event */
  uint8_t fsm13            :  1; /* fsm 13 interrupt event */
  uint8_t fsm14            :  1; /* fsm 14 interrupt event */
  uint8_t fsm15            :  1; /* fsm 15 interrupt event */
  uint8_t fsm16            :  1; /* fsm 16 interrupt event */
  uint8_t mlc1             :  1; /* mlc 1 interrupt event */
  uint8_t mlc2             :  1; /* mlc 2 interrupt event */
  uint8_t mlc3             :  1; /* mlc 3 interrupt event */
  uint8_t mlc4             :  1; /* mlc 4 interrupt event */
  uint8_t mlc5             :  1; /* mlc 5 interrupt event */
  uint8_t mlc6             :  1; /* mlc 6 interrupt event */
  uint8_t mlc7             :  1; /* mlc 7 interrupt event */
  uint8_t mlc8             :  1; /* mlc 8 interrupt event */
  uint8_t sh_endop         :  1; /* sensor hub end operation */
uint8_t sh_slave0_nack   :
  1; /* Not acknowledge on sensor hub slave 0 */
uint8_t sh_slave1_nack   :
  1; /* Not acknowledge on sensor hub slave 1 */
uint8_t sh_slave2_nack   :
  1; /* Not acknowledge on sensor hub slave 2 */
uint8_t sh_slave3_nack   :
  1; /* Not acknowledge on sensor hub slave 3 */
uint8_t sh_wr_once       :
  1; /* "WRITE_ONCE" end on sensor hub slave 0 */
uint16_t fifo_diff       :
  10; /* Number of unread sensor data in FIFO*/
  uint8_t fifo_ovr_latched :  1; /* Latched FIFO overrun status */
uint8_t fifo_bdr         :
  1; /* FIFO Batch counter threshold reached */
  uint8_t fifo_full        :  1; /* FIFO full */
  uint8_t fifo_ovr         :  1; /* FIFO overrun */
  uint8_t fifo_th          :  1; /* FIFO threshold reached */
} lsm6dso_all_sources_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t tap_ths_y                : 5;
  uint8_t inact_en                 : 2;
  uint8_t interrupts_enable        : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t interrupts_enable        : 1;
  uint8_t inact_en                 : 2;
  uint8_t tap_ths_y                : 5;
#endif /* DRV_BYTE_ORDER */
} lsm6dso_tap_cfg2_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 2;
  uint8_t stepcounter_bit_set      : 1;
  uint8_t step_overflow            : 1;
  uint8_t step_count_delta_ia      : 1;
  uint8_t step_detected            : 1;
  uint8_t not_used_02              : 1;
  uint8_t pedo_rst_step            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t pedo_rst_step            : 1;
  uint8_t not_used_02              : 1;
  uint8_t step_detected            : 1;
  uint8_t step_count_delta_ia      : 1;
  uint8_t step_overflow            : 1;
  uint8_t stepcounter_bit_set      : 1;
  uint8_t not_used_01              : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dso_emb_func_src_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ad_det_en                : 1;
  uint8_t not_used_01              : 1;
  uint8_t fp_rejection_en          : 1;
  uint8_t carry_count_en           : 1;
  uint8_t not_used_02              : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 4;
  uint8_t carry_count_en           : 1;
  uint8_t fp_rejection_en          : 1;
  uint8_t not_used_01              : 1;
  uint8_t ad_det_en                : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dso_pedo_cmd_reg_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 3;
  uint8_t pedo_en                  : 1;
  uint8_t tilt_en                  : 1;
  uint8_t sign_motion_en           : 1;
  uint8_t not_used_02              : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 2;
  uint8_t sign_motion_en           : 1;
  uint8_t tilt_en                  : 1;
  uint8_t pedo_en                  : 1;
  uint8_t not_used_01              : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dso_emb_func_en_a_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_en                   : 1;
  uint8_t not_used_01              : 2;
  uint8_t fifo_compr_en            : 1;
  uint8_t pedo_adv_en              : 1;
  uint8_t not_used_02              : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 3;
  uint8_t pedo_adv_en              : 1;
  uint8_t fifo_compr_en            : 1;
  uint8_t not_used_01              : 2;
  uint8_t fsm_en                   : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dso_emb_func_en_b_t;

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   LSM6DSO SPI interface switch.
 * @details If set to @p TRUE the support for SPI is included.
 * @note    The default is @p FALSE.
 */
#if !defined(LSM6DSO_USE_SPI) || defined(__DOXYGEN__)
#define LSM6DSO_USE_SPI                     FALSE
#endif

/**
 * @brief   LSM6DSO shared SPI switch.
 * @details If set to @p TRUE the device acquires SPI bus ownership
 *          on each transaction.
 * @note    The default is @p FALSE. Requires SPI_USE_MUTUAL_EXCLUSION.
 */
#if !defined(LSM6DSO_SHARED_SPI) || defined(__DOXYGEN__)
#define LSM6DSO_SHARED_SPI                  FALSE
#endif

/**
 * @brief   LSM6DSO I2C interface switch.
 * @details If set to @p TRUE the support for I2C is included.
 * @note    The default is @p TRUE.
 */
#if !defined(LSM6DSO_USE_I2C) || defined(__DOXYGEN__)
#define LSM6DSO_USE_I2C                     TRUE
#endif

/**
 * @brief   LSM6DSO shared I2C switch.
 * @details If set to @p TRUE the device acquires I2C bus ownership
 *          on each transaction.
 * @note    The default is @p FALSE. Requires I2C_USE_MUTUAL_EXCLUSION.
 */
#if !defined(LSM6DSO_SHARED_I2C) || defined(__DOXYGEN__)
#define LSM6DSO_SHARED_I2C                  FALSE
#endif

/**
 * @brief   LSM6DSO advanced configurations switch.
 * @details If set to @p TRUE more configurations are available.
 * @note    The default is @p FALSE.
 */
#if !defined(LSM6DSO_USE_ADVANCED) || defined(__DOXYGEN__)
#define LSM6DSO_USE_ADVANCED                FALSE
#endif

/**
 * @brief   LSM6DSO free fall detection configuration switch.
 * @details If set to @p TRUE more configurations are available.
 * @note    The default is @p FALSE.
 */
#if !defined(LSM6DSO_USE_FREE_FALL) || defined(__DOXYGEN__)
#define LSM6DSO_USE_FREE_FALL                FALSE
#endif

/**
 * @brief   LSM6DSO interrupt configuration switch.
 * @details If set to @p TRUE more configurations are available.
 * @note    The default is @p FALSE.
 */
#if !defined(LSM6DSO_USE_INTERRUPT) || defined(__DOXYGEN__)
#define LSM6DSO_USE_INTERRUPT                FALSE
#endif

/**
 * @brief   LSM6DSO 6d orientation detection configuration switch.
 * @details If set to @p TRUE more configurations are available.
 * @note    The default is @p FALSE.
 */
#if !defined(LSM6DSO_USE_ORIENTATION) || defined(__DOXYGEN__)
#define LSM6DSO_USE_ORIENTATION                FALSE
#endif

/**
 * @brief   LSM6DSO pedometer configuration switch.
 * @details If set to @p TRUE more configurations are available.
 * @note    The default is @p FALSE.
 */
#if !defined(LSM6DSO_USE_PEDOMETER) || defined(__DOXYGEN__)
#define LSM6DSO_USE_PEDOMETER                FALSE
#endif

/**
 * @brief   Number of acquisitions for gyroscope bias removal.
 * @details This is the number of acquisitions performed to compute the
 *          bias. A repetition is required in order to remove noise.
 */
#if !defined(LSM6DSO_GYRO_BIAS_ACQ_TIMES) || defined(__DOXYGEN__)
#define LSM6DSO_GYRO_BIAS_ACQ_TIMES         50
#endif

/**
 * @brief   Settling time for gyroscope bias removal.
 * @details This is the time between each bias acquisition.
 */
#if !defined(LSM6DSO_GYRO_BIAS_SETTLING_US) || defined(__DOXYGEN__)
#define LSM6DSO_GYRO_BIAS_SETTLING_US       5000
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !(LSM6DSO_USE_SPI ^ LSM6DSO_USE_I2C)
#error "LSM6DSO_USE_SPI and LSM6DSO_USE_I2C cannot be both true or both false"
#endif

#if LSM6DSO_USE_SPI && !HAL_USE_SPI
#error "LSM6DSO_USE_SPI requires HAL_USE_SPI"
#endif

#if LSM6DSO_SHARED_SPI && !SPI_USE_MUTUAL_EXCLUSION
#error "LSM6DSO_SHARED_SPI requires SPI_USE_MUTUAL_EXCLUSION"
#endif

#if LSM6DSO_USE_I2C && !HAL_USE_I2C
#error "LSM6DSO_USE_I2C requires HAL_USE_I2C"
#endif

#if LSM6DSO_SHARED_I2C && !I2C_USE_MUTUAL_EXCLUSION
#error "LSM6DSO_SHARED_I2C requires I2C_USE_MUTUAL_EXCLUSION"
#endif

/*
 * CHTODO: Add support for LSM6DSO over SPI.
 */
#if LSM6DSO_USE_SPI
#error "LSM6DSO over SPI still not supported"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @name    LSM6DSO data structures and types.
 * @{
 */
/**
 * @brief   Structure representing a LSM6DSO driver.
 */
typedef struct LSM6DSODriver LSM6DSODriver;

/**
 * @brief  Accelerometer and Gyroscope Slave Address.
 */
typedef enum {
  LSM6DSO_SAD_GND = 0x6A,           /**< SAD pin connected to GND.          */
  LSM6DSO_SAD_VCC = 0x6B            /**< SAD pin connected to VCC.          */
} lsm6dso_sad_t;

/**
 * @brief   LSM6DSO accelerometer subsystem full scale.
 */
typedef enum {
  LSM6DSO_ACC_FS_2G = 0,         /**< Full scale �2g.                    */
  LSM6DSO_ACC_FS_4G = 2,         /**< Full scale �4g.                    */
  LSM6DSO_ACC_FS_8G = 3,         /**< Full scale �8g.                    */
  LSM6DSO_ACC_FS_16G = 1         /**< Full scale �16g.                   */
} lsm6dso_acc_fs_t;

/**
 * @brief   LSM6DSO accelerometer subsystem output data rate.
 */
typedef enum {
  LSM6DSO_ACC_ODR_PD = 0,        /**< Power down                         */
  LSM6DSO_ACC_ODR_1P6Hz = 11,     /**< ODR 1.6 Hz (Low Power only)        */
  LSM6DSO_ACC_ODR_12P5Hz = 1,    /**< ODR 12.5 Hz                        */
  LSM6DSO_ACC_ODR_26Hz = 2,      /**< ODR 26 Hz                          */
  LSM6DSO_ACC_ODR_52Hz = 3,      /**< ODR 52 Hz                          */
  LSM6DSO_ACC_ODR_104Hz = 4,     /**< ODR 104 Hz                         */
  LSM6DSO_ACC_ODR_208Hz = 5,     /**< ODR 208 Hz                         */
  LSM6DSO_ACC_ODR_417Hz = 6,     /**< ODR 417 Hz                         */
  LSM6DSO_ACC_ODR_833Hz = 7,     /**< ODR 833 Hz                         */
  LSM6DSO_ACC_ODR_1P67Hz = 8,    /**< ODR 1.67 kHz                       */
  LSM6DSO_ACC_ODR_3P33Hz = 9,    /**< ODR 3.33 kHz                       */
  LSM6DSO_ACC_ODR_6P66Hz = 10     /**< ODR 6.66 kHz                       */
} lsm6dso_acc_odr_t;

/**
 * @brief   LSM6DSO accelerometer subsystem output data rate.
 */
typedef enum {
  LSM6DSO_ACC_LP_DISABLED = 0x00,   /**< Low power disabled                 */
  LSM6DSO_ACC_LP_ENABLED = 0x10     /**< Low power enabled                  */
} lsm6dso_acc_lp_t;

/**
 * @brief LSM6DSO gyroscope subsystem full scale.
 */
typedef enum {
  LSM6DSO_GYRO_FS_125DPS  = 1,   /**< Full scale �125 degree per second  */
  LSM6DSO_GYRO_FS_250DPS  = 0,   /**< Full scale �250 degree per second  */
  LSM6DSO_GYRO_FS_500DPS  = 2,   /**< Full scale �500 degree per second  */
  LSM6DSO_GYRO_FS_1000DPS = 4,   /**< Full scale �1000 degree per second */
  LSM6DSO_GYRO_FS_2000DPS = 6    /**< Full scale �2000 degree per second */
} lsm6dso_gyro_fs_t;

/**
 * @brief   LSM6DSO gyroscope subsystem output data rate.
 */
typedef enum {
  LSM6DSO_GYRO_ODR_PD = 0,       /**< Power down                         */
  LSM6DSO_GYRO_ODR_12P5Hz = 1,   /**< ODR 12.5 Hz                        */
  LSM6DSO_GYRO_ODR_26Hz = 2,     /**< ODR 26 Hz                          */
  LSM6DSO_GYRO_ODR_52Hz = 3,     /**< ODR 52 Hz                          */
  LSM6DSO_GYRO_ODR_104Hz = 4,    /**< ODR 104 Hz                         */
  LSM6DSO_GYRO_ODR_208Hz = 5,    /**< ODR 208 Hz                         */
  LSM6DSO_GYRO_ODR_417Hz = 6,    /**< ODR 417 Hz                         */
  LSM6DSO_GYRO_ODR_833Hz = 7,    /**< ODR 833 Hz                         */
  LSM6DSO_GYRO_ODR_1P67Hz = 8,   /**< ODR 1.67 kHz                       */
  LSM6DSO_GYRO_ODR_3P33Hz = 9,   /**< ODR 3.33 kHz                       */
  LSM6DSO_GYRO_ODR_6P67Hz = 10    /**< ODR 6.67 kHz                       */
} lsm6dso_gyro_odr_t;

/**
 * @brief LSM6DSO gyroscope subsystem low mode configuration.
 */
typedef enum {
  LSM6DSO_GYRO_LP_DISABLED = 0x00,  /**< Low power mode disabled.           */
  LSM6DSO_GYRO_LP_ENABLED = 0x80    /**< Low power mode enabled.            */
} lsm6dso_gyro_lp_t;

/**
 * @brief  LSM6DSO gyroscope subsystem output selection.
 */
typedef enum {
  LSM6DSO_GYRO_LPF_DISABLED = -1,   /**< Low pass filter disabled.          */
  LSM6DSO_GYRO_LPF_FTYPE0 = 0x00,   /**< Refer to table 68 of Datasheet.    */
  LSM6DSO_GYRO_LPF_FTYPE1 = 0x01,   /**< Refer to table 68 of Datasheet.    */
  LSM6DSO_GYRO_LPF_FTYPE2 = 0x10,   /**< Refer to table 68 of Datasheet.    */
  LSM6DSO_GYRO_LPF_FTYPE3 = 0x11    /**< Refer to table 68 of Datasheet.    */
} lsm6dso_gyro_lpf_t;

/**
 * @brief LSM6DSO block data update.
 */
typedef enum {
  LSM6DSO_BDU_CONTINUOUS = 0x00,    /**< Block data continuously updated.   */
  LSM6DSO_BDU_BLOCKED = 0x40        /**< Block data updated after reading.  */
} lsm6dso_bdu_t;

/**
 * @brief LSM6DSO endianness.
 */
typedef enum {
  LSM6DSO_END_LITTLE = 0x00,        /**< Little endian.                     */
  LSM6DSO_END_BIG = 0x20            /**< Big endian.                        */
} lsm6dso_end_t;

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  LSM6DSO_UNINIT = 0,               /**< Not initialized.                   */
  LSM6DSO_STOP = 1,                 /**< Stopped.                           */
  LSM6DSO_READY = 2,                /**< Ready.                             */
} lsm6dso_state_t;

typedef enum
{
  LSM6DSO_DEG_80  = 0,
  LSM6DSO_DEG_70  = 1,
  LSM6DSO_DEG_60  = 2,
  LSM6DSO_DEG_50  = 3,
} lsm6dso_sixd_ths_t;

typedef struct
{
  uint8_t sig_mot      : 1; /* significant motion */
  uint8_t tilt         : 1; /* tilt detection  */
  uint8_t step         : 1; /* step counter/detector */
  uint8_t step_adv     : 1; /* step counter advanced mode */
  uint8_t fsm          : 1; /* finite state machine */
  uint8_t fifo_compr   : 1; /* FIFO compression */
} lsm6dso_emb_sens_t;

typedef enum
{
  LSM6DSO_PEDO_BASE_MODE            = 0x00,
  LSM6DSO_FALSE_STEP_REJ            = 0x10,
  LSM6DSO_FALSE_STEP_REJ_ADV_MODE   = 0x30,
} lsm6dso_pedo_md_t;

/**
 * @brief LSM6DSO configuration structure.
 */
typedef struct {
#if (LSM6DSO_USE_SPI) || defined(__DOXYGEN__)
  /**
   * @brief SPI driver associated to this LSM6DSO.
   */
  SPIDriver                 *spip;
  /**
   * @brief SPI configuration associated to this LSM6DSO accelerometer
   *        subsystem.
   */
  const SPIConfig           *accspicfg;
#endif /* LSM6DSO_USE_SPI */
#if (LSM6DSO_USE_I2C) || defined(__DOXYGEN__)
  /**
   * @brief I2C driver associated to this LSM6DSO.
   */
  I2CDriver                 *i2cp;
  /**
   * @brief I2C configuration associated to this LSM6DSO accelerometer
   *        subsystem.
   */
  const I2CConfig           *i2ccfg;
  /**
   * @brief LSM6DSO Slave Address
   */
  lsm6dso_sad_t             slaveaddress;
#endif /* LSM6DSO_USE_I2C */
  /**
   * @brief LSM6DSO accelerometer subsystem initial sensitivity.
   */
  float                     *accsensitivity;
  /**
   * @brief LSM6DSO accelerometer subsystem initial bias.
   */
  float                     *accbias;
  /**
   * @brief LSM6DSO accelerometer subsystem full scale.
   */
  lsm6dso_acc_fs_t          accfullscale;
  /**
   * @brief LSM6DSO accelerometer subsystem output data rate.
   */
  lsm6dso_acc_odr_t         accoutdatarate;
#if LSM6DSO_USE_ADVANCED || defined(__DOXYGEN__)
  /**
   * @brief LSM6DSO accelerometer subsystem low power mode.
   */
  lsm6dso_acc_lp_t          acclpmode;
#endif /* LSM6DSO_USE_ADVANCED */
  /**
   * @brief LSM6DSO gyroscope subsystem initial sensitivity.
   */
  float                     *gyrosensitivity;
  /**
   * @brief LSM6DSO gyroscope subsystem initial bias.
   */
  float                     *gyrobias;
  /**
   * @brief LSM6DSO gyroscope subsystem full scale.
   */
  lsm6dso_gyro_fs_t         gyrofullscale;
  /**
   * @brief LSM6DSO gyroscope subsystem output data rate.
   */
  lsm6dso_gyro_odr_t        gyrooutdatarate;
#if LSM6DSO_USE_ADVANCED || defined(__DOXYGEN__)
  /**
   * @brief LSM6DSO gyroscope subsystem low mode configuration.
   */
  lsm6dso_gyro_lp_t         gyrolpmode;
  /**
   * @brief LSM6DSO gyroscope subsystem low pass filter configuration.
   */
  lsm6dso_gyro_lpf_t        gyrolowpassfilter;
  /**
   * @brief LSM6DSO block data update
   */
  lsm6dso_bdu_t             blockdataupdate;
  /**
   * @brief LSM6DSO  endianness
   */
  lsm6dso_end_t             endianness;
#endif /* LSM6DSO_USE_ADVANCED */
#if LSM6DSO_USE_FREE_FALL || defined(__DOXYGEN__)
  /**
   * @brief LSM6DSO free fall duration.
   */
  uint8_t         freefallduration;

  /**
   * @brief LSM6DSO free fall threshold.
   */
  lsm6dso_ff_ths_t freefallthreshold;

#endif /* LSM6DSO_USE_FREE_FALL */

#if LSM6DSO_USE_INTERRUPT || defined(__DOXYGEN__)
  /**
   * @brief LSM6DSO value of lir in reg TAP_CFG0.
   */
  lsm6dso_lir_t lirvalue;

#endif /* LSM6DSO_USE_INTERRUPT */

#if LSM6DSO_USE_ORIENTATION || defined(__DOXYGEN__)
  /**
   * @brief LSM6DSO 6D orientation threshold.
   */
  lsm6dso_sixd_ths_t sixdthreshold;

  /**
   * @brief LSM6DSO 6D lowpass filter enable.
   */
  uint8_t sixdlowpassenable;

  /**
   * @brief LSM6DSO 4D orientation detection enable.
   * 4D orientation detection disable Z-axis events.
   */
  uint8_t sixd4dmodeenable;

#endif /* LSM6DSO_USE_ORIENTATION */

#if LSM6DSO_USE_PEDOMETER || defined(__DOXYGEN__)
  /**
   * @brief LSM6DSO pedometer mode.
   */
  lsm6dso_pedo_md_t pedomode;

#endif /* LSM6DSO_USE_PEDOMETER */
} LSM6DSOConfig;

/**
 * @brief   @p LSM6DSO specific methods.
 */
#define _lsm6dso_methods_alone                                              \
  /* Change full scale value of LSM6DSO accelerometer subsystem .*/         \
  msg_t (*acc_set_full_scale)(LSM6DSODriver *devp, lsm6dso_acc_fs_t fs);    \
  /* Change full scale value of LSM6DSO gyroscope subsystem .*/             \
  msg_t (*gyro_set_full_scale)(LSM6DSODriver *devp, lsm6dso_gyro_fs_t fs);

/**
 * @brief   @p LSM6DSO specific methods with inherited ones.
 */
#define _lsm6dso_methods                                                    \
  _base_object_methods                                                      \
  _lsm6dso_methods_alone

/**
 * @extends BaseObjectVMT
 *
 * @brief @p LSM6DSO virtual methods table.
 */
struct LSM6DSOVMT {
  _lsm6dso_methods
};

/**
 * @brief   @p LSM6DSODriver specific data.
 */
#define _lsm6dso_data                                                       \
  _base_sensor_data                                                         \
  /* Driver state.*/                                                        \
  lsm6dso_state_t           state;                                          \
  /* Current configuration data.*/                                          \
  const LSM6DSOConfig       *config;                                        \
  /* Accelerometer subsystem axes number.*/                                 \
  size_t                    accaxes;                                        \
  /* Accelerometer subsystem current sensitivity.*/                         \
  float                     accsensitivity[LSM6DSO_ACC_NUMBER_OF_AXES];     \
  /* Accelerometer subsystem current bias .*/                               \
  float                     accbias[LSM6DSO_ACC_NUMBER_OF_AXES];            \
  /* Accelerometer subsystem current full scale value.*/                    \
  float                     accfullscale;                                   \
  /* Gyroscope subsystem axes number.*/                                     \
  size_t                    gyroaxes;                                       \
  /* Gyroscope subsystem current sensitivity.*/                             \
  float                     gyrosensitivity[LSM6DSO_GYRO_NUMBER_OF_AXES];   \
  /* Gyroscope subsystem current Bias.*/                                    \
  float                     gyrobias[LSM6DSO_GYRO_NUMBER_OF_AXES];          \
  /* Gyroscope subsystem current full scale value.*/                        \
  float                     gyrofullscale;

/**
 * @brief LSM6DSO 6-axis accelerometer/gyroscope class.
 */
struct LSM6DSODriver {
  /** @brief Virtual Methods Table.*/
  const struct LSM6DSOVMT     *vmt;
  /** @brief Base accelerometer interface.*/
  BaseAccelerometer           acc_if;
  /** @brief Base gyroscope interface.*/
  BaseGyroscope               gyro_if;
  _lsm6dso_data
};
/** @} */

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Return the number of axes of the BaseAccelerometer.
 *
 * @param[in] devp      pointer to @p LSM6DSODriver.
 *
 * @return              the number of axes.
 *
 * @api
 */
#define lsm6dsoAccelerometerGetAxesNumber(devp)                             \
        accelerometerGetAxesNumber(&((devp)->acc_if))

/**
 * @brief   Retrieves raw data from the BaseAccelerometer.
 * @note    This data is retrieved from MEMS register without any algebraical
 *          manipulation.
 * @note    The axes array must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] devp      pointer to @p LSM6DSODriver.
 * @param[out] axes     a buffer which would be filled with raw data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 *
 * @api
 */
#define lsm6dsoAccelerometerReadRaw(devp, axes)                             \
        accelerometerReadRaw(&((devp)->acc_if), axes)

void lsm6dsoAllSourcesGet(LSM6DSODriver *devp,
                                lsm6dso_all_sources_t *val);

void lsm6dsoNumberOfStepsGet(LSM6DSODriver *devp, uint16_t *val);

/**
 * @brief   Retrieves cooked data from the BaseAccelerometer.
 * @note    This data is manipulated according to the formula
 *          cooked = (raw * sensitivity) - bias.
 * @note    Final data is expressed as milli-G.
 * @note    The axes array must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] devp      pointer to @p LSM6DSODriver.
 * @param[out] axes     a buffer which would be filled with cooked data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 *
 * @api
 */
#define lsm6dsoAccelerometerReadCooked(devp, axes)                          \
        accelerometerReadCooked(&((devp)->acc_if), axes)

/**
 * @brief   Set bias values for the BaseAccelerometer.
 * @note    Bias must be expressed as milli-G.
 * @note    The bias buffer must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] devp      pointer to @p LSM6DSODriver.
 * @param[in] bp        a buffer which contains biases.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lsm6dsoAccelerometerSetBias(devp, bp)                               \
        accelerometerSetBias(&((devp)->acc_if), bp)

/**
 * @brief   Reset bias values for the BaseAccelerometer.
 * @note    Default biases value are obtained from device datasheet when
 *          available otherwise they are considered zero.
 *
 * @param[in] devp      pointer to @p LSM6DSODriver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lsm6dsoAccelerometerResetBias(devp)                                 \
        accelerometerResetBias(&((devp)->acc_if))

/**
 * @brief   Set sensitivity values for the BaseAccelerometer.
 * @note    Sensitivity must be expressed as milli-G/LSB.
 * @note    The sensitivity buffer must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] devp      pointer to @p LSM6DSODriver.
 * @param[in] sp        a buffer which contains sensitivities.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lsm6dsoAccelerometerSetSensitivity(devp, sp)                        \
        accelerometerSetSensitivity(&((devp)->acc_if), sp)

/**
 * @brief   Reset sensitivity values for the BaseAccelerometer.
 * @note    Default sensitivities value are obtained from device datasheet.
 *
 * @param[in] devp      pointer to @p LSM6DSODriver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 *
 * @api
 */
#define lsm6dsoAccelerometerResetSensitivity(devp)                          \
        accelerometerResetSensitivity(&((devp)->acc_if))

/**
 * @brief   Changes the LSM6DSODriver accelerometer fullscale value.
 * @note    This function also rescale sensitivities and biases based on
 *          previous and next fullscale value.
 * @note    A recalibration is highly suggested after calling this function.
 *
 * @param[in] devp      pointer to @p LSM6DSODriver.
 * @param[in] fs        new fullscale value.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 *
 * @api
 */
#define lsm6dsoAccelerometerSetFullScale(devp, fs)                          \
        (devp)->vmt->acc_set_full_scale(devp, fs)

/**
 * @brief   Return the number of axes of the BaseGyroscope.
 *
 * @param[in] devp      pointer to @p LSM6DSODriver.
 *
 * @return              the number of axes.
 *
 * @api
 */
#define lsm6dsoGyroscopeGetAxesNumber(devp)                                 \
        gyroscopeGetAxesNumber(&((devp)->gyro_if))

/**
 * @brief   Retrieves raw data from the BaseGyroscope.
 * @note    This data is retrieved from MEMS register without any algebraical
 *          manipulation.
 * @note    The axes array must be at least the same size of the
 *          BaseGyroscope axes number.
 *
 * @param[in] devp      pointer to @p LSM6DSODriver.
 * @param[out] axes     a buffer which would be filled with raw data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 *
 * @api
 */
#define lsm6dsoGyroscopeReadRaw(devp, axes)                                 \
        gyroscopeReadRaw(&((devp)->gyro_if), axes)

/**
 * @brief   Retrieves cooked data from the BaseGyroscope.
 * @note    This data is manipulated according to the formula
 *          cooked = (raw * sensitivity) - bias.
 * @note    Final data is expressed as DPS.
 * @note    The axes array must be at least the same size of the
 *          BaseGyroscope axes number.
 *
 * @param[in] devp      pointer to @p LSM6DSODriver.
 * @param[out] axes     a buffer which would be filled with cooked data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 *
 * @api
 */
#define lsm6dsoGyroscopeReadCooked(devp, axes)                              \
        gyroscopeReadCooked(&((devp)->gyro_if), axes)

/**
 * @brief   Samples bias values for the BaseGyroscope.
 * @note    The LSM6DSO shall not be moved during the whole procedure.
 * @note    After this function internal bias is automatically updated.
 * @note    The behavior of this function depends on @p LSM6DSO_BIAS_ACQ_TIMES
 *          and @p LSM6DSO_BIAS_SETTLING_US.
 *
 * @param[in] devp      pointer to @p LSM6DSODriver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 *
 * @api
 */
#define lsm6dsoGyroscopeSampleBias(devp)                                    \
        gyroscopeSampleBias(&((devp)->gyro_if))

/**
 * @brief   Set bias values for the BaseGyroscope.
 * @note    Bias must be expressed as DPS.
 * @note    The bias buffer must be at least the same size of the BaseGyroscope
 *          axes number.
 *
 * @param[in] devp      pointer to @p LSM6DSODriver.
 * @param[in] bp        a buffer which contains biases.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lsm6dsoGyroscopeSetBias(devp, bp)                                   \
        gyroscopeSetBias(&((devp)->gyro_if), bp)

/**
 * @brief   Reset bias values for the BaseGyroscope.
 * @note    Default biases value are obtained from device datasheet when
 *          available otherwise they are considered zero.
 *
 * @param[in] devp      pointer to @p LSM6DSODriver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lsm6dsoGyroscopeResetBias(devp)                                     \
        gyroscopeResetBias(&((devp)->gyro_if))

/**
 * @brief   Set sensitivity values for the BaseGyroscope.
 * @note    Sensitivity must be expressed as DPS/LSB.
 * @note    The sensitivity buffer must be at least the same size of the
 *          BaseGyroscope axes number.
 *
 * @param[in] devp      pointer to @p LSM6DSODriver.
 * @param[in] sp        a buffer which contains sensitivities.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lsm6dsoGyroscopeSetSensitivity(devp, sp)                            \
        gyroscopeSetSensitivity(&((devp)->gyro_if), sp)

/**
 * @brief   Reset sensitivity values for the BaseGyroscope.
 * @note    Default sensitivities value are obtained from device datasheet.
 *
 * @param[in] devp      pointer to @p LSM6DSODriver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 *
 * @api
 */
#define lsm6dsoGyroscopeResetSensitivity(devp)                              \
        gyroscopeResetSensitivity(&((devp)->gyro_if))

/**
 * @brief   Changes the LSM6DSODriver gyroscope fullscale value.
 * @note    This function also rescale sensitivities and biases based on
 *          previous and next fullscale value.
 * @note    A recalibration is highly suggested after calling this function.
 *
 * @param[in] devp      pointer to @p LSM6DSODriver.
 * @param[in] fs        new fullscale value.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 *
 * @api
 */
#define lsm6dsoGyroscopeSetFullScale(devp, fs)                              \
        (devp)->vmt->acc_set_full_scale(devp, fs)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void lsm6dsoObjectInit(LSM6DSODriver *devp);
  void lsm6dsoStart(LSM6DSODriver *devp, const LSM6DSOConfig *config);
  void lsm6dsoStop(LSM6DSODriver *devp);
#ifdef __cplusplus
}
#endif

#endif /* _LSM6DSO_H_ */

/** @} */
