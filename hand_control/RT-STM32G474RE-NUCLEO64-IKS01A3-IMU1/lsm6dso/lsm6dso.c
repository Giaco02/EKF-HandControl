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
 * @file    lsm6dso.c
 * @brief   LSM6DSO MEMS interface module code.
 *
 * @addtogroup LSM6DSO
 * @ingroup EX_ST
 * @{
 */

#include "hal.h"
#include "lsm6dso.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

#if (LSM6DSO_USE_I2C) || defined(__DOXYGEN__)
/**
 * @brief   Reads registers value using I2C.
 * @pre     The I2C interface must be initialized and the driver started.
 * @note    IF_ADD_INC bit must be 1 in CTRL_REG8
 *
 * @param[in]  i2cp      pointer to the I2C interface
 * @param[in]  sad       slave address without R bit
 * @param[in]  reg       first sub-register address
 * @param[out] rxbuf     pointer to an output buffer
 * @param[in]  n         number of consecutive register to read
 * @return               the operation status.
 * @notapi
 */
msg_t lsm6dsoI2CReadRegister(I2CDriver *i2cp, lsm6dso_sad_t sad, uint8_t reg,
                             uint8_t* rxbuf, size_t n) {

  return i2cMasterTransmitTimeout(i2cp, sad, &reg, 1, rxbuf, n,
                                  TIME_INFINITE);
}

/**
 * @brief   Writes a value into a register using I2C.
 * @pre     The I2C interface must be initialized and the driver started.
 *
 * @param[in] i2cp       pointer to the I2C interface
 * @param[in] sad        slave address without R bit
 * @param[in] txbuf      buffer containing sub-address value in first position
 *                       and values to write
 * @param[in] n          size of txbuf less one (not considering the first
 *                       element)
 * @return               the operation status.
 * @notapi
 */
#define lsm6dsoI2CWriteRegister(i2cp, sad, txbuf, n)                        \
        i2cMasterTransmitTimeout(i2cp, sad, txbuf, n + 1, NULL, 0,          \
                                  TIME_INFINITE)
#endif /* LSM6DSO_USE_I2C */

/**
 * @brief   Return the number of axes of the BaseAccelerometer.
 *
 * @param[in] ip        pointer to @p BaseAccelerometer interface.
 *
 * @return              the number of axes.
 */
static size_t acc_get_axes_number(void *ip) {
  (void)ip;

  return LSM6DSO_ACC_NUMBER_OF_AXES;
}

/**
 * @brief   Retrieves raw data from the BaseAccelerometer.
 * @note    This data is retrieved from MEMS register without any algebraical
 *          manipulation.
 * @note    The axes array must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] ip        pointer to @p BaseAccelerometer interface.
 * @param[out] axes     a buffer which would be filled with raw data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 */
static msg_t acc_read_raw(void *ip, int32_t axes[]) {
  LSM6DSODriver* devp;
  uint8_t buff [LSM6DSO_ACC_NUMBER_OF_AXES * 2], i;
  int16_t tmp;
  msg_t msg;

  osalDbgCheck((ip != NULL) && (axes != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LSM6DSODriver*, (BaseAccelerometer*)ip);

  osalDbgAssert((devp->state == LSM6DSO_READY),
                "acc_read_raw(), invalid state");
#if LSM6DSO_USE_I2C
  osalDbgAssert((devp->config->i2cp->state == I2C_READY),
                "acc_read_raw(), channel not ready");

#if LSM6DSO_SHARED_I2C
  i2cAcquireBus(devp->config->i2cp);
  i2cStart(devp->config->i2cp,
           devp->config->i2ccfg);
#endif /* LSM6DSO_SHARED_I2C */

  msg = lsm6dsoI2CReadRegister(devp->config->i2cp, devp->config->slaveaddress,
                               LSM6DSO_AD_OUTX_L_XL, buff,
                               LSM6DSO_ACC_NUMBER_OF_AXES * 2);

#if LSM6DSO_SHARED_I2C
  i2cReleaseBus(devp->config->i2cp);
#endif /* LSM6DSO_SHARED_I2C */
#endif /* LSM6DSO_USE_I2C */
  if(msg == MSG_OK)
    for(i = 0; i < LSM6DSO_ACC_NUMBER_OF_AXES; i++) {
      tmp = buff[2 * i] + (buff[2 * i + 1] << 8);
      axes[i] = (int32_t)tmp;
    }
  return msg;
}

/**
 * @brief   Retrieves cooked data from the BaseAccelerometer.
 * @note    This data is manipulated according to the formula
 *          cooked = (raw * sensitivity) - bias.
 * @note    Final data is expressed as milli-G.
 * @note    The axes array must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] ip        pointer to @p BaseAccelerometer interface.
 * @param[out] axes     a buffer which would be filled with cooked data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 */
static msg_t acc_read_cooked(void *ip, float axes[]) {
  LSM6DSODriver* devp;
  uint32_t i;
  int32_t raw[LSM6DSO_ACC_NUMBER_OF_AXES];
  msg_t msg;

  osalDbgCheck((ip != NULL) && (axes != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LSM6DSODriver*, (BaseAccelerometer*)ip);

  osalDbgAssert((devp->state == LSM6DSO_READY),
                "acc_read_cooked(), invalid state");

  msg = acc_read_raw(ip, raw);
  for(i = 0; i < LSM6DSO_ACC_NUMBER_OF_AXES; i++) {
    axes[i] = (raw[i] * devp->accsensitivity[i]) - devp->accbias[i];
  }
  return msg;
}

/**
 * @brief   Set bias values for the BaseAccelerometer.
 * @note    Bias must be expressed as milli-G.
 * @note    The bias buffer must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] ip        pointer to @p BaseAccelerometer interface.
 * @param[in] bp        a buffer which contains biases.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t acc_set_bias(void *ip, float *bp) {
  LSM6DSODriver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  osalDbgCheck((ip != NULL) && (bp != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LSM6DSODriver*, (BaseAccelerometer*)ip);

  osalDbgAssert((devp->state == LSM6DSO_READY),
                "acc_set_bias(), invalid state");

  for(i = 0; i < LSM6DSO_ACC_NUMBER_OF_AXES; i++) {
    devp->accbias[i] = bp[i];
  }
  return msg;
}

/**
 * @brief   Reset bias values for the BaseAccelerometer.
 * @note    Default biases value are obtained from device datasheet when
 *          available otherwise they are considered zero.
 *
 * @param[in] ip        pointer to @p BaseAccelerometer interface.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t acc_reset_bias(void *ip) {
  LSM6DSODriver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  osalDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LSM6DSODriver*, (BaseAccelerometer*)ip);

  osalDbgAssert((devp->state == LSM6DSO_READY),
                "acc_reset_bias(), invalid state");

  for(i = 0; i < LSM6DSO_ACC_NUMBER_OF_AXES; i++)
    devp->accbias[i] = LSM6DSO_ACC_BIAS;
  return msg;
}

/**
 * @brief   Set sensitivity values for the BaseAccelerometer.
 * @note    Sensitivity must be expressed as milli-G/LSB.
 * @note    The sensitivity buffer must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] ip        pointer to @p BaseAccelerometer interface.
 * @param[in] sp        a buffer which contains sensitivities.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t acc_set_sensivity(void *ip, float *sp) {
  LSM6DSODriver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LSM6DSODriver*, (BaseAccelerometer*)ip);

  osalDbgCheck((ip != NULL) && (sp != NULL));

  osalDbgAssert((devp->state == LSM6DSO_READY),
                "acc_set_sensivity(), invalid state");

  for(i = 0; i < LSM6DSO_ACC_NUMBER_OF_AXES; i++) {
    devp->accsensitivity[i] = sp[i];
  }
  return msg;
}

/**
 * @brief   Reset sensitivity values for the BaseAccelerometer.
 * @note    Default sensitivities value are obtained from device datasheet.
 *
 * @param[in] ip        pointer to @p BaseAccelerometer interface.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 */
static msg_t acc_reset_sensivity(void *ip) {
  LSM6DSODriver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  osalDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LSM6DSODriver*, (BaseAccelerometer*)ip);

  osalDbgAssert((devp->state == LSM6DSO_READY),
                "acc_reset_sensivity(), invalid state");

  if(devp->config->accfullscale == LSM6DSO_ACC_FS_2G)
    for(i = 0; i < LSM6DSO_ACC_NUMBER_OF_AXES; i++)
      devp->accsensitivity[i] = LSM6DSO_ACC_SENS_2G;
  else if(devp->config->accfullscale == LSM6DSO_ACC_FS_4G)
	for(i = 0; i < LSM6DSO_ACC_NUMBER_OF_AXES; i++)
      devp->accsensitivity[i] = LSM6DSO_ACC_SENS_4G;
  else if(devp->config->accfullscale == LSM6DSO_ACC_FS_8G)
	for(i = 0; i < LSM6DSO_ACC_NUMBER_OF_AXES; i++)
      devp->accsensitivity[i] = LSM6DSO_ACC_SENS_8G;
  else if(devp->config->accfullscale == LSM6DSO_ACC_FS_16G)
	for(i = 0; i < LSM6DSO_ACC_NUMBER_OF_AXES; i++)
      devp->accsensitivity[i] = LSM6DSO_ACC_SENS_16G;
  else {
    osalDbgAssert(FALSE, "reset_sensivity(), accelerometer full scale issue");
    msg = MSG_RESET;
  }
  return msg;
}

/**
 * @brief   Changes the LSM6DSODriver accelerometer fullscale value.
 * @note    This function also rescale sensitivities and biases based on
 *          previous and next fullscale value.
 * @note    A recalibration is highly suggested after calling this function.
 *
 * @param[in] devp      pointer to @p LSM6DSODriver interface.
 * @param[in] fs        new fullscale value.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 */
static msg_t acc_set_full_scale(LSM6DSODriver *devp, lsm6dso_acc_fs_t fs) {
  float newfs, scale;
  uint8_t i, buff[2];
  msg_t msg;

  osalDbgCheck(devp != NULL);

  osalDbgAssert((devp->state == LSM6DSO_READY),
                "acc_set_full_scale(), invalid state");
  osalDbgAssert((devp->config->i2cp->state == I2C_READY),
                "acc_set_full_scale(), channel not ready");

  /* Computing new fullscale value.*/
  if(fs == LSM6DSO_ACC_FS_2G) {
    newfs = LSM6DSO_ACC_2G;
  }
  else if(fs == LSM6DSO_ACC_FS_4G) {
    newfs = LSM6DSO_ACC_4G;
  }
  else if(fs == LSM6DSO_ACC_FS_8G) {
    newfs = LSM6DSO_ACC_8G;
  }
  else if(fs == LSM6DSO_ACC_FS_16G) {
    newfs = LSM6DSO_ACC_16G;
  }
  else {
    msg = MSG_RESET;
    return msg;
  }

  if(newfs != devp->accfullscale) {
    /* Computing scale value.*/
    scale = newfs / devp->accfullscale;
    devp->accfullscale = newfs;

#if LSM6DSO_SHARED_I2C
		i2cAcquireBus(devp->config->i2cp);
		i2cStart(devp->config->i2cp,
						 devp->config->i2ccfg);
#endif /* LSM6DSO_SHARED_I2C */

    /* Updating register.*/
    msg = lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_AD_CTRL1_XL, &buff[1], 1);

#if LSM6DSO_SHARED_I2C
        i2cReleaseBus(devp->config->i2cp);
#endif /* LSM6DSO_SHARED_I2C */

    if(msg != MSG_OK)
      return msg;

    buff[1] &= ~(LSMDSL_CTRL1_XL_FS_MASK);
    buff[1] |= fs;
    buff[0] = LSM6DSO_AD_CTRL1_XL;

#if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
#endif /* LSM6DSO_SHARED_I2C */

    msg = lsm6dsoI2CWriteRegister(devp->config->i2cp,
                                  devp->config->slaveaddress, buff, 1);

#if LSM6DSO_SHARED_I2C
		i2cReleaseBus(devp->config->i2cp);
#endif /* LSM6DSO_SHARED_I2C */

    if(msg != MSG_OK)
      return msg;

    /* Scaling sensitivity and bias. Re-calibration is suggested anyway.*/
    for(i = 0; i < LSM6DSO_ACC_NUMBER_OF_AXES; i++) {
      devp->accsensitivity[i] *= scale;
      devp->accbias[i] *= scale;
    }
  }
  return msg;
}

/**
 * @brief   Return the number of axes of the BaseGyroscope.
 *
 * @param[in] ip        pointer to @p BaseGyroscope interface.
 *
 * @return              the number of axes.
 */
static size_t gyro_get_axes_number(void *ip) {
  (void)ip;

  return LSM6DSO_GYRO_NUMBER_OF_AXES;
}

/**
 * @brief   Retrieves raw data from the BaseGyroscope.
 * @note    This data is retrieved from MEMS register without any algebraical
 *          manipulation.
 * @note    The axes array must be at least the same size of the
 *          BaseGyroscope axes number.
 *
 * @param[in] ip        pointer to @p BaseGyroscope interface.
 * @param[out] axes     a buffer which would be filled with raw data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 */
static msg_t gyro_read_raw(void *ip, int32_t axes[LSM6DSO_GYRO_NUMBER_OF_AXES]) {
  LSM6DSODriver* devp;
  int16_t tmp;
  uint8_t i, buff [2 * LSM6DSO_GYRO_NUMBER_OF_AXES];
  msg_t msg = MSG_OK;

  osalDbgCheck((ip != NULL) && (axes != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LSM6DSODriver*, (BaseGyroscope*)ip);

  osalDbgAssert((devp->state == LSM6DSO_READY),
                "gyro_read_raw(), invalid state");
#if LSM6DSO_USE_I2C
  osalDbgAssert((devp->config->i2cp->state == I2C_READY),
                "gyro_read_raw(), channel not ready");

#if LSM6DSO_SHARED_I2C
  i2cAcquireBus(devp->config->i2cp);
  i2cStart(devp->config->i2cp,
           devp->config->i2ccfg);
#endif /* LSM6DSO_SHARED_I2C */

  msg = lsm6dsoI2CReadRegister(devp->config->i2cp, devp->config->slaveaddress,
                               LSM6DSO_AD_OUTX_L_G, buff,
                               LSM6DSO_GYRO_NUMBER_OF_AXES * 2);

#if	LSM6DSO_SHARED_I2C
  i2cReleaseBus(devp->config->i2cp);
#endif /* LSM6DSO_SHARED_I2C */
#endif /* LSM6DSO_USE_I2C */

    for(i = 0; i < LSM6DSO_GYRO_NUMBER_OF_AXES; i++) {
      tmp = buff[2 * i] + (buff[2 * i + 1] << 8);
      axes[i] = (int32_t)tmp;
    }
  return msg;
}

/**
 * @brief   Retrieves cooked data from the BaseGyroscope.
 * @note    This data is manipulated according to the formula
 *          cooked = (raw * sensitivity) - bias.
 * @note    Final data is expressed as DPS.
 * @note    The axes array must be at least the same size of the
 *          BaseGyroscope axes number.
 *
 * @param[in] ip        pointer to @p BaseGyroscope interface.
 * @param[out] axes     a buffer which would be filled with cooked data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 */
static msg_t gyro_read_cooked(void *ip, float axes[]) {
  LSM6DSODriver* devp;
  uint32_t i;
  int32_t raw[LSM6DSO_GYRO_NUMBER_OF_AXES];
  msg_t msg;

  osalDbgCheck((ip != NULL) && (axes != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LSM6DSODriver*, (BaseGyroscope*)ip);

  osalDbgAssert((devp->state == LSM6DSO_READY),
                "gyro_read_cooked(), invalid state");

  msg = gyro_read_raw(ip, raw);
  for(i = 0; i < LSM6DSO_GYRO_NUMBER_OF_AXES; i++){
    axes[i] = (raw[i] * devp->gyrosensitivity[i]) - devp->gyrobias[i];
  }
  return msg;
}

/**
 * @brief   Samples bias values for the BaseGyroscope.
 * @note    The LSM6DSO shall not be moved during the whole procedure.
 * @note    After this function internal bias is automatically updated.
 * @note    The behavior of this function depends on @p LSM6DSO_BIAS_ACQ_TIMES
 *          and @p LSM6DSO_BIAS_SETTLING_US.
 *
 * @param[in] ip        pointer to @p BaseGyroscope interface.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t gyro_sample_bias(void *ip) {
  LSM6DSODriver* devp;
  uint32_t i, j;
  int32_t raw[LSM6DSO_GYRO_NUMBER_OF_AXES];
  int32_t buff[LSM6DSO_GYRO_NUMBER_OF_AXES] = {0, 0, 0};
  msg_t msg;
	
  osalDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LSM6DSODriver*, (BaseGyroscope*)ip);

  osalDbgAssert((devp->state == LSM6DSO_READY),
                "gyro_sample_bias(), invalid state");
#if LSM6DSO_USE_I2C
  osalDbgAssert((devp->config->i2cp->state == I2C_READY),
                "gyro_sample_bias(), channel not ready");
#endif

  for(i = 0; i < LSM6DSO_GYRO_BIAS_ACQ_TIMES; i++){
    msg = gyro_read_raw(ip, raw);
		if(msg != MSG_OK)
			return msg;
    for(j = 0; j < LSM6DSO_GYRO_NUMBER_OF_AXES; j++){
      buff[j] += raw[j];
    }
    osalThreadSleepMicroseconds(LSM6DSO_GYRO_BIAS_SETTLING_US);
  }

  for(i = 0; i < LSM6DSO_GYRO_NUMBER_OF_AXES; i++){
    devp->gyrobias[i] = (buff[i] / LSM6DSO_GYRO_BIAS_ACQ_TIMES);
    devp->gyrobias[i] *= devp->gyrosensitivity[i];
  }
  return msg;
}

/**
 * @brief   Set bias values for the BaseGyroscope.
 * @note    Bias must be expressed as DPS.
 * @note    The bias buffer must be at least the same size of the BaseGyroscope
 *          axes number.
 *
 * @param[in] ip        pointer to @p BaseGyroscope interface.
 * @param[in] bp        a buffer which contains biases.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t gyro_set_bias(void *ip, float *bp) {
  LSM6DSODriver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  osalDbgCheck((ip != NULL) && (bp != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LSM6DSODriver*, (BaseGyroscope*)ip);

  osalDbgAssert((devp->state == LSM6DSO_READY),
                "gyro_set_bias(), invalid state");

  for(i = 0; i < LSM6DSO_GYRO_NUMBER_OF_AXES; i++) {
    devp->gyrobias[i] = bp[i];
  }
  return msg;
}

/**
 * @brief   Reset bias values for the BaseGyroscope.
 * @note    Default biases value are obtained from device datasheet when
 *          available otherwise they are considered zero.
 *
 * @param[in] ip        pointer to @p BaseGyroscope interface.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t gyro_reset_bias(void *ip) {
  LSM6DSODriver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  osalDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LSM6DSODriver*, (BaseGyroscope*)ip);

  osalDbgAssert((devp->state == LSM6DSO_READY),
                "gyro_reset_bias(), invalid state");

  for(i = 0; i < LSM6DSO_GYRO_NUMBER_OF_AXES; i++)
    devp->gyrobias[i] = LSM6DSO_GYRO_BIAS;
  return msg;
}

/**
 * @brief   Set sensitivity values for the BaseGyroscope.
 * @note    Sensitivity must be expressed as DPS/LSB.
 * @note    The sensitivity buffer must be at least the same size of the
 *          BaseGyroscope axes number.
 *
 * @param[in] ip        pointer to @p BaseGyroscope interface.
 * @param[in] sp        a buffer which contains sensitivities.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t gyro_set_sensivity(void *ip, float *sp) {
  LSM6DSODriver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  osalDbgCheck((ip != NULL) && (sp !=NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LSM6DSODriver*, (BaseGyroscope*)ip);

  osalDbgAssert((devp->state == LSM6DSO_READY),
                "gyro_set_sensivity(), invalid state");

  for(i = 0; i < LSM6DSO_GYRO_NUMBER_OF_AXES; i++) {
    devp->gyrosensitivity[i] = sp[i];
  }
  return msg;
}

/**
 * @brief   Reset sensitivity values for the BaseGyroscope.
 * @note    Default sensitivities value are obtained from device datasheet.
 *
 * @param[in] ip        pointer to @p BaseGyroscope interface.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 */
static msg_t gyro_reset_sensivity(void *ip) {
  LSM6DSODriver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  osalDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LSM6DSODriver*, (BaseGyroscope*)ip);

  osalDbgAssert((devp->state == LSM6DSO_READY),
                "gyro_reset_sensivity(), invalid state");
  if(devp->config->gyrofullscale == LSM6DSO_GYRO_FS_125DPS)
    for(i = 0; i < LSM6DSO_GYRO_NUMBER_OF_AXES; i++)
      devp->gyrosensitivity[i] = LSM6DSO_GYRO_SENS_125DPS;
  else if(devp->config->gyrofullscale == LSM6DSO_GYRO_FS_250DPS)
    for(i = 0; i < LSM6DSO_GYRO_NUMBER_OF_AXES; i++)
      devp->gyrosensitivity[i] = LSM6DSO_GYRO_SENS_250DPS;
  else if(devp->config->gyrofullscale == LSM6DSO_GYRO_FS_500DPS)
	for(i = 0; i < LSM6DSO_GYRO_NUMBER_OF_AXES; i++)
      devp->gyrosensitivity[i] = LSM6DSO_GYRO_SENS_500DPS;
  else if(devp->config->gyrofullscale == LSM6DSO_GYRO_FS_1000DPS)
	for(i = 0; i < LSM6DSO_GYRO_NUMBER_OF_AXES; i++)
      devp->gyrosensitivity[i] = LSM6DSO_GYRO_SENS_1000DPS;
  else if(devp->config->gyrofullscale == LSM6DSO_GYRO_FS_2000DPS)
	for(i = 0; i < LSM6DSO_GYRO_NUMBER_OF_AXES; i++)
      devp->gyrosensitivity[i] = LSM6DSO_GYRO_SENS_2000DPS;
  else {
    osalDbgAssert(FALSE, "gyro_reset_sensivity(), full scale issue");
    return MSG_RESET;
  }
  return msg;
}

/**
 * @brief   Changes the LSM6DSODriver gyroscope fullscale value.
 * @note    This function also rescale sensitivities and biases based on
 *          previous and next fullscale value.
 * @note    A recalibration is highly suggested after calling this function.
 *
 * @param[in] devp      pointer to @p BaseGyroscope interface.
 * @param[in] fs        new fullscale value.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 */
static msg_t gyro_set_full_scale(LSM6DSODriver *devp, lsm6dso_gyro_fs_t fs) {
  float newfs, scale;
  uint8_t i, buff[2];
  msg_t msg = MSG_OK;

  osalDbgCheck(devp != NULL);

  osalDbgAssert((devp->state == LSM6DSO_READY),
                "gyro_set_full_scale(), invalid state");
#if LSM6DSO_USE_I2C
  osalDbgAssert((devp->config->i2cp->state == I2C_READY),
                "gyro_set_full_scale(), channel not ready");
#endif

  if(fs == LSM6DSO_GYRO_FS_125DPS) {
    newfs = LSM6DSO_GYRO_125DPS;
  }
  else if(fs == LSM6DSO_GYRO_FS_250DPS) {
    newfs = LSM6DSO_GYRO_250DPS;
  }
  else if(fs == LSM6DSO_GYRO_FS_500DPS) {
    newfs = LSM6DSO_GYRO_500DPS;
  }
  else if(fs == LSM6DSO_GYRO_FS_1000DPS) {
    newfs = LSM6DSO_GYRO_1000DPS;
  }
  else if(fs == LSM6DSO_GYRO_FS_2000DPS) {
    newfs = LSM6DSO_GYRO_2000DPS;
  }
  else {
    return MSG_RESET;
  }

  if(newfs != devp->gyrofullscale) {
    scale = newfs / devp->gyrofullscale;
    devp->gyrofullscale = newfs;

#if LSM6DSO_USE_I2C
#if	LSM6DSO_SHARED_I2C
		i2cAcquireBus(devp->config->i2cp);
		i2cStart(devp->config->i2cp,
						 devp->config->i2ccfg);
#endif /* LSM6DSO_SHARED_I2C */

    /* Updating register.*/
    msg = lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_AD_CTRL2_G, &buff[1], 1);

#if	LSM6DSO_SHARED_I2C
		i2cReleaseBus(devp->config->i2cp);
#endif /* LSM6DSO_SHARED_I2C */
#endif /* LSM6DSO_USE_I2C */

    buff[1] &= ~(LSMDSL_CTRL2_G_FS_MASK);
    buff[1] |= fs;
    buff[0] = LSM6DSO_AD_CTRL2_G;

#if LSM6DSO_USE_I2C
#if	LSM6DSO_SHARED_I2C
		i2cAcquireBus(devp->config->i2cp);
		i2cStart(devp->config->i2cp,
						 devp->config->i2ccfg);
#endif /* LSM6DSO_SHARED_I2C */

    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                            buff, 1);

#if	LSM6DSO_SHARED_I2C
		i2cReleaseBus(devp->config->i2cp);
#endif /* LSM6DSO_SHARED_I2C */
#endif /* LSM6DSO_USE_I2C */

    /* Scaling sensitivity and bias. Re-calibration is suggested anyway. */
    for(i = 0; i < LSM6DSO_GYRO_NUMBER_OF_AXES; i++) {
      devp->gyrosensitivity[i] *= scale;
      devp->gyrobias[i] *= scale;
    }
  }
  return msg;
}

static const struct LSM6DSOVMT vmt_device = {
  (size_t)0,
  acc_set_full_scale, gyro_set_full_scale
};

static const struct BaseAccelerometerVMT vmt_accelerometer = {
  sizeof(struct LSM6DSOVMT*),
  acc_get_axes_number, acc_read_raw, acc_read_cooked,
  acc_set_bias, acc_reset_bias, acc_set_sensivity, acc_reset_sensivity
};

static const struct BaseGyroscopeVMT vmt_gyroscope = {
  sizeof(struct LSM6DSOVMT*) + sizeof(BaseAccelerometer),
  gyro_get_axes_number, gyro_read_raw, gyro_read_cooked,
  gyro_sample_bias, gyro_set_bias, gyro_reset_bias,
  gyro_set_sensivity, gyro_reset_sensivity
};

static void bytecpy(uint8_t *target, uint8_t *source)
{
  if ((target != NULL) && (source != NULL))
  {
    *target = *source;
  }
}

void lsm6dsoAllSourcesGet(LSM6DSODriver *devp,
                                lsm6dso_all_sources_t *val)
{
  lsm6dso_emb_func_status_mainpage_t emb_func_status_mainpage;
  lsm6dso_status_master_mainpage_t   status_master_mainpage;
  lsm6dso_fsm_status_a_mainpage_t    fsm_status_a_mainpage;
  lsm6dso_fsm_status_b_mainpage_t    fsm_status_b_mainpage;
  lsm6dso_fifo_status1_t             fifo_status1;
  lsm6dso_fifo_status2_t             fifo_status2;
  lsm6dso_all_int_src_t              all_int_src;
  lsm6dso_wake_up_src_t              wake_up_src;
  lsm6dso_status_reg_t               status_reg;
  lsm6dso_tap_src_t                  tap_src;
  lsm6dso_d6d_src_t                  d6d_src;
  uint8_t                            reg[5];

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_AD_MASTER_CONFIG, (uint8_t*)&reg, 5);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  bytecpy((uint8_t *)&all_int_src, &reg[0]);
  bytecpy((uint8_t *)&wake_up_src, &reg[1]);
  bytecpy((uint8_t *)&tap_src, &reg[2]);
  bytecpy((uint8_t *)&d6d_src, &reg[3]);
  bytecpy((uint8_t *)&status_reg, &reg[4]);
  val->timestamp = all_int_src.timestamp_endcount;
  val->wake_up_z    = wake_up_src.z_wu;
  val->wake_up_y    = wake_up_src.y_wu;
  val->wake_up_x    = wake_up_src.x_wu;
  val->wake_up      = wake_up_src.wu_ia;
  val->sleep_state  = wake_up_src.sleep_state;
  val->free_fall    = wake_up_src.ff_ia;
  val->sleep_change = wake_up_src.sleep_change_ia;
  val->tap_x      = tap_src.x_tap;
  val->tap_y      = tap_src.y_tap;
  val->tap_z      = tap_src.z_tap;
  val->tap_sign   = tap_src.tap_sign;
  val->double_tap = tap_src.double_tap;
  val->single_tap = tap_src.single_tap;
  val->six_d_xl = d6d_src.xl;
  val->six_d_xh = d6d_src.xh;
  val->six_d_yl = d6d_src.yl;
  val->six_d_yh = d6d_src.yh;
  val->six_d_zl = d6d_src.zl;
  val->six_d_zh = d6d_src.zh;
  val->six_d    = d6d_src.d6d_ia;
  val->den_flag = d6d_src.den_drdy;
  val->drdy_xl   = status_reg.xlda;
  val->drdy_g    = status_reg.gda;
  val->drdy_temp = status_reg.tda;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_AD_SENSORHUB8_REG, (uint8_t*)&reg, 3);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  bytecpy((uint8_t *)&emb_func_status_mainpage, &reg[0]);
  bytecpy((uint8_t *)&fsm_status_a_mainpage, &reg[1]);
  bytecpy((uint8_t *)&fsm_status_b_mainpage, &reg[2]);
  val->step_detector = emb_func_status_mainpage.is_step_det;
  val->tilt          = emb_func_status_mainpage.is_tilt;
  val->sig_mot       = emb_func_status_mainpage.is_sigmot;
  val->fsm_lc        = emb_func_status_mainpage.is_fsm_lc;
  val->fsm1 = fsm_status_a_mainpage.is_fsm1;
  val->fsm2 = fsm_status_a_mainpage.is_fsm2;
  val->fsm3 = fsm_status_a_mainpage.is_fsm3;
  val->fsm4 = fsm_status_a_mainpage.is_fsm4;
  val->fsm5 = fsm_status_a_mainpage.is_fsm5;
  val->fsm6 = fsm_status_a_mainpage.is_fsm6;
  val->fsm7 = fsm_status_a_mainpage.is_fsm7;
  val->fsm8 = fsm_status_a_mainpage.is_fsm8;
  val->fsm9  = fsm_status_b_mainpage.is_fsm9;
  val->fsm10 = fsm_status_b_mainpage.is_fsm10;
  val->fsm11 = fsm_status_b_mainpage.is_fsm11;
  val->fsm12 = fsm_status_b_mainpage.is_fsm12;
  val->fsm13 = fsm_status_b_mainpage.is_fsm13;
  val->fsm14 = fsm_status_b_mainpage.is_fsm14;
  val->fsm15 = fsm_status_b_mainpage.is_fsm15;
  val->fsm16 = fsm_status_b_mainpage.is_fsm16;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_AD_SENSORHUB12_REG, (uint8_t*)&reg, 3);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  bytecpy((uint8_t *)&status_master_mainpage, &reg[0]);
  bytecpy((uint8_t *)&fifo_status1, &reg[1]);
  bytecpy((uint8_t *)&fifo_status2, &reg[2]);
  val->sh_endop       = status_master_mainpage.sens_hub_endop;
  val->sh_slave0_nack = status_master_mainpage.slave0_nack;
  val->sh_slave1_nack = status_master_mainpage.slave1_nack;
  val->sh_slave2_nack = status_master_mainpage.slave2_nack;
  val->sh_slave3_nack = status_master_mainpage.slave3_nack;
  val->sh_wr_once     = status_master_mainpage.wr_once_done;
  val->fifo_diff = (256U * fifo_status2.diff_fifo) +
                   fifo_status1.diff_fifo;
  val->fifo_ovr_latched = fifo_status2.over_run_latched;
  val->fifo_bdr         = fifo_status2.counter_bdr_ia;
  val->fifo_full        = fifo_status2.fifo_full_ia;
  val->fifo_ovr         = fifo_status2.fifo_ovr_ia;
  val->fifo_th          = fifo_status2.fifo_wtm_ia;
}

void lsm6dsoMemBankSet(LSM6DSODriver *devp,
                             lsm6dso_reg_access_t val)
{
  lsm6dso_func_cfg_access_t reg = {0};
  uint8_t cr[2];

  /*  no need to read it first as the other bits are reserved and must be zero */
  reg.reg_access = (uint8_t)val;

  cr[0] = LSM6DSO_AD_FUNC_CFG_ACCESS;
  cr[1] = *(uint8_t*)&reg;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                               cr, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */
}

void lsm6dso_ln_pg_write(LSM6DSODriver *devp, uint16_t address,
                            uint8_t *buf, uint8_t len)
{
  lsm6dso_page_rw_t page_rw;
  lsm6dso_page_sel_t page_sel;
  lsm6dso_page_address_t  page_address;
  uint8_t msb;
  uint8_t lsb;
  uint8_t i ;
  uint8_t cr[2];

  msb = ((uint8_t)(address >> 8) & 0x0FU);
  lsb = (uint8_t)address & 0xFFU;

  lsm6dsoMemBankSet(devp, LSM6DSO_EMBEDDED_FUNC_BANK);

  /* set page write */
  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_PAGE_RW, (uint8_t*)&page_rw, 1);
  
  #if LSM6DSO_SHARED_I2C
  i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */
  page_rw.page_rw = 0x02; /* page_write enable*/
  cr[0] = LSM6DSO_PAGE_RW;
  cr[1] = *(uint8_t*)&page_rw;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                               cr, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  /* select page */
  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_PAGE_SEL, (uint8_t*)&page_sel, 1);
  
  #if LSM6DSO_SHARED_I2C
  i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */
  page_sel.page_sel = msb;
  page_sel.not_used_01 = 1;
  cr[0] = LSM6DSO_PAGE_SEL;
  cr[1] = *(uint8_t*)&page_sel;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                               cr, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  /* set page addr */
  page_address.page_addr = lsb;
  cr[0] = LSM6DSO_PAGE_ADDRESS;
  cr[1] = *(uint8_t*)&page_address;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                               cr, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  for (i = 0; i < len; i++)
  {
  cr[0] = LSM6DSO_PAGE_VALUE;
  cr[1] = *(uint8_t*)&buf[i];

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                               cr, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

    lsb++;

    /* Check if page wrap */
    if ((lsb & 0xFFU) == 0x00U)
    {
      msb++;
  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_PAGE_SEL, (uint8_t*)&page_sel, 1);
  
  #if LSM6DSO_SHARED_I2C
  i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

      page_sel.page_sel = msb;
      page_sel.not_used_01 = 1;
  cr[0] = LSM6DSO_PAGE_SEL;
  cr[1] = *(uint8_t*)&page_sel;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                               cr, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */
    }
  }

  page_sel.page_sel = 0;
  page_sel.not_used_01 = 1;
  cr[0] = LSM6DSO_PAGE_SEL;
  cr[1] = *(uint8_t*)&page_sel;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                               cr, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  /* unset page write */
  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_PAGE_RW, (uint8_t*)&page_rw, 1);
  
  #if LSM6DSO_SHARED_I2C
  i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */
  page_rw.page_rw = 0x00; /* page_write disable */
  cr[0] = LSM6DSO_PAGE_RW;
  cr[1] = *(uint8_t*)&page_rw;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                               cr, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

    lsm6dsoMemBankSet(devp, LSM6DSO_USER_BANK);
}

void lsm6dso_ln_pg_write_byte(LSM6DSODriver *devp, uint16_t address, uint8_t *val)
{
  lsm6dso_ln_pg_write(devp, address, val, 1);
}

void lsm6dsoNumberOfStepsGet(LSM6DSODriver *devp, uint16_t *val)
{
  uint8_t buff[2];
  lsm6dsoMemBankSet(devp, LSM6DSO_EMBEDDED_FUNC_BANK);

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_STEP_COUNTER_L, (uint8_t*)&buff, 2);
  
  #if LSM6DSO_SHARED_I2C
  i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  *val = buff[1];
  *val = (*val * 256U) +  buff[0];

  lsm6dsoMemBankSet(devp, LSM6DSO_USER_BANK);
}

void lsm6dsoIntNotificationSet(LSM6DSODriver *devp,
                                     lsm6dso_lir_t val)
{
  lsm6dso_tap_cfg0_t tap_cfg0;
  lsm6dso_page_rw_t page_rw;

  uint8_t cr[2];

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_TAP_CFG0, (uint8_t*)&tap_cfg0, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */
  
  tap_cfg0.lir = (uint8_t)val & 0x01U;
  tap_cfg0.int_clr_on_read = (uint8_t)val & 0x01U;

  cr[0] = LSM6DSO_TAP_CFG0;
  cr[1] = *(uint8_t*)&tap_cfg0;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                               cr, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  lsm6dsoMemBankSet(devp, LSM6DSO_EMBEDDED_FUNC_BANK);

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_AD_CTRL8_XL, (uint8_t*)&page_rw, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  page_rw.emb_func_lir = ((uint8_t)val & 0x02U) >> 1;

  cr[0] = LSM6DSO_AD_CTRL8_XL;
  cr[1] = *(uint8_t*)&page_rw;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                               cr, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  lsm6dsoMemBankSet(devp, LSM6DSO_USER_BANK);
}

void lsm6dsoReset(LSM6DSODriver *devp) {
  lsm6dso_ctrl3_c_t reg;
  uint8_t cr[2];

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_AD_CTRL3_C, (uint8_t*)&reg, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  reg.sw_reset = 1U;

  cr[0] = LSM6DSO_AD_CTRL3_C;
  cr[1] = *(uint8_t*)&reg;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                               cr, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  uint8_t val;

  do {
    lsm6dso_ctrl3_c_t new_reg;

    #if LSM6DSO_USE_I2C
    #if LSM6DSO_SHARED_I2C
      i2cAcquireBus(devp->config->i2cp);
    #endif /* LSM6DSO_SHARED_I2C */

      i2cStart(devp->config->i2cp, devp->config->i2ccfg);
      lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_AD_CTRL3_C, (uint8_t*)&new_reg, 1);

    #if LSM6DSO_SHARED_I2C
      i2cReleaseBus(devp->config->i2cp);
    #endif /* LSM6DSO_SHARED_I2C */
    #endif /* LSM6DSO_USE_I2C */
    val = new_reg.sw_reset;
  }
  while(val);
}

void lsm6dso_ln_pg_read(LSM6DSODriver *devp, uint16_t address, uint8_t *buf,
                           uint8_t len)
{
  lsm6dso_page_rw_t page_rw;
  lsm6dso_page_sel_t page_sel;
  lsm6dso_page_address_t  page_address;
  uint8_t msb;
  uint8_t lsb;
  uint8_t i ;
  uint8_t cr1[2];

  msb = ((uint8_t)(address >> 8) & 0x0FU);
  lsb = (uint8_t)address & 0xFFU;

  lsm6dsoMemBankSet(devp, LSM6DSO_EMBEDDED_FUNC_BANK);

  /* set page write */
  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_PAGE_RW, (uint8_t*)&page_rw, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */
  page_rw.page_rw = 0x01; /* page_read enable*/
  cr1[0] = LSM6DSO_PAGE_RW;
  cr1[1] = *(uint8_t*)&page_rw;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                               cr1, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  /* select page */
  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_PAGE_SEL, (uint8_t*)&page_sel, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */
  page_sel.page_sel = msb;
  page_sel.not_used_01 = 1;
  cr1[0] = LSM6DSO_PAGE_SEL;
  cr1[1] = *(uint8_t*)&page_sel;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                               cr1, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  for (i = 0; i < len; i++)
  {
    /* set page addr */
    page_address.page_addr = lsb;
    cr1[0] = LSM6DSO_PAGE_ADDRESS;
    cr1[1] = *(uint8_t*)&page_address;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                               cr1, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_PAGE_VALUE, (uint8_t*)&buf[i], 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */
    #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_PAGE_VALUE, (uint8_t*)&buf[i], 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

    lsb++;

    /* Check if page wrap */
    if ((lsb & 0xFFU) == 0x00U)
    {
      msb++;
  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_PAGE_SEL, (uint8_t*)&page_sel, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

      page_sel.page_sel = msb;
      page_sel.not_used_01 = 1;
      cr1[0] = LSM6DSO_PAGE_SEL;
      cr1[1] = *(uint8_t*)&page_sel;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                               cr1, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */
    }
  }

  page_sel.page_sel = 0;
  page_sel.not_used_01 = 1;
      cr1[0] = LSM6DSO_PAGE_SEL;
      cr1[1] = *(uint8_t*)&page_sel;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                               cr1, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  /* unset page write */
  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_PAGE_RW, (uint8_t*)&page_rw, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */
  page_rw.page_rw = 0x00; /* page_write disable */
  cr1[0] = LSM6DSO_PAGE_RW;
  cr1[1] = *(uint8_t*)&page_rw;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                               cr1, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  lsm6dsoMemBankSet(devp, LSM6DSO_USER_BANK);
}

void lsm6dso_ln_pg_read_byte(LSM6DSODriver *devp, uint16_t address, uint8_t *val)
{
  lsm6dso_ln_pg_read(devp, address, val, 1);
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initializes an instance.
 *
 * @param[out] devp     pointer to the @p LSM6DSODriver object
 *
 * @init
 */
void lsm6dsoObjectInit(LSM6DSODriver *devp) {
  devp->vmt = &vmt_device;
  devp->acc_if.vmt = &vmt_accelerometer;
  devp->gyro_if.vmt = &vmt_gyroscope;

  devp->config = NULL;

  devp->accaxes = LSM6DSO_ACC_NUMBER_OF_AXES;
  devp->gyroaxes = LSM6DSO_GYRO_NUMBER_OF_AXES;

  devp->state = LSM6DSO_STOP;
}

/**
 * @brief   Configures and activates LSM6DSO Complex Driver peripheral.
 *
 * @param[in] devp      pointer to the @p LSM6DSODriver object
 * @param[in] config    pointer to the @p LSM6DSOConfig object
 *
 * @api
 */
void lsm6dsoStart(LSM6DSODriver *devp, const LSM6DSOConfig *config) {
  uint32_t i;
  uint8_t cr1[2];
  uint8_t cr2[2];
  osalDbgCheck((devp != NULL) && (config != NULL));

  osalDbgAssert((devp->state == LSM6DSO_STOP) ||
                (devp->state == LSM6DSO_READY),
                "lsm6dsoStart(), invalid state");

  devp->config = config;

  lsm6dsoReset(devp);

  // start register configuration

  // disable i3c

  lsm6dso_i3c_bus_avb_t i3c_bus_avb;
  lsm6dso_ctrl9_xl_t ctrl9_xl;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_AD_CTRL9_XL, (uint8_t*)&ctrl9_xl, 1);
    lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_I3C_BUS_AVB, (uint8_t*)&i3c_bus_avb, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  ctrl9_xl.i3c_disable = ((uint8_t)(0x80) & 0x80U) >> 7;
  i3c_bus_avb.i3c_bus_avb_sel = (uint8_t)(0x80) & 0x03U;

  cr1[0] = LSM6DSO_AD_CTRL9_XL;
  cr1[1] = *(uint8_t*)&ctrl9_xl;

  cr2[0] = LSM6DSO_I3C_BUS_AVB;
  cr2[1] = *(uint8_t*)&i3c_bus_avb;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                               cr1, 1);
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                               cr2, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  // enable bdu

  lsm6dso_ctrl3_c_t reg3;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_AD_CTRL3_C, (uint8_t*)&reg3, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  reg3.bdu = 1U;

  cr1[0] = LSM6DSO_AD_CTRL3_C;
  cr1[1] = *(uint8_t*)&reg3;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                               cr1, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  // setting accelerometer full scale and odr

  lsm6dso_ctrl1_xl_t reg1;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_AD_CTRL1_XL, (uint8_t*)&reg1, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  reg1.odr_xl = (uint8_t) config->accoutdatarate;
  reg1.fs_xl = (uint8_t) config->accfullscale;

  cr1[0] = LSM6DSO_AD_CTRL1_XL;
  cr1[1] = *(uint8_t*)&reg1;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                               cr1, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  // setting gyroscope full scale and odr

  lsm6dso_ctrl2_g_t reg2;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_AD_CTRL2_G, (uint8_t*)&reg2, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  reg2.odr_g = (uint8_t) config->gyrooutdatarate;
  reg2.fs_g = (uint8_t) config->gyrofullscale;

  cr1[0] = LSM6DSO_AD_CTRL2_G;
  cr1[1] = *(uint8_t*)&reg2;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                               cr1, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  #if LSM6DSO_USE_FREE_FALL || defined(__DOXYGEN__)
  
  // setting free fall parameters

  lsm6dso_wake_up_dur_t wake_up_dur;
  lsm6dso_free_fall_t free_fall;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_AD_WAKE_UP_DUR, (uint8_t*)&wake_up_dur, 1);
    lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_AD_FREE_FALL, (uint8_t*)&free_fall, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  wake_up_dur.ff_dur = (((uint8_t)config->freefallduration) & 0x20U) >> 5;
  free_fall.ff_dur = ((uint8_t)config->freefallduration) & 0x1FU;
  free_fall.ff_ths = ((uint8_t)config->freefallthreshold);

  cr1[0] = LSM6DSO_AD_WAKE_UP_DUR;
  cr1[1] = *(uint8_t*)&wake_up_dur;

  cr2[0] = LSM6DSO_AD_FREE_FALL;
  cr2[1] = *(uint8_t*)&free_fall;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                               cr1, 1);
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                               cr2, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */



  #endif /* LSM6DSO_USE_FREE_FALL */

  #if LSM6DSO_USE_INTERRUPT || defined(__DOXYGEN__)

  // enable interrupts

  lsm6dso_tap_cfg2_t tap_cfg_2;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_AD_TAP_CFG, (uint8_t*)&tap_cfg_2, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  tap_cfg_2.interrupts_enable = 1U;

  cr1[0] = LSM6DSO_AD_TAP_CFG;
  cr1[1] = *(uint8_t*)&tap_cfg_2;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                               cr1, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */
  
  lsm6dsoIntNotificationSet(devp,config->lirvalue);

  #endif /* LSM6DSO_USE_INTERRUPT */

  #if LSM6DSO_USE_ORIENTATION || defined(__DOXYGEN__)
  
  // setting 6D orientation parameters.

  lsm6dso_tap_ths_6d_t reg_6d;
  lsm6dso_ctrl8_xl_t reg8;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_AD_TAP_THS_6D, (uint8_t*)&reg_6d, 1);
    lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_AD_CTRL8_XL, (uint8_t*)&reg8, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  reg8.low_pass_on_6d = devp->config->sixdlowpassenable;
  reg_6d.sixd_ths = devp->config->sixdthreshold;
  reg_6d.d4d_en = devp->config->sixd4dmodeenable;

  cr1[0] = LSM6DSO_AD_TAP_THS_6D;
  cr1[1] = *(uint8_t*)&reg_6d;

  cr2[0] = LSM6DSO_AD_CTRL8_XL;
  cr2[1] = *(uint8_t*)&reg8;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                               cr1, 1);
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                               cr2, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */



  #endif /* LSM6DSO_USE_ORIENTATION */

    #if LSM6DSO_USE_PEDOMETER || defined(__DOXYGEN__)
  
  // setting 6D orientation parameters.

  lsm6dso_emb_func_src_t pedo_src_reg;
  lsm6dso_pedo_cmd_reg_t pedo_cmd_reg;
  lsm6dso_emb_func_en_a_t emb_func_en_a;
  lsm6dso_emb_func_en_b_t emb_func_en_b;
  lsm6dso_emb_sens_t emb_sens = {0,0,0,0,0,0};

  lsm6dsoMemBankSet(devp, LSM6DSO_EMBEDDED_FUNC_BANK);

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_EMB_FUNC_SRC, (uint8_t*)&pedo_src_reg, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  pedo_src_reg.pedo_rst_step = 1U;

  cr1[0] = LSM6DSO_EMB_FUNC_SRC;
  cr1[1] = *(uint8_t*)&pedo_src_reg;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                               cr1, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  lsm6dsoMemBankSet(devp, LSM6DSO_USER_BANK);

  lsm6dso_ln_pg_read_byte(devp, LSM6DSO_PEDO_CMD_REG,
                                (uint8_t *)&pedo_cmd_reg);

  pedo_cmd_reg.fp_rejection_en = ((uint8_t)devp->config->pedomode & 0x10U) >> 4;
  pedo_cmd_reg.ad_det_en = ((uint8_t)devp->config->pedomode & 0x20U) >> 5;

  lsm6dso_ln_pg_write_byte(devp, LSM6DSO_PEDO_CMD_REG,
                                   (uint8_t *)&pedo_cmd_reg);

  emb_sens.step = 1U;
  emb_sens.step_adv = 1U;

  lsm6dsoMemBankSet(devp, LSM6DSO_EMBEDDED_FUNC_BANK);

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_EMB_FUNC_EN_A, (uint8_t*)&emb_func_en_a, 1);
    lsm6dsoI2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DSO_EMB_FUNC_EN_B, (uint8_t*)&emb_func_en_b, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  emb_func_en_b.fsm_en = emb_sens.fsm;
  emb_func_en_a.tilt_en = emb_sens.tilt;
  emb_func_en_a.pedo_en = emb_sens.step;
  emb_func_en_b.pedo_adv_en = emb_sens.step_adv;
  emb_func_en_a.sign_motion_en = emb_sens.sig_mot;
  emb_func_en_b.fifo_compr_en = emb_sens.fifo_compr;

  cr1[0] = LSM6DSO_EMB_FUNC_EN_A;
  cr1[1] = *(uint8_t*)&emb_func_en_a;

  cr2[0] = LSM6DSO_EMB_FUNC_EN_B;
  cr2[1] = *(uint8_t*)&emb_func_en_b;

  #if LSM6DSO_USE_I2C
  #if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                               cr1, 1);
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                               cr2, 1);
  
  #if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
  #endif /* LSM6DSO_SHARED_I2C */
  #endif /* LSM6DSO_USE_I2C */

  lsm6dsoMemBankSet(devp, LSM6DSO_USER_BANK);

  #endif /* LSM6DSO_USE_PEDOMETER */

  /* Storing sensitivity according to user settings */
  if(devp->config->accfullscale == LSM6DSO_ACC_FS_2G) {
    for(i = 0; i < LSM6DSO_ACC_NUMBER_OF_AXES; i++) {
     if(devp->config->accsensitivity == NULL)
       devp->accsensitivity[i] = LSM6DSO_ACC_SENS_2G;
     else
       devp->accsensitivity[i] = devp->config->accsensitivity[i];
    }
    devp->accfullscale = LSM6DSO_ACC_2G;
  }
  else if(devp->config->accfullscale == LSM6DSO_ACC_FS_4G) {
   for(i = 0; i < LSM6DSO_ACC_NUMBER_OF_AXES; i++) {
     if(devp->config->accsensitivity == NULL)
       devp->accsensitivity[i] = LSM6DSO_ACC_SENS_4G;
     else
       devp->accsensitivity[i] = devp->config->accsensitivity[i];
    }
   devp->accfullscale = LSM6DSO_ACC_4G;
  }
  else if(devp->config->accfullscale == LSM6DSO_ACC_FS_8G) {
   for(i = 0; i < LSM6DSO_ACC_NUMBER_OF_AXES; i++) {
     if(devp->config->accsensitivity == NULL)
       devp->accsensitivity[i] = LSM6DSO_ACC_SENS_8G;
     else
       devp->accsensitivity[i] = devp->config->accsensitivity[i];
    }
   devp->accfullscale = LSM6DSO_ACC_8G;
  }
  else if(devp->config->accfullscale == LSM6DSO_ACC_FS_16G) {
    for(i = 0; i < LSM6DSO_ACC_NUMBER_OF_AXES; i++) {
      if(devp->config->accsensitivity == NULL)
        devp->accsensitivity[i] = LSM6DSO_ACC_SENS_16G;
      else
        devp->accsensitivity[i] = devp->config->accsensitivity[i];
    }
    devp->accfullscale = LSM6DSO_ACC_16G;
  }
  else
    osalDbgAssert(FALSE, "lsm6dsoStart(), accelerometer full scale issue");

  /* Storing bias information */
  if(devp->config->accbias != NULL)
    for(i = 0; i < LSM6DSO_ACC_NUMBER_OF_AXES; i++)
      devp->accbias[i] = devp->config->accbias[i];
  else
    for(i = 0; i < LSM6DSO_ACC_NUMBER_OF_AXES; i++)
      devp->accbias[i] = LSM6DSO_ACC_BIAS;

  if(devp->config->gyrofullscale == LSM6DSO_GYRO_FS_125DPS) {
    for(i = 0; i < LSM6DSO_GYRO_NUMBER_OF_AXES; i++) {
      if(devp->config->gyrosensitivity == NULL)
        devp->gyrosensitivity[i] = LSM6DSO_GYRO_SENS_125DPS;
      else
        devp->gyrosensitivity[i] = devp->config->gyrosensitivity[i];
    }
    devp->gyrofullscale = LSM6DSO_GYRO_125DPS;
  }
  else if(devp->config->gyrofullscale == LSM6DSO_GYRO_FS_250DPS) {
    for(i = 0; i < LSM6DSO_GYRO_NUMBER_OF_AXES; i++) {
      if(devp->config->gyrosensitivity == NULL)
        devp->gyrosensitivity[i] = LSM6DSO_GYRO_SENS_250DPS;
      else
        devp->gyrosensitivity[i] = devp->config->gyrosensitivity[i];
    }
    devp->gyrofullscale = LSM6DSO_GYRO_250DPS;
  }
  else if(devp->config->gyrofullscale == LSM6DSO_GYRO_FS_500DPS) {
    for(i = 0; i < LSM6DSO_GYRO_NUMBER_OF_AXES; i++) {
      if(devp->config->gyrosensitivity == NULL)
        devp->gyrosensitivity[i] = LSM6DSO_GYRO_SENS_500DPS;
      else
        devp->gyrosensitivity[i] = devp->config->gyrosensitivity[i];
    }
    devp->gyrofullscale = LSM6DSO_GYRO_500DPS;
  }
  else if(devp->config->gyrofullscale == LSM6DSO_GYRO_FS_1000DPS) {
    for(i = 0; i < LSM6DSO_GYRO_NUMBER_OF_AXES; i++) {
      if(devp->config->gyrosensitivity == NULL)
        devp->gyrosensitivity[i] = LSM6DSO_GYRO_SENS_1000DPS;
      else
        devp->gyrosensitivity[i] = devp->config->gyrosensitivity[i];
    }
    devp->gyrofullscale = LSM6DSO_GYRO_1000DPS;
  }
  else if(devp->config->gyrofullscale == LSM6DSO_GYRO_FS_2000DPS) {
    for(i = 0; i < LSM6DSO_GYRO_NUMBER_OF_AXES; i++) {
      if(devp->config->gyrosensitivity == NULL)
        devp->gyrosensitivity[i] = LSM6DSO_GYRO_SENS_2000DPS;
      else
        devp->gyrosensitivity[i] = devp->config->gyrosensitivity[i];
    }
    devp->gyrofullscale = LSM6DSO_GYRO_2000DPS;
  }
  else
    osalDbgAssert(FALSE, "lsm6dsoStart(), gyroscope full scale issue");

  /* Storing bias information */
  if(devp->config->gyrobias != NULL)
    for(i = 0; i < LSM6DSO_GYRO_NUMBER_OF_AXES; i++)
      devp->gyrobias[i] = devp->config->gyrobias[i];
  else
    for(i = 0; i < LSM6DSO_GYRO_NUMBER_OF_AXES; i++)
      devp->gyrobias[i] = LSM6DSO_GYRO_BIAS;

  /* This is the MEMS transient recovery time */
  osalThreadSleepMilliseconds(5);

  devp->state = LSM6DSO_READY;
}

/**
 * @brief   Deactivates the LSM6DSO Complex Driver peripheral.
 *
 * @param[in] devp       pointer to the @p LSM6DSODriver object
 *
 * @api
 */
void lsm6dsoStop(LSM6DSODriver *devp) {
  uint8_t cr[2];

  osalDbgCheck(devp != NULL);

  osalDbgAssert((devp->state == LSM6DSO_STOP) || (devp->state == LSM6DSO_READY),
                "lsm6dsoStop(), invalid state");

  if (devp->state == LSM6DSO_READY) {
#if LSM6DSO_USE_I2C
#if LSM6DSO_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
#endif /* LSM6DSO_SHARED_I2C */


    cr[0] = LSM6DSO_AD_CTRL1_XL;
    /* Disabling accelerometer.*/
    cr[1] = LSM6DSO_ACC_ODR_PD;
    /* Disabling gyroscope.*/
    cr[2] = LSM6DSO_GYRO_ODR_PD;
    lsm6dsoI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                            cr, 2);

    i2cStop(devp->config->i2cp);
#if LSM6DSO_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
#endif /* LSM6DSO_SHARED_I2C */
#endif /* LSM6DSO_USE_I2C */
  }
  devp->state = LSM6DSO_STOP;
}
/** @} */
