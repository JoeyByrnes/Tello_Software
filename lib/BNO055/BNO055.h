/***************************************************************************
  This is a library for the BNO055 orientation sensor
  Designed specifically to work with the Adafruit BNO055 Breakout.
  Pick one up today in the adafruit shop!

  ------> http://www.adafruit.com/products

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by KTOWN for Adafruit Industries.
  Modified for operation on a linux platform by William Gerhard

  MIT license, all text above must be included in any redistribution
 ***************************************************************************/

#ifndef _BNO055_H
#define _BNO055_H
#include <inttypes.h>
#include "I2C.h"
#include "imumaths.h"

// Register Definitions

#define BNO055_ADDRESS_A (0x28)
#define BNO055_ADDRESS_B (0x29)
#define BNO055_ID        (0xA0)

#define NUM_BNO055_OFFSET_REGISTERS (22)

typedef struct
{
    uint16_t accel_offset_x;
    uint16_t accel_offset_y;
    uint16_t accel_offset_z;
    uint16_t gyro_offset_x;
    uint16_t gyro_offset_y;
    uint16_t gyro_offset_z;
    uint16_t mag_offset_x;
    uint16_t mag_offset_y;
    uint16_t mag_offset_z;

    uint16_t accel_radius;
    uint16_t mag_radius;
} bno055_offsets_t;

//! Main class that exports features for BNO055 chip
class BNO055 {
public:

  typedef enum
  {
    BNO055_ACCEL_CONFIG_ADDR = 0x08
  } page1_reg_t;

  typedef enum
  {
    BNO055_ACCEL_CONFIG_2G = 2,
    BNO055_ACCEL_CONFIG_4G = 4,
    BNO055_ACCEL_CONFIG_8G = 8,
    BNO055_ACCEL_CONFIG_16G = 16,

  } accel_config_G;

    typedef enum
  {
    BNO055_ACCEL_CONFIG_8Hz    = 8,
    BNO055_ACCEL_CONFIG_16Hz   = 16,
    BNO055_ACCEL_CONFIG_32Hz   = 32,
    BNO055_ACCEL_CONFIG_64Hz   = 64,
    BNO055_ACCEL_CONFIG_125Hz  = 125,
    BNO055_ACCEL_CONFIG_250Hz  = 250,
    BNO055_ACCEL_CONFIG_500Hz  = 500,
    BNO055_ACCEL_CONFIG_1000Hz = 1000,

  } accel_config_Hz;

 typedef enum
    {
      /* Page id register definition */
      BNO055_PAGE_ID_ADDR                                     = 0X07,

      /* PAGE0 REGISTER DEFINITION START*/
      BNO055_CHIP_ID_ADDR                                     = 0x00,
      BNO055_ACCEL_REV_ID_ADDR                                = 0x01,
      BNO055_MAG_REV_ID_ADDR                                  = 0x02,
      BNO055_GYRO_REV_ID_ADDR                                 = 0x03,
      BNO055_SW_REV_ID_LSB_ADDR                               = 0x04,
      BNO055_SW_REV_ID_MSB_ADDR                               = 0x05,
      BNO055_BL_REV_ID_ADDR                                   = 0X06,

      /* Accel data register */
      BNO055_ACCEL_DATA_X_LSB_ADDR                            = 0X08,
      BNO055_ACCEL_DATA_X_MSB_ADDR                            = 0X09,
      BNO055_ACCEL_DATA_Y_LSB_ADDR                            = 0X0A,
      BNO055_ACCEL_DATA_Y_MSB_ADDR                            = 0X0B,
      BNO055_ACCEL_DATA_Z_LSB_ADDR                            = 0X0C,
      BNO055_ACCEL_DATA_Z_MSB_ADDR                            = 0X0D,

      /* Mag data register */
      BNO055_MAG_DATA_X_LSB_ADDR                              = 0X0E,
      BNO055_MAG_DATA_X_MSB_ADDR                              = 0X0F,
      BNO055_MAG_DATA_Y_LSB_ADDR                              = 0X10,
      BNO055_MAG_DATA_Y_MSB_ADDR                              = 0X11,
      BNO055_MAG_DATA_Z_LSB_ADDR                              = 0X12,
      BNO055_MAG_DATA_Z_MSB_ADDR                              = 0X13,

      /* Gyro data registers */
      BNO055_GYRO_DATA_X_LSB_ADDR                             = 0X14,
      BNO055_GYRO_DATA_X_MSB_ADDR                             = 0X15,
      BNO055_GYRO_DATA_Y_LSB_ADDR                             = 0X16,
      BNO055_GYRO_DATA_Y_MSB_ADDR                             = 0X17,
      BNO055_GYRO_DATA_Z_LSB_ADDR                             = 0X18,
      BNO055_GYRO_DATA_Z_MSB_ADDR                             = 0X19,

      /* Euler data registers */
      BNO055_EULER_H_LSB_ADDR                                 = 0X1A,
      BNO055_EULER_H_MSB_ADDR                                 = 0X1B,
      BNO055_EULER_R_LSB_ADDR                                 = 0X1C,
      BNO055_EULER_R_MSB_ADDR                                 = 0X1D,
      BNO055_EULER_P_LSB_ADDR                                 = 0X1E,
      BNO055_EULER_P_MSB_ADDR                                 = 0X1F,

      /* Quaternion data registers */
      BNO055_QUATERNION_DATA_W_LSB_ADDR                       = 0X20,
      BNO055_QUATERNION_DATA_W_MSB_ADDR                       = 0X21,
      BNO055_QUATERNION_DATA_X_LSB_ADDR                       = 0X22,
      BNO055_QUATERNION_DATA_X_MSB_ADDR                       = 0X23,
      BNO055_QUATERNION_DATA_Y_LSB_ADDR                       = 0X24,
      BNO055_QUATERNION_DATA_Y_MSB_ADDR                       = 0X25,
      BNO055_QUATERNION_DATA_Z_LSB_ADDR                       = 0X26,
      BNO055_QUATERNION_DATA_Z_MSB_ADDR                       = 0X27,

      /* Linear acceleration data registers */
      BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR                     = 0X28,
      BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR                     = 0X29,
      BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR                     = 0X2A,
      BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR                     = 0X2B,
      BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR                     = 0X2C,
      BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR                     = 0X2D,

      /* Gravity data registers */
      BNO055_GRAVITY_DATA_X_LSB_ADDR                          = 0X2E,
      BNO055_GRAVITY_DATA_X_MSB_ADDR                          = 0X2F,
      BNO055_GRAVITY_DATA_Y_LSB_ADDR                          = 0X30,
      BNO055_GRAVITY_DATA_Y_MSB_ADDR                          = 0X31,
      BNO055_GRAVITY_DATA_Z_LSB_ADDR                          = 0X32,
      BNO055_GRAVITY_DATA_Z_MSB_ADDR                          = 0X33,

      /* Temperature data register */
      BNO055_TEMP_ADDR                                        = 0X34,

      /* Status registers */
      BNO055_CALIB_STAT_ADDR                                  = 0X35,
      BNO055_SELFTEST_RESULT_ADDR                             = 0X36,
      BNO055_INTR_STAT_ADDR                                   = 0X37,

      BNO055_SYS_CLK_STAT_ADDR                                = 0X38,
      BNO055_SYS_STAT_ADDR                                    = 0X39,
      BNO055_SYS_ERR_ADDR                                     = 0X3A,

      /* Unit selection register */
      BNO055_UNIT_SEL_ADDR                                    = 0X3B,
      BNO055_DATA_SELECT_ADDR                                 = 0X3C,

      /* Mode registers */
      BNO055_OPR_MODE_ADDR                                    = 0X3D,
      BNO055_PWR_MODE_ADDR                                    = 0X3E,

      BNO055_SYS_TRIGGER_ADDR                                 = 0X3F,
      BNO055_TEMP_SOURCE_ADDR                                 = 0X40,

      /* Axis remap registers */
      BNO055_AXIS_MAP_CONFIG_ADDR                             = 0X41,
      BNO055_AXIS_MAP_SIGN_ADDR                               = 0X42,

      /* SIC registers */
      BNO055_SIC_MATRIX_0_LSB_ADDR                            = 0X43,
      BNO055_SIC_MATRIX_0_MSB_ADDR                            = 0X44,
      BNO055_SIC_MATRIX_1_LSB_ADDR                            = 0X45,
      BNO055_SIC_MATRIX_1_MSB_ADDR                            = 0X46,
      BNO055_SIC_MATRIX_2_LSB_ADDR                            = 0X47,
      BNO055_SIC_MATRIX_2_MSB_ADDR                            = 0X48,
      BNO055_SIC_MATRIX_3_LSB_ADDR                            = 0X49,
      BNO055_SIC_MATRIX_3_MSB_ADDR                            = 0X4A,
      BNO055_SIC_MATRIX_4_LSB_ADDR                            = 0X4B,
      BNO055_SIC_MATRIX_4_MSB_ADDR                            = 0X4C,
      BNO055_SIC_MATRIX_5_LSB_ADDR                            = 0X4D,
      BNO055_SIC_MATRIX_5_MSB_ADDR                            = 0X4E,
      BNO055_SIC_MATRIX_6_LSB_ADDR                            = 0X4F,
      BNO055_SIC_MATRIX_6_MSB_ADDR                            = 0X50,
      BNO055_SIC_MATRIX_7_LSB_ADDR                            = 0X51,
      BNO055_SIC_MATRIX_7_MSB_ADDR                            = 0X52,
      BNO055_SIC_MATRIX_8_LSB_ADDR                            = 0X53,
      BNO055_SIC_MATRIX_8_MSB_ADDR                            = 0X54,

      /* Accelerometer Offset registers */
      ACCEL_OFFSET_X_LSB_ADDR                                 = 0X55,
      ACCEL_OFFSET_X_MSB_ADDR                                 = 0X56,
      ACCEL_OFFSET_Y_LSB_ADDR                                 = 0X57,
      ACCEL_OFFSET_Y_MSB_ADDR                                 = 0X58,
      ACCEL_OFFSET_Z_LSB_ADDR                                 = 0X59,
      ACCEL_OFFSET_Z_MSB_ADDR                                 = 0X5A,

      /* Magnetometer Offset registers */
      MAG_OFFSET_X_LSB_ADDR                                   = 0X5B,
      MAG_OFFSET_X_MSB_ADDR                                   = 0X5C,
      MAG_OFFSET_Y_LSB_ADDR                                   = 0X5D,
      MAG_OFFSET_Y_MSB_ADDR                                   = 0X5E,
      MAG_OFFSET_Z_LSB_ADDR                                   = 0X5F,
      MAG_OFFSET_Z_MSB_ADDR                                   = 0X60,

      /* Gyroscope Offset register s*/
      GYRO_OFFSET_X_LSB_ADDR                                  = 0X61,
      GYRO_OFFSET_X_MSB_ADDR                                  = 0X62,
      GYRO_OFFSET_Y_LSB_ADDR                                  = 0X63,
      GYRO_OFFSET_Y_MSB_ADDR                                  = 0X64,
      GYRO_OFFSET_Z_LSB_ADDR                                  = 0X65,
      GYRO_OFFSET_Z_MSB_ADDR                                  = 0X66,

      /* Radius registers */
      ACCEL_RADIUS_LSB_ADDR                                   = 0X67,
      ACCEL_RADIUS_MSB_ADDR                                   = 0X68,
      MAG_RADIUS_LSB_ADDR                                     = 0X69,
      MAG_RADIUS_MSB_ADDR                                     = 0X6A
    } bno055_reg_t;

    typedef enum
    {
      POWER_MODE_NORMAL                                       = 0X00,
      POWER_MODE_LOWPOWER                                     = 0X01,
      POWER_MODE_SUSPEND                                      = 0X02
    } bno055_powermode_t;

    typedef enum
    {
      /* Operation mode settings*/
      OPERATION_MODE_CONFIG                                   = 0X00,
      OPERATION_MODE_ACCONLY                                  = 0X01,
      OPERATION_MODE_MAGONLY                                  = 0X02,
      OPERATION_MODE_GYRONLY                                  = 0X03,
      OPERATION_MODE_ACCMAG                                   = 0X04,
      OPERATION_MODE_ACCGYRO                                  = 0X05,
      OPERATION_MODE_MAGGYRO                                  = 0X06,
      OPERATION_MODE_AMG                                      = 0X07,
      OPERATION_MODE_IMUPLUS                                  = 0X08,
      OPERATION_MODE_COMPASS                                  = 0X09,
      OPERATION_MODE_M4G                                      = 0X0A,
      OPERATION_MODE_NDOF_FMC_OFF                             = 0X0B,
      OPERATION_MODE_NDOF                                     = 0X0C
    } bno055_opmode_t;

    typedef enum
    {
      REMAP_CONFIG_P0                                         = 0x21,
      REMAP_CONFIG_P1                                         = 0x24, // default
      REMAP_CONFIG_P2                                         = 0x24,
      REMAP_CONFIG_P3                                         = 0x21,
      REMAP_CONFIG_P4                                         = 0x24,
      REMAP_CONFIG_P5                                         = 0x21,
      REMAP_CONFIG_P6                                         = 0x21,
      REMAP_CONFIG_P7                                         = 0x24
    } bno055_axis_remap_config_t;

    // typedef enum
    // {
    //   ROTATION_ZYX_TO_ZYX           = 0b00100100,
    //   ROTATION_ZYX_TO_YZX           = 0b00011000,
    //   ROTATION_ZYX_TO_XYZ           = 0b00000110,
    //   ROTATION_ZYX_TO_ZXY           = 0b00100001,
    //   ROTATION_ZYX_TO_YXZ           = 0b00010010,
    //   ROTATION_ZYX_TO_XZY           = 0b00000110
    // } bno055_rotate_config_t;

    // typedef enum
    // {
    //   REMAP_SIGN_XP_YP_ZP                                           = 0b00000000,
    //   REMAP_SIGN_XP_YP_ZN                                           = 0b00000001, // default
    //   REMAP_SIGN_XP_YN_ZP                                           = 0b00000010,
    //   REMAP_SIGN_XP_YN_ZN                                           = 0b00000011,
    //   REMAP_SIGN_XN_YP_ZP                                           = 0b00000100,
    //   REMAP_SIGN_XN_YP_ZN                                           = 0b00000101,
    //   REMAP_SIGN_XN_YN_ZP                                           = 0b00000110,
    //   REMAP_SIGN_XN_YN_ZN                                           = 0b00000111
    // } bno055_axis_remap_sign_t;

    typedef enum
    {
      REMAP_SIGN_P0                                           = 0x04,
      REMAP_SIGN_P1                                           = 0x00, // default
      REMAP_SIGN_P2                                           = 0x06,
      REMAP_SIGN_P3                                           = 0x02,
      REMAP_SIGN_P4                                           = 0x03,
      REMAP_SIGN_P5                                           = 0x01,
      REMAP_SIGN_P6                                           = 0x07,
      REMAP_SIGN_P7                                           = 0x05
    } bno055_axis_remap_sign_t;


    typedef struct
    {
      uint8_t  accel_rev;
      uint8_t  mag_rev;
      uint8_t  gyro_rev;
      uint16_t sw_rev;
      uint8_t  bl_rev;
    } bno055_rev_info_t;

    typedef enum
    {
      VECTOR_ACCELEROMETER = BNO055_ACCEL_DATA_X_LSB_ADDR,
      VECTOR_MAGNETOMETER  = BNO055_MAG_DATA_X_LSB_ADDR,
      VECTOR_GYROSCOPE     = BNO055_GYRO_DATA_X_LSB_ADDR,
      VECTOR_EULER         = BNO055_EULER_H_LSB_ADDR,
      VECTOR_LINEARACCEL   = BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR,
      VECTOR_GRAVITY       = BNO055_GRAVITY_DATA_X_LSB_ADDR
    } vector_type_t;



	BNO055(int32_t, int,int);
	virtual ~BNO055();


    bool  begin               ( bno055_opmode_t mode = OPERATION_MODE_NDOF );
    void  setMode             ( bno055_opmode_t mode );
    void  getRevInfo          ( bno055_rev_info_t* );
    void  displayRevInfo      ( void );
    void  setExtCrystalUse    ( bool usextal );
    void  getSystemStatus     ( uint8_t *system_status,
                                uint8_t *self_test_result,
                                uint8_t *system_error);
    void  displaySystemStatus ( void );
    void  getCalibration      ( uint8_t* system, uint8_t* gyro, uint8_t* accel, uint8_t* mag);

    Eigen::VectorXd getVector ( vector_type_t vector_type );
    imu::Quaternion getQuat   ( void );
    int8_t          getTemp   ( void );


/* Functions to deal with raw calibration data */
    bool  getSensorOffsets(uint8_t* calibData);
    bool  getSensorOffsets(bno055_offsets_t &offsets_type);
    void  setSensorOffsets(const uint8_t* calibData);
    void  setSensorOffsets(const bno055_offsets_t &offsets_type);
    bool  isFullyCalibrated(void);


    void setAxisRemap( bno055_axis_remap_config_t remapcode);
    void setAxisSign(bno055_axis_remap_sign_t remapsign);

    void setActivePage(uint8_t page);
    int  getActivePage();

    void setAccel_16G_1000Hz();

    void setAccelConfig(accel_config_G G, accel_config_Hz Hz);
	

private:
	I2C *i2c;
	void reset(void);

    int _address;
    int32_t _sensorID;
    bno055_opmode_t _mode;


};
#endif

