#ifndef mpu6050_H
#define mpu6050_H

#include "my_I2C.h"
#include "esp_rom_sys.h"
#define MPU6050_ADDRESS 0x68 // I2C address for the MPU6050

/*
Registers of interest on the MPU6050 -- see datasheet.
Many values such as X/Y/Z acceleration or gyro are split into high/low bytes.
For example, 0x3B holds X-out high bits, 0x3C holds X-out low bits.
*/
#define MPU6050_SMPLRT_DIV_REG     0x19
#define MPU6050_CONFIGURATION_REG  0x1A
#define MPU6050_GYRO_CONFIG_REG    0x1B
#define MPU6050_ACCEL_CONFIG_REG   0x1C

#define MPU6050_ACCEL_X_OUT_REG    0x3B
#define MPU6050_ACCEL_Y_OUT_REG    0x3D
#define MPU6050_ACCEL_Z_OUT_REG    0x3F
#define MPU6050_TEMP_OUT_REG       0x41
#define MPU6050_GYRO_X_OUT_REG     0x43
#define MPU6050_GYRO_Y_OUT_REG     0x45
#define MPU6050_GYRO_Z_OUT_REG     0x47

#define MPU6050_PWR_MGMT_1_REG     0x6B
#define MPU6050_PWR_MGMT_2_REG     0x6C

typedef int16_t mpu6050_raw_data;

/**
 * Gyroscope full-scale range options in degrees per second.
 */
typedef enum {
    MPU6050_RANGE_250_DEG  = 0,
    MPU6050_RANGE_500_DEG  = 1,
    MPU6050_RANGE_1000_DEG = 2,
    MPU6050_RANGE_2000_DEG = 3
} MPU6050_GYROSCOPE_RANGE;

/**
 * Accelerometer full-scale range options in g (1 g ≈ 9.81 m/s²).
 */
typedef enum {
    MPU6050_RANGE_2_G  = 0,
    MPU6050_RANGE_4_G  = 1,
    MPU6050_RANGE_8_G  = 2,
    MPU6050_RANGE_16_G = 3
} MPU6050_ACCELEROMETER_RANGE;

/**
 * Digital low-pass filter (DLPF) settings.
 * These correspond to gyro cutoff frequencies (accelerometer is slightly lower).
 * Settings 0 and 7 are treated as "disabled."
 */
typedef enum {
    MPU6050_DLPF_260_HZ = 0,
    MPU6050_DLPF_184_HZ = 1,
    MPU6050_DLPF_94_HZ  = 2,
    MPU6050_DLPF_44_HZ  = 3,
    MPU6050_DLPF_21_HZ  = 4,
    MPU6050_DLPF_10_HZ  = 5,
    MPU6050_DLPF_5_HZ   = 6,
    MPU6050_DLPF_DISABLED = 7
} MPU6050_DLPF_FREQ;

/* Current configuration trackers — updated when setters are called. */
extern MPU6050_GYROSCOPE_RANGE current_gyro_range;
extern MPU6050_ACCELEROMETER_RANGE current_accel_range;
extern MPU6050_DLPF_FREQ current_DLPF_val;

/**
 * Convenience struct for storing X/Y/Z sensor data in floating-point form.
 */
typedef struct {
    float x;
    float y;
    float z;
} mpu6050_xyz_data;

/**
 * @brief Initialize the MPU6050 with specified accelerometer and gyroscope ranges.
 *
 * Resets the sensor, sets accelerometer/gyro full-scale ranges, selects PLL clock,
 * applies a ~44 Hz DLPF, and sets the sample rate to 1 kHz.
 *
 * @param accel_range Desired accelerometer range (±2/4/8/16 g).
 * @param gyro_range Desired gyroscope range (±250/500/1000/2000 deg/s).
 * @return true on success, false if any I2C transaction fails.
 */
bool mpu6050_init(MPU6050_ACCELEROMETER_RANGE accel_range,
                  MPU6050_GYROSCOPE_RANGE gyro_range);

/**
 * @brief Change the gyroscope full-scale range.
 *
 * Writes to the GYRO_CONFIG register and updates @ref current_gyro_range.
 *
 * @param gyro_range Desired range (±250/500/1000/2000 deg/s).
 * @return true if the register write succeeds, false otherwise.
 */
bool mpu6050_set_gyro_range(MPU6050_GYROSCOPE_RANGE gyro_range);

/**
 * @brief Change the accelerometer full-scale range.
 *
 * Writes to the ACCEL_CONFIG register and updates @ref current_accel_range.
 *
 * @param accel_range Desired range (±2/4/8/16 g).
 * @return true if the register write succeeds, false otherwise.
 */
bool mpu6050_set_accel_range(MPU6050_ACCELEROMETER_RANGE accel_range);

/**
 * @brief Read acceleration, gyroscope, and temperature data in one I2C transaction.
 *
 * Reads 14 consecutive bytes (Accel XYZ, Temperature, Gyro XYZ) starting at
 * ACCEL_XOUT_H. Converts raw readings into floating-point units based on current
 * full-scale settings.
 *
 * @param accel Pointer to receive scaled acceleration data in g.
 * @param gyro Pointer to receive scaled gyro data in deg/s.
 * @param temperature Pointer to receive temperature in degrees Celsius.
 * @return true if the read succeeds, false otherwise.
 */
bool mpu6050_read_all(mpu6050_xyz_data* accel,
                      mpu6050_xyz_data* gyro,
                      float* temperature);

/**
 * @brief Configure the digital low-pass filter (DLPF).
 *
 * Writes to CONFIG register and updates @ref current_DLPF_val.
 *
 * @param freq Desired DLPF setting (0–7).
 * @return true if the register write succeeds, false otherwise.
 */
bool mpu6050_set_DLPF_frequency(MPU6050_DLPF_FREQ freq);

/**
 * @brief Set the output sample rate.
 *
 * The base rate is 8 kHz if DLPF is disabled, otherwise 1 kHz.  
 * The actual rate is base_rate / (1 + divider).  
 *
 * @param sample_rate_hz Desired rate in Hz (must be ≤ base rate and > 0).
 * @return true if the register write succeeds, false otherwise.
 */
bool mpu6050_set_sample_rate(uint32_t sample_rate_hz);

#endif /* mpu6050_H */
