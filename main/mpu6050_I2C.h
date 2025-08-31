#ifndef mpu_6050_h // header guard
#define mpu_6050h

#include "my_I2C.h" // we need access to I2C functions
#define MPU6050_ADDRESS 0x68 //68
// #define MPU6050_OUTPUT_WIDTH 2 // 16 bit ADCs

/*
registers of interest on the MPU6050 -- see datasheet
many of them like the x,y,z acceleration/gyro data will be split into H and L
this means the subsequent address will be used for the same variable but the lower bits

ex register 0x3B is for xout_H and 3D for xout_L since each half is 8 bits
*/

#define MPU6050_ACCEL_X_OUT_REG   0x3B
#define MPU6050_ACCEL_Y_OUT_REG   0x3D
#define MPU6050_ACCEL_Z_OUT_REG   0x3F
#define MPU6050_TEMP_OUT_REG      0x41
#define MPU6050_GYRO_X_OUT_REG    0x43
#define MPU6050_GYRO_Y_OUT_REG    0x45
#define MPU6050_GYRO_Z_OUT_REG    0x47

#define MPU6050_GYRO_CONFIG_REG   0x1B
#define MPU6050_ACCEL_CONFIG_REG  0x1C
#define MPU6050_PWR_MGMT_1_REG    0x6B // power management registers
#define MPU6050_PWR_MGMT_2_REG    0x6C


/*
these are our configuration options for the full range of the gyroscope
units are in degrees per second
*/
typedef enum gyro_ranges{
    MPU6050_RANGE_250_DEG = 0,
    MPU6050_RANGE_500_DEG = 1,
    MPU6050_RANGE_1000_DEG = 2,
    MPU6050_RANGE_2000_DEG = 3
} MPU6050_GYROSCOPE_RANGE;

/*
these are our configuration options for the full range of the gyroscope
units are in g (1g = 9.8 m/s^2)
*/
typedef enum accel_ranges{
    MPU6050_RANGE_2_G = 0,
    MPU6050_RANGE_4_G = 1,
    MPU6050_RANGE_8_G = 2,
    MPU6050_RANGE_16_G = 3
} MPU6050_ACCELEROMETER_RANGE;

// for packing acceleration/gyro data into structs
typedef struct xyz{
    float x;
    float y;
    float z;
} mpu6050_xyz_data;

bool mpu6050_init(const MPU6050_ACCELEROMETER_RANGE a, const MPU6050_GYROSCOPE_RANGE g);
bool mpu6050_set_gyro_range(const MPU6050_GYROSCOPE_RANGE range);
bool mpu6050_set_accel_range(const MPU6050_ACCELEROMETER_RANGE range);
bool mpu6050_reset(void);
static inline bool mpu6050_write_to_register(const byte register_to_write_to, const byte value_to_write);
bool mpu6050_read_all(mpu6050_xyz_data* accel, mpu6050_xyz_data* gyro, float* temperature);

/*

NOTE: we will read in two bytes at a time since we are reading floats (16 bits)

FROM DATASHEET:

To write the internal MPU-60X0 registers, the master transmits the start condition (S), followed by the I2C
address and the write bit (0). At the 9th clock cycle (when the clock is high), the MPU-60X0 acknowledges the
transfer. Then the master puts the register address (RA) on the bus. After the MPU-60X0 acknowledges the
reception of the register address, the master puts the register data onto the bus. This is followed by the ACK
signal, and data transfer may be concluded by the stop condition (P). To write multiple bytes after the last
ACK signal, the master can continue outputting data rather than transmitting a stop signal. In this case, the
MPU-60X0 automatically increments the register address and loads the data to the appropriate register. The
following figures show single and two-byte write sequences.


To read the internal MPU-60X0 registers, the master sends a start condition, followed by the I2C address and
a write bit, and then the register address that is going to be read. Upon receiving the ACK signal from the
MPU-60X0, the master transmits a start signal followed by the slave address and read bit. As a result, the
MPU-60X0 sends an ACK signal and the data. The communication ends with a not acknowledge (NACK)
signal and a stop bit from master. The NACK condition is defined such that the SDA line remains high at the
9th clock cycle. The following figures show single and two-byte read sequences

*/

bool mpu6050_init(const MPU6050_ACCELEROMETER_RANGE a, const MPU6050_GYROSCOPE_RANGE g) {
    // reset the sensor first
    if (!mpu6050_reset()) {
        printf("failed reset\n");
        return false;
    }
    if (!mpu6050_write_to_register(MPU6050_GYRO_CONFIG_REG, g)) {
        printf("failed gyro set\n");
        return false;
    }
    // printf("second\n");
    if (!mpu6050_write_to_register(MPU6050_ACCEL_CONFIG_REG, a)) {
        printf("failed accel set\n");
        return false;
    }
    
    return true;
}

static inline bool mpu6050_write_to_register(const byte register_to_write_to, const byte value_to_write) {
    // send register we want to write to, then the value. Make sure the register can be written to
    byte transmission[2] = {register_to_write_to, value_to_write};
    return I2C_send_byte_stream(MPU6050_ADDRESS, transmission, 2, WRITE, true, true);
}

/*
gets acceleration, gyro, and temperature data
retrieves the most recent values written. This is determined by the sample rate in register 25
*/
bool mpu6050_read_all(mpu6050_xyz_data* accel, mpu6050_xyz_data* gyro, float* temperature) {

    return false;
}

// resets all internal registers to default state
bool mpu6050_reset(void) {
    // set the MSB of the power register to 1
    bool status = mpu6050_write_to_register(MPU6050_PWR_MGMT_1_REG, 0x80);
    // important: wait for the reset to complete
    // trying to use the sensor write away will NACK
    vTaskDelay(1); // 10 ms delay 
    return status;
}

bool mpu6050_set_gyro_range(const MPU6050_GYROSCOPE_RANGE g) {
    return mpu6050_write_to_register(MPU6050_GYRO_CONFIG_REG, g << 3);
}

bool mpu6050_set_accel_range(const MPU6050_ACCELEROMETER_RANGE a) {
    return mpu6050_write_to_register(MPU6050_GYRO_CONFIG_REG, a << 3);
}
#endif /* mpu_6050.h */