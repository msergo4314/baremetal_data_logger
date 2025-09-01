#include "mpu6050_I2C.h"
#include "esp_rom_sys.h"

// trackers for settings that would otherwise have to be read from registers (slow)
MPU6050_GYROSCOPE_RANGE current_gyro_range;
MPU6050_ACCELEROMETER_RANGE current_accel_range;
MPU6050_DLPF_FREQ current_DLPF_val;

// helpers not to be used outside of this file
static inline bool mpu6050_write_to_register(byte register_to_write_to, byte value_to_write);
static inline bool mpu6050_read_from_register(byte register_to_read, byte* register_value);
static inline bool mpu6050_read_register_block(byte register_to_read, byte* register_values, byte number_of_registers);
static inline int16_t combine_bytes(byte high, byte low);
static float get_temperature_centigrade(mpu6050_raw_data raw_temperature_reading);
static inline float raw_accel_to_float(mpu6050_raw_data raw_accel);
static inline float raw_gyro_to_float(mpu6050_raw_data raw_gyro);

bool mpu6050_init(MPU6050_ACCELEROMETER_RANGE accel_range, MPU6050_GYROSCOPE_RANGE gyro_range) {
    // reset the sensor first
    if (!mpu6050_reset()) return false;
    if (!mpu6050_set_accel_range(accel_range)) return false;
    if (!mpu6050_set_gyro_range(gyro_range)) return false;
    // Wake up and select PLL clock
    if (!mpu6050_write_to_register(MPU6050_PWR_MGMT_1_REG, 0x01)) return false;

    // Set DLPF to ~44Hz
    if (!mpu6050_set_DLPF_frequency(MPU6050_DLPF_44_HZ)) return false;

    // Set sample rate divider to 0 -> 1kHz
    if (!mpu6050_set_sample_rate(1000)) return false;
    return true;
}

/*
gets acceleration, gyro, and temperature data.
Retrieves the most recent values written. This is determined by the sample rate in register 25 (0x19)
Acceleration and gyro will be scaled according to the ranges set. Temperature is in degrees centigrade.
*/
bool mpu6050_read_all(mpu6050_xyz_data* accel, mpu6050_xyz_data* gyro, float* temperature) {

    /*
    xyz data is 2 bytes for each dimension (6 bytes) 
    we are reading xyz data for both acceleration and gyro (12 bytes)
    two addional bytes are needed for the temperature
    total = 14 bytes
    */
    if (!accel || !gyro || !temperature) {
        printf("passed NULL pointer to mpu6050_read_all() function\n");
        return false;
    }
    byte read_data[14] = {0};
    if (!mpu6050_read_register_block(MPU6050_ACCEL_X_OUT_REG, read_data, sizeof(read_data))) return false;
    mpu6050_raw_data a_x = combine_bytes(read_data[0], read_data[1]);
    mpu6050_raw_data a_y = combine_bytes(read_data[2], read_data[3]);
    mpu6050_raw_data a_z = combine_bytes(read_data[4], read_data[5]);  

    mpu6050_raw_data raw_temp = combine_bytes(read_data[6], read_data[7]);

    mpu6050_raw_data g_x = combine_bytes(read_data[8], read_data[9]);
    mpu6050_raw_data g_y = combine_bytes(read_data[10], read_data[11]);
    mpu6050_raw_data g_z = combine_bytes(read_data[12], read_data[13]);

    accel->x = raw_accel_to_float(a_x);
    accel->y = raw_accel_to_float(a_y);
    accel->z = raw_accel_to_float(a_z);

    *temperature = get_temperature_centigrade(raw_temp);

    gyro->x = raw_gyro_to_float(g_x);
    gyro->y = raw_gyro_to_float(g_y);
    gyro->z = raw_gyro_to_float(g_z);

    return true;
}

// resets all internal registers to default state
bool mpu6050_reset(void) {
    // set the MSB of the power register to 1
    bool status = mpu6050_write_to_register(MPU6050_PWR_MGMT_1_REG, 0x80);
    // important: wait for the reset to complete
    // trying to use the sensor write away will NACK
    // trying to use vTaskDelay() here won't work so use esp_rom_delay_us()
    esp_rom_delay_us(100 * 1000);
    return status;
}

bool mpu6050_set_gyro_range(MPU6050_GYROSCOPE_RANGE gyro_range) {
    if (!mpu6050_write_to_register(MPU6050_GYRO_CONFIG_REG, gyro_range << 3)) return false;
    current_gyro_range = gyro_range;
    return true;
}

bool mpu6050_set_accel_range(MPU6050_ACCELEROMETER_RANGE accel_range) {
    if (!mpu6050_write_to_register(MPU6050_ACCEL_CONFIG_REG, accel_range << 3)) return false;
    current_accel_range = accel_range;
    return true;
}

bool mpu6050_set_DLPF_frequency(MPU6050_DLPF_FREQ freq) {
    // bottom 3 bits of the register control DLPF
    if (!mpu6050_write_to_register(MPU6050_CONFIGURATION_REG, freq)) return false;
    current_DLPF_val = freq;
    return true;
}

bool mpu6050_set_sample_rate(uint32_t sample_rate_hz) {
    uint32_t gyro_out = (current_DLPF_val == 0 || current_DLPF_val == MPU6050_DLPF_DISABLED) ? 8000U : 1000U;
    if (sample_rate_hz == 0 || sample_rate_hz > gyro_out) return false;
    uint8_t sample_rate_div = (uint8_t)((gyro_out / sample_rate_hz) - 1);
    return mpu6050_write_to_register(MPU6050_SMPLRT_DIV_REG, sample_rate_div);
}


// packs the high and low bytes into a 16 bit signed integer
static inline int16_t combine_bytes(byte high, byte low) {
    return (int16_t)((high << 8) | low);
}

/*
convert the raw reading to a celsius value
note that this is die temperature and is > room temperature
*/
static inline float get_temperature_centigrade(mpu6050_raw_data raw_temperature_reading) {
    return ((float)raw_temperature_reading / 340.0f) + 36.53f;
}

// converts a single dimension raw reading to float
static inline float raw_accel_to_float(mpu6050_raw_data raw_accel) {
    
    float raw_LSBs_per_g;
    switch (current_accel_range) {
        case MPU6050_RANGE_2_G:
            raw_LSBs_per_g = 16384.0;
            break;
        case MPU6050_RANGE_4_G:
            raw_LSBs_per_g = 8192.0;
            break;
        case MPU6050_RANGE_8_G:
            raw_LSBs_per_g = 4096.0;
            break;
        case MPU6050_RANGE_16_G:
            raw_LSBs_per_g = 2048.0;
            break;
        default:
            printf("Invalid accel range!\n");
            raw_LSBs_per_g = 16384.0;
            break;        
    }
    // convert to gs
    return ((float)raw_accel) / raw_LSBs_per_g;
}

static inline bool mpu6050_write_to_register(byte register_to_write_to, byte value_to_write) {
    // send register we want to write to, then the value. Make sure the register can be written to
    byte transmission[2] = {register_to_write_to, value_to_write};
    return I2C_send_byte_stream(MPU6050_ADDRESS, transmission, 2, WRITE, true, true);
}

// wrapper for I2C_read_one() function
static inline bool mpu6050_read_from_register(byte register_to_read, byte* register_value) {
    return I2C_read_one(MPU6050_ADDRESS, register_to_read, register_value);
}

// wrapper for I2C_read_many() function
static inline bool mpu6050_read_register_block(byte starting_register, byte* register_values, byte number_of_registers) {
    return I2C_read_many(MPU6050_ADDRESS, starting_register, number_of_registers, register_values);
}

// converts a single dimension raw reading to float
static inline float raw_gyro_to_float(mpu6050_raw_data raw_gyro) {
    
    float raw_LSBs_per_degree_per_second;
    switch (current_gyro_range) {
        case MPU6050_RANGE_250_DEG:
            raw_LSBs_per_degree_per_second = 131.0;
            break;
        case MPU6050_RANGE_500_DEG:
            raw_LSBs_per_degree_per_second = 65.5;
            break;
        case MPU6050_RANGE_1000_DEG:
            raw_LSBs_per_degree_per_second = 32.8;
            break;
        case MPU6050_RANGE_2000_DEG:
            raw_LSBs_per_degree_per_second = 16.4;
            break;
        default:
            printf("Invalid gyro range!\n");
            raw_LSBs_per_degree_per_second = 131.0;
            break;
    }
    // convert to degrees per second
    return (float)(raw_gyro / raw_LSBs_per_degree_per_second);
}
