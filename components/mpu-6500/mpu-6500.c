#include "mpu-6500.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <string.h>
#include "freertos/task.h"


int mpu6500_init(mpu6500 *device, calData *cal, uint8_t address)
{
    // Initialize address variable and calibration data
    device->IMUAddress = address;
    device->aRes = 16.0 / 32768.0;      // ares value for full range (16g) readings
    device->gRes = 2000.0 / 32768.0;    // gres value for full range (2000dps) readings
    device->geometryIndex = 0;

    if (!cal->valid)
    {
        device->calibration = (calData){0};
    }
    else
    {
        device->calibration = *cal;
    }

    // Read WHO_AM_I register
    uint8_t whoAmI = readByte(device->IMUAddress, MPU6500_WHO_AM_I_MPU6500);
    if (whoAmI != MPU6500_WHOAMI_DEFAULT_VALUE)
    {
        return -1;
    }

    // Reset device
    writeByte(device->IMUAddress, MPU6500_PWR_MGMT_1, 0x80);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // reset device
    writeByte(device->IMUAddress, MPU6500_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // wake up device
    writeByte(device->IMUAddress, MPU6500_PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
    vTaskDelay(100 / portTICK_PERIOD_MS);                                      // Wait for all registers to reset

    // get stable time source
    writeByte(device->IMUAddress, MPU6500_PWR_MGMT_1, 0x01); // Auto select clock source to be PLL gyroscope reference if ready else
    vTaskDelay(200 / portTICK_PERIOD_MS);

    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
    // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
    // be higher than 1 / 0.0059 = 170 Hz
    // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU6500, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
    writeByte(device->IMUAddress, MPU6500_MPU_CONFIG, 0x03);

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    writeByte(device->IMUAddress, MPU6500_SMPLRT_DIV, 0x02); // Use a 500 Hz rate; a rate consistent with the filter update rate
    // determined inset in CONFIG above

    // Set gyroscope full scale range
    // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    uint8_t c = readByte(device->IMUAddress, MPU6500_GYRO_CONFIG); // get current GYRO_CONFIG register value
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x03;           // Clear Fchoice bits [1:0]
    c = c & ~0x18;           // Clear GFS bits [4:3]
    c = c | (uint8_t)3 << 3; // Set full scale range for the gyro (11 on 4:3)
    // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
    writeByte(device->IMUAddress, MPU6500_GYRO_CONFIG, c); // Write new GYRO_CONFIG value to register

    // Set accelerometer full-scale range configuration
    c = readByte(device->IMUAddress, MPU6500_ACCEL_CONFIG); // get current ACCEL_CONFIG register value
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x18;                                  // Clear AFS bits [4:3]
    c = c | (uint8_t)3 << 3;                        // Set full scale range for the accelerometer (11 on 4:3)
    writeByte(device->IMUAddress, MPU6500_ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    c = readByte(device->IMUAddress, MPU6500_ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
    c = c & ~0x0F;                                   // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    c = c | 0x03;                                    // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    writeByte(device->IMUAddress, MPU6500_ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the Arduino as master
    writeByte(device->IMUAddress, MPU6500_INT_PIN_CFG, 0x22); // enable Magnetometer bypass
    writeByte(device->IMUAddress, MPU6500_INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
    vTaskDelay(100 / portTICK_PERIOD_MS);
    return 0;
}

void mpu6500_update(mpu6500 *device)
{
    if (!dataAvailable(device))
        return;

    int16_t IMUCount[7]; // used to read all 14 bytes at once from the MPU6500 accel/gyro
    uint8_t rawData[14]; // x/y/z accel register data stored here

    readBytes(device->IMUAddress, MPU6500_ACCEL_XOUT_H, 14, &rawData[0]); // Read the 14 raw data registers into data array

    IMUCount[0] = ((int16_t)rawData[0] << 8) | rawData[1]; // Turn the MSB and LSB into a signed 16-bit value
    IMUCount[1] = ((int16_t)rawData[2] << 8) | rawData[3];
    IMUCount[2] = ((int16_t)rawData[4] << 8) | rawData[5];
    IMUCount[3] = ((int16_t)rawData[6] << 8) | rawData[7];
    IMUCount[4] = ((int16_t)rawData[8] << 8) | rawData[9];
    IMUCount[5] = ((int16_t)rawData[10] << 8) | rawData[11];
    IMUCount[6] = ((int16_t)rawData[12] << 8) | rawData[13];

    float ax, ay, az, gx, gy, gz;

    // Calculate the accel value into actual g's per second
    ax = (float)IMUCount[0] * device->aRes - device->calibration.accelBias[0];
    ay = (float)IMUCount[1] * device->aRes - device->calibration.accelBias[1];
    az = (float)IMUCount[2] * device->aRes - device->calibration.accelBias[2];

    // Calculate the gyro value into actual degrees per second
    gx = (float)IMUCount[4] * device->gRes - device->calibration.gyroBias[0];
    gy = (float)IMUCount[5] * device->gRes - device->calibration.gyroBias[1];
    gz = (float)IMUCount[6] * device->gRes - device->calibration.gyroBias[2];

    switch (device->geometryIndex)
    {
    case 0:
        device->accel.accelX = ax;
        device->gyro.gyroX = gx;
        device->accel.accelY = ay;
        device->gyro.gyroY = gy;
        device->accel.accelZ = az;
        device->gyro.gyroZ = gz;
        break;
    case 1:
        device->accel.accelX = -ay;
        device->gyro.gyroX = -gy;
        device->accel.accelY = ax;
        device->gyro.gyroY = gx;
        device->accel.accelZ = az;
        device->gyro.gyroZ = gz;
        break;
    case 2:
        device->accel.accelX = -ax;
        device->gyro.gyroX = -gx;
        device->accel.accelY = -ay;
        device->gyro.gyroY = -gy;
        device->accel.accelZ = az;
        device->gyro.gyroZ = gz;
        break;
    case 3:
        device->accel.accelX = ay;
        device->gyro.gyroX = gy;
        device->accel.accelY = -ax;
        device->gyro.gyroY = -gx;
        device->accel.accelZ = az;
        device->gyro.gyroZ = gz;
        break;
    case 4:
        device->accel.accelX = -az;
        device->gyro.gyroX = -gz;
        device->accel.accelY = -ay;
        device->gyro.gyroY = -gy;
        device->accel.accelZ = -ax;
        device->gyro.gyroZ = -gx;
        break;
    case 5:
        device->accel.accelX = -az;
        device->gyro.gyroX = -gz;
        device->accel.accelY = ax;
        device->gyro.gyroY = gx;
        device->accel.accelZ = -ay;
        device->gyro.gyroZ = -gy;
        break;
    case 6:
        device->accel.accelX = -az;
        device->gyro.gyroX = -gz;
        device->accel.accelY = ay;
        device->gyro.gyroY = gy;
        device->accel.accelZ = ax;
        device->gyro.gyroZ = gx;
        break;
    case 7:
        device->accel.accelX = -az;
        device->gyro.gyroX = -gz;
        device->accel.accelY = -ax;
        device->gyro.gyroY = -gx;
        device->accel.accelZ = ay;
        device->gyro.gyroZ = gy;
        break;
    }
}

void mpu6500_getAccel(mpu6500 *device, AccelData *out)
{
    memcpy(out, &(device->accel), sizeof(device->accel));
}
void mpu6500_getGyro(mpu6500 *device, GyroData *out)
{
    memcpy(out, &(device->gyro), sizeof(device->gyro));
}

int mpu6500_setAccelRange(mpu6500 *device, int range)
{
    uint8_t c;
    if (range == 16)
    {
        device->aRes = 16.f / 32768.f; // ares value for full range (16g) readings
        c = 0x03 << 3;
    }
    else if (range == 8)
    {
        device->aRes = 8.f / 32768.f; // ares value for range (8g) readings
        c = 0x02 << 3;
    }
    else if (range == 4)
    {
        device->aRes = 4.f / 32768.f; // ares value for range (4g) readings
        c = 0x01 << 3;
    }
    else if (range == 2)
    {
        device->aRes = 2.f / 32768.f; // ares value for range (2g) readings
        c = 0x00 << 3;
    }
    else
    {
        return -1;
    }
    writeByte(device->IMUAddress, MPU6500_ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value
    return 0;
}

int mpu6500_setGyroRange(mpu6500 *device, int range)
{
    uint8_t c;
    if (range == 2000)
    {
        device->gRes = 2000.f / 32768.f; // ares value for full range (2000dps) readings
        c = 0x03 << 3;
    }
    else if (range == 1000)
    {
        device->gRes = 1000.f / 32768.f; // ares value for range (1000dps) readings
        c = 0x02 << 3;
    }
    else if (range == 500)
    {
        device->gRes = 500.f / 32768.f; // ares value for range (500dps) readings
        c = 0x01 << 3;
    }
    else if (range == 250)
    {
        device->gRes = 250.f / 32768.f; // ares value for range (250dps) readings
        c = 0x00 << 3;
    }
    else
    {
        return -1;
    }
    writeByte(device->IMUAddress, MPU6500_GYRO_CONFIG, c); // Write new GYRO_CONFIG register value
    return 0;
}

void mpu6500_calibrateAccelGyro(mpu6500 *device, calData *cal)
{
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

    // reset device
    writeByte(device->IMUAddress, MPU6500_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
    // else use the internal oscillator, bits 2:0 = 001
    writeByte(device->IMUAddress, MPU6500_PWR_MGMT_1, 0x01);
    writeByte(device->IMUAddress, MPU6500_PWR_MGMT_2, 0x00);
    vTaskDelay(200 / portTICK_PERIOD_MS);

    // Configure device for bias calculation
    writeByte(device->IMUAddress, MPU6500_INT_ENABLE, 0x00);   // Disable all interrupts
    writeByte(device->IMUAddress, MPU6500_FIFO_EN, 0x00);      // Disable FIFO
    writeByte(device->IMUAddress, MPU6500_PWR_MGMT_1, 0x00);   // Turn on internal clock source
    writeByte(device->IMUAddress, MPU6500_I2C_MST_CTRL, 0x00); // Disable I2C master
    writeByte(device->IMUAddress, MPU6500_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    writeByte(device->IMUAddress, MPU6500_USER_CTRL, 0x0C);    // Reset FIFO and DMP
    vTaskDelay(15 / portTICK_PERIOD_MS);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    writeByte(device->IMUAddress, MPU6500_MPU_CONFIG, 0x01);   // Set low-pass filter to 188 Hz
    writeByte(device->IMUAddress, MPU6500_SMPLRT_DIV, 0x00);   // Set sample rate to 1 kHz
    writeByte(device->IMUAddress, MPU6500_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    writeByte(device->IMUAddress, MPU6500_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

    uint16_t gyrosensitivity = 131;    // = 131 LSB/degrees/sec
    uint16_t accelsensitivity = 16384; // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    writeByte(device->IMUAddress, MPU6500_USER_CTRL, 0x40); // Enable FIFO
    writeByte(device->IMUAddress, MPU6500_FIFO_EN, 0x78);   // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
    vTaskDelay(40 / portTICK_PERIOD_MS);                    // accumulate 40 samples in 40 milliseconds = 480 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    writeByte(device->IMUAddress, MPU6500_FIFO_EN, 0x00);            // Disable gyro and accelerometer sensors for FIFO
    readBytes(device->IMUAddress, MPU6500_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++)
    {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        readBytes(device->IMUAddress, MPU6500_FIFO_R_W, 12, &data[0]);        // read data for averaging
        accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]); // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
        accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
        gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
        gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
        gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

        accel_bias[0] += (int32_t)accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t)accel_temp[1];
        accel_bias[2] += (int32_t)accel_temp[2];
        gyro_bias[0] += (int32_t)gyro_temp[0];
        gyro_bias[1] += (int32_t)gyro_temp[1];
        gyro_bias[2] += (int32_t)gyro_temp[2];
    }
    accel_bias[0] /= (int32_t)packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t)packet_count;
    accel_bias[2] /= (int32_t)packet_count;
    gyro_bias[0] /= (int32_t)packet_count;
    gyro_bias[1] /= (int32_t)packet_count;
    gyro_bias[2] /= (int32_t)packet_count;

    switch (device->geometryIndex)
    {
    case 0:
    case 1:
    case 2:
    case 3:
        if (accel_bias[2] > 0L)
        {
            accel_bias[2] -= (int32_t)accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
        }
        else
        {
            accel_bias[2] += (int32_t)accelsensitivity;
        }
        break;
    case 4:
    case 6:
        if (accel_bias[0] > 0L)
        {
            accel_bias[0] -= (int32_t)accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
        }
        else
        {
            accel_bias[0] += (int32_t)accelsensitivity;
        }
        break;
    case 5:
    case 7:
        if (accel_bias[1] > 0L)
        {
            accel_bias[1] -= (int32_t)accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
        }
        else
        {
            accel_bias[1] += (int32_t)accelsensitivity;
        }
        break;
    }
    // Output scaled accelerometer biases for display in the main program
    cal->accelBias[0] = (float)accel_bias[0] / (float)accelsensitivity;
    cal->accelBias[1] = (float)accel_bias[1] / (float)accelsensitivity;
    cal->accelBias[2] = (float)accel_bias[2] / (float)accelsensitivity;
    // Output scaled gyro biases for display in the main program
    cal->gyroBias[0] = (float)gyro_bias[0] / (float)gyrosensitivity;
    cal->gyroBias[1] = (float)gyro_bias[1] / (float)gyrosensitivity;
    cal->gyroBias[2] = (float)gyro_bias[2] / (float)gyrosensitivity;
    cal->valid = true;
}

// I2C read and write functions
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, subAddress, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK)
    {
        ESP_LOGE("MPU6500", "I2C write failed");
    }
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
    uint8_t data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, subAddress, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK)
    {
        ESP_LOGE("MPU6500", "I2C read failed");
    }
    return data;
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest)
{
    if (count > 0)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, subAddress, true);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);
        if (count > 1)
        {
            i2c_master_read(cmd, dest, count - 1, I2C_MASTER_ACK);
        }
        i2c_master_read_byte(cmd, dest + count - 1, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        if (ret != ESP_OK)
        {
            ESP_LOGE("MPU6500", "I2C read multiple failed");
        }
    }
}

bool dataAvailable(mpu6500 *device) { return (readByte(device->IMUAddress, MPU6500_INT_STATUS) & 0x01); }