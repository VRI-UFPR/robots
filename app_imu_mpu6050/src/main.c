// ======================================================================================
//  Header
// ======================================================================================

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <ufr.h>

// MPU6050 Default I2C Address 
#define MPU6050_ADDR         0x68

// MPU6050 Register Map
#define REG_PWR_MGMT_1       0x6B
#define REG_ACCEL_XOUT_H     0x3B
#define REG_GYRO_XOUT_H      0x43

// Sensitivity Scale Factors (Default +/-2g, +/-250 deg/s)
#define ACCEL_SCALE          16384.0
#define GYRO_SCALE           131.0

// Global variable for private functions
int i2c_fd = -1;

static
int mpu6050_write_reg(uint8_t reg, uint8_t data);

// ======================================================================================
//  Private Functions
// ======================================================================================

// Abre a conexão com o MPU6050
static
int mpu6050_open(const char *i2c_bus) {
    // 1. Open the I2C bus device file
    i2c_fd = open(i2c_bus, O_RDWR);
    if (i2c_fd < 0) {
        perror("Failed to open I2C bus. Check if i2c-dev is loaded");
        return -1;
    }

    // 2. Set the I2C slave address for the MPU6050
    if (ioctl(i2c_fd, I2C_SLAVE, MPU6050_ADDR) < 0) {
        perror("Failed to acquire bus access/talk to slave");
        close(i2c_fd);
        return -2;
    }

    // 3. Wake up MPU6050 (Clears sleep mode bit in PWR_MGMT_1)
    if (mpu6050_write_reg(REG_PWR_MGMT_1, 0x00) < 0) {
        close(i2c_fd);
        return -3;
    }

    // 4. Success
    return 0;
}

// Fecha a conexão com o mpu6050
static
void mpu6050_close() {
    close(i2c_fd);
}

// Write a single byte to a specific register
static
int mpu6050_write_reg(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    if (write(i2c_fd, buf, 2) != 2) {
        perror("Failed to write to register");
        return -1;
    }
    return 0;
}

// Read continuous data blocks starting from a specific register
static
int mpu6050_read_block(uint8_t reg, uint8_t buffer[], int size) {
    if (write(i2c_fd, &reg, 1) != 1) {
        perror("Failed to send register address");
        return -1;
    }
    if (read(i2c_fd, buffer, size) != size) {
        perror("Failed to read data block");
        return -1;
    }
    return 0;
}

// ======================================================================================
//  Main
// ======================================================================================

int main(int argc, char** argv) {
    // 1. Inicializa o ufr_stdout
    // ufr_stdout("@new mqtt @coder msgpack @host 177.153.62.174 @topic /imu");
    ufr_stdout("@new mqtt @coder msgpack @host 177.153.62.174 @topic /imu");

    // 2. Change "/dev/i2c-1" to match your specific hardware's I2C bus index
    const int res = mpu6050_open("/dev/i2c-1");
    if ( res < 0 ) {
        return 1;
    }

    // 3. Data retrieval loop
    uint8_t data_buf[14];
    int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
    printf("MPU6050 Initialized Successfully.\n");
    while ( ufr_loop() ) {
        // Read 14 sequential bytes (Accel X/Y/Z, Temp, Gyro X/Y/Z)
        if (mpu6050_read_block(REG_ACCEL_XOUT_H, data_buf, 14) == 0) {
            
            // Combine high and low bytes into signed 16-bit values
            raw_ax = (data_buf[0] << 8) | data_buf[1];
            raw_ay = (data_buf[2] << 8) | data_buf[3];
            raw_az = (data_buf[4] << 8) | data_buf[5];
            
            raw_gx = (data_buf[8] << 8) | data_buf[9];
            raw_gy = (data_buf[10] << 8) | data_buf[11];
            raw_gz = (data_buf[12] << 8) | data_buf[13];

            // Print raw and scaled physical units
            /*printf("\033[H\033[J"); // Clear screen terminal escape codes
            printf("--- MPU6050 Sensor Readings ---\n");
            printf("Accel X: %6d | %6.2fg\n", raw_ax, (float)raw_ax / ACCEL_SCALE);
            printf("Accel Y: %6d | %6.2fg\n", raw_ay, (float)raw_ay / ACCEL_SCALE);
            printf("Accel Z: %6d | %6.2fg\n", raw_az, (float)raw_az / ACCEL_SCALE);
            printf("Gyro  X: %6d | %6.2f°/s\n", raw_gx, (float)raw_gx / GYRO_SCALE);
            printf("Gyro  Y: %6d | %6.2f°/s\n", raw_gy, (float)raw_gy / GYRO_SCALE);
            printf("Gyro  Z: %6d | %6.2f°/s\n", raw_gz, (float)raw_gz / GYRO_SCALE);*/

            ufr_printf("%d %d %d", raw_ax, raw_ay, raw_az);
            ufr_printf("%d %d %d\n", raw_gx, raw_gy, raw_gz);
        }
        usleep(200000); // Sample loop delay (200ms)
    }

    // 5. Fim
    mpu6050_close();
    return EXIT_SUCCESS;
}
