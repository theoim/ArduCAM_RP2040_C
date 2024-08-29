#include "ArduCAM.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/irq.h"
#include "pico/binary_info.h"
#include "ov2640_regs.h"
#include "ov5642_regs.h"

// 기본 초기화 함수
void ArduCAM_Init_Model(ArduCAM *cam) {
    cam->sensor_model = OV7670; // 모델 기본값 설정
    cam->sensor_addr = 0x42;    // 주소 기본값 설정
}

// 매개변수가 있는 초기화 함수
void ArduCAM_InitWithModelAndCS(ArduCAM *cam, uint8_t model, int CS) {
    cam->B_CS = CS;
    cam->P_CS = &CS;  // P_CS는 CS의 주소를 가리키게 설정
    // sbi 함수는 여기서 칩 선택 핀을 활성화하는 동작을 수행해야 함
    sbi(cam->P_CS, cam->B_CS);

    cam->sensor_model = model;
    switch (model) {
        case OV2640:
            cam->sensor_addr = 0x30;
            break;
        case OV5642:
            cam->sensor_addr = 0x3C;
            break;
        default:
            cam->sensor_addr = 0x60; // 기본값 설정
            break;
    }
}

void ArduCAM_InitCAM(ArduCAM* cam) {
    switch (cam->sensor_model) {
        case OV2640:
            ArduCAM_wrSensorReg8_8(cam, 0xff, 0x01);
            ArduCAM_wrSensorReg8_8(cam, 0x12, 0x80);
            sleep_ms(100);
            if (cam->m_fmt == JPEG) {
                ArduCAM_wrSensorRegs8_8(cam, OV2640_JPEG_INIT);
                ArduCAM_wrSensorRegs8_8(cam, OV2640_YUV422);
                ArduCAM_wrSensorRegs8_8(cam, OV2640_JPEG);
                ArduCAM_wrSensorReg8_8(cam, 0xff, 0x01);
                ArduCAM_wrSensorReg8_8(cam, 0x15, 0x00);
                ArduCAM_wrSensorRegs8_8(cam, OV2640_320x240_JPEG);
            } else {
                ArduCAM_wrSensorRegs8_8(cam, OV2640_QVGA);
            }
            break;
        case OV5642:
            ArduCAM_wrSensorReg16_8(cam, 0x3008, 0x80);
            if (cam->m_fmt == RAW) {
                ArduCAM_wrSensorRegs16_8(cam, OV5642_1280x960_RAW);
                ArduCAM_wrSensorRegs16_8(cam, OV5642_640x480_RAW);
            } else {
                ArduCAM_wrSensorRegs16_8(cam, OV5642_QVGA_Preview);
                sleep_ms(100);
                if (cam->m_fmt == JPEG) {
                    sleep_ms(100);
                    ArduCAM_wrSensorRegs16_8(cam, OV5642_JPEG_Capture_QSXGA);
                    ArduCAM_wrSensorRegs16_8(cam, ov5642_320x240);
                    sleep_ms(100);
                    ArduCAM_wrSensorReg16_8(cam, 0x3818, 0xa8);
                    ArduCAM_wrSensorReg16_8(cam, 0x3621, 0x10);
                    ArduCAM_wrSensorReg16_8(cam, 0x3801, 0xb0);
                    ArduCAM_wrSensorReg16_8(cam, 0x4407, 0x04);
                } else {
                    byte reg_val;
                    ArduCAM_wrSensorReg16_8(cam, 0x4740, 0x21);
                    ArduCAM_wrSensorReg16_8(cam, 0x501e, 0x2a);
                    ArduCAM_wrSensorReg16_8(cam, 0x5002, 0xf8);
                    ArduCAM_wrSensorReg16_8(cam, 0x501f, 0x01);
                    ArduCAM_wrSensorReg16_8(cam, 0x4300, 0x61);
                    ArduCAM_rdSensorReg16_8(cam, 0x3818, &reg_val);
                    ArduCAM_wrSensorReg16_8(cam, 0x3818, (reg_val | 0x60) & 0xff);
                    ArduCAM_rdSensorReg16_8(cam, 0x3621, &reg_val);
                    ArduCAM_wrSensorReg16_8(cam, 0x3621, reg_val & 0xdf);
                }
            }
            break;
        default:
            break;
    }
}

// CS 핀 제어 함수
void ArduCAM_CS_HIGH(ArduCAM* cam) {
    sbi(cam->P_CS, cam->B_CS);
}

void ArduCAM_CS_LOW(ArduCAM* cam) {
    cbi(cam->P_CS, cam->B_CS);
}

// FIFO 제어 함수들
void ArduCAM_flush_fifo(ArduCAM* cam) {
    ArduCAM_write_reg(cam, ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

void ArduCAM_start_capture(ArduCAM* cam) {
    ArduCAM_write_reg(cam, ARDUCHIP_FIFO, FIFO_START_MASK);
}

void ArduCAM_clear_fifo_flag(ArduCAM* cam) {
    ArduCAM_write_reg(cam, ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

uint8_t ArduCAM_read_fifo(ArduCAM* cam) {
    return ArduCAM_bus_read(cam, SINGLE_FIFO_READ);
}



uint8_t ArduCAM_read_reg(ArduCAM* cam, uint8_t addr) {
    uint8_t value = 0;
    addr &= 0x7f;  // Mask to ensure MSB is 0 for a read operation
    cbi(cam->P_CS, cam->B_CS);
    int sent = spi_write_blocking(SPI_PORT, &addr, 1);
    int received = spi_read_blocking(SPI_PORT, 0, &value, 1);
    sbi(cam->P_CS, cam->B_CS);
    
    printf("read Sent %d bytes (Addr: 0x%02X)\n", sent, addr);
    printf("read Received %d bytes (Value: 0x%02X)\n", received, value);

    return value;
}

// 레지스터 쓰기 함수
void ArduCAM_write_reg(ArduCAM* cam, uint8_t addr, uint8_t data) {
    uint8_t buf[2] = { addr | WRITE_BIT, data };  // Set MSB for write operation
    cbi(cam->P_CS, cam->B_CS);
    int sent = spi_write_blocking(SPI_PORT, buf, 2);
    sbi(cam->P_CS, cam->B_CS);
    sleep_ms(1);  // Short delay after writing

    printf("Write Sent %d bytes (Addr: 0x%02X, Data: 0x%02X)\n", sent, addr, data);
}


// FIFO 길이 읽기
uint32_t ArduCAM_read_fifo_length(ArduCAM* cam) {
    uint32_t len1 = 0;
    uint32_t len2 = 0;
    uint32_t len3 = 0;
    uint32_t length = 0;

    len1 = ArduCAM_read_reg(cam ,FIFO_SIZE1);
    len2 = ArduCAM_read_reg(cam ,FIFO_SIZE2);
    len3 = ArduCAM_read_reg(cam ,FIFO_SIZE3) & 0x7f;
    length = ((len3 << 16) | (len2 << 8) | len1) & 0x07fffff;
    return length;
}

void ArduCAM_set_fifo_burst(ArduCAM* cam) {
    uint8_t value;
    spi_read_blocking(SPI_PORT, BURST_FIFO_READ, &value, 1);
}

// 비트 제어 함수들
void ArduCAM_set_bit(ArduCAM* cam, uint8_t addr, uint8_t bit) {
    uint8_t temp = ArduCAM_read_reg(cam, addr);
    ArduCAM_write_reg(cam, addr, temp | bit);
}

void ArduCAM_clear_bit(ArduCAM* cam, uint8_t addr, uint8_t bit) {
    uint8_t temp = ArduCAM_read_reg(cam, addr);
    ArduCAM_write_reg(cam, addr, temp & (~bit));
}

uint8_t ArduCAM_get_bit(ArduCAM* cam, uint8_t addr, uint8_t bit) {
    return ArduCAM_read_reg(cam, addr) & bit;
}

// 버스 읽기/쓰기 함수
uint8_t ArduCAM_bus_write(ArduCAM* cam, int address, int value) {
    cbi(cam->P_CS, cam->B_CS);
    // SPI 전송 코드 추가 필요
    sbi(cam->P_CS, cam->B_CS);
    return 1;
}

uint8_t ArduCAM_bus_read(ArduCAM* cam, int address) {
    uint8_t value;
    cbi(cam->P_CS, cam->B_CS);
    // SPI 전송 코드 추가 필요
    sbi(cam->P_CS, cam->B_CS);
    return value;
}

// 센서 레지스터 쓰기/읽기 함수들
int ArduCAM_wrSensorRegs8_8(ArduCAM* cam, const struct sensor_reg reglist[]) {
    int err = 0;
    const struct sensor_reg *next = reglist;
    while ((next->reg != 0xff) || (next->val != 0xff)) {
        err = ArduCAM_wrSensorReg8_8(cam, next->reg, next->val);
        next++;
    }
    return err;
}

int ArduCAM_wrSensorRegs8_16(ArduCAM* cam, const struct sensor_reg reglist[]) {
    int err = 0;
    const struct sensor_reg *next = reglist;
    while ((next->reg != 0xff) || (next->val != 0xffff)) {
        err = ArduCAM_wrSensorReg8_16(cam, next->reg, next->val);
        next++;
    }
    return err;
}

int ArduCAM_wrSensorRegs16_8(ArduCAM* cam, const struct sensor_reg reglist[]) {
    int err = 0;
    const struct sensor_reg *next = reglist;
    while ((next->reg != 0xffff) || (next->val != 0xff)) {
        err = ArduCAM_wrSensorReg16_8(cam, next->reg, next->val);
        next++;
    }
    return err;
}

// 센서 레지스터 개별 쓰기/읽기 함수
byte ArduCAM_wrSensorReg8_8(ArduCAM* cam, int regID, int regDat) {
    uint8_t buf[2] = { regID, regDat };
    i2c_write_blocking(I2C_PORT, cam->sensor_addr, buf, 2, true);
    return 1;
}

byte ArduCAM_rdSensorReg8_8(ArduCAM* cam, uint8_t regID, uint8_t* regDat) {
    i2c_write_blocking(I2C_PORT, cam->sensor_addr, &regID, 1, true);
    i2c_read_blocking(I2C_PORT, cam->sensor_addr, regDat, 1, false);
    return 1;
}

byte ArduCAM_wrSensorReg16_8(ArduCAM* cam, int regID, int regDat) {
    uint8_t buf[3] = { (regID >> 8) & 0xff, regID & 0xff, regDat };
    i2c_write_blocking(I2C_PORT, cam->sensor_addr, buf, 3, true);
    sleep_ms(2);
    return 1;
}

byte ArduCAM_rdSensorReg16_8(ArduCAM* cam, uint16_t regID, uint8_t* regDat) {
    uint8_t buffer[2] = { (regID >> 8) & 0xff, regID & 0xff };
    i2c_write_blocking(I2C_PORT, cam->sensor_addr, buffer, 2, true);
    i2c_read_blocking(I2C_PORT, cam->sensor_addr, regDat, 1, false);
    return 1;
}

void ArduCAM_OV2640_set_Special_effects(ArduCAM* cam, uint8_t Special_effect) {
    switch(Special_effect) {
        case Antique:
            ArduCAM_wrSensorReg8_8(cam, 0xff, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x18);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x05);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x40);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0xa6);
            break;
        case Bluish:
            ArduCAM_wrSensorReg8_8(cam, 0xff, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x18);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x05);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0xa0);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x40);
            break;
        case Greenish:
            ArduCAM_wrSensorReg8_8(cam, 0xff, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x18);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x05);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x40);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x40);
            break;
        case Reddish:
            ArduCAM_wrSensorReg8_8(cam, 0xff, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x18);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x05);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x40);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0xc0);
            break;
        case BW:
            ArduCAM_wrSensorReg8_8(cam, 0xff, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x18);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x05);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x80);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x80);
            break;
        case Negative:
            ArduCAM_wrSensorReg8_8(cam, 0xff, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x40);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x05);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x80);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x80);
            break;
        case BWnegative:
            ArduCAM_wrSensorReg8_8(cam, 0xff, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x58);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x05);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x80);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x80);
            break;
        case Normal:
            ArduCAM_wrSensorReg8_8(cam, 0xff, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x05);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x80);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x80);
            break;
    }
}


void ArduCAM_OV2640_set_Contrast(ArduCAM* cam, uint8_t Contrast) {
    switch(Contrast) {
        case Contrast2:
            ArduCAM_wrSensorReg8_8(cam, 0xff, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x04);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x07);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x20);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x28);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x0c);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x06);
            break;
        case Contrast1:
            ArduCAM_wrSensorReg8_8(cam, 0xff, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x04);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x07);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x20);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x24);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x16);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x06); 
            break;
        case Contrast0:
            ArduCAM_wrSensorReg8_8(cam, 0xff, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x04);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x07);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x20);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x20);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x20);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x06); 
            break;
        case Contrast_1:
            ArduCAM_wrSensorReg8_8(cam, 0xff, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x04);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x07);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x20);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x20);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x2a);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x06);    
            break;
        case Contrast_2:
            ArduCAM_wrSensorReg8_8(cam, 0xff, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x04);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x07);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x20);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x18);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x34);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x06);
            break;
    }
}


void ArduCAM_OV2640_set_Brightness(ArduCAM* cam, uint8_t Brightness) {
    switch(Brightness) {
        case Brightness2:
            ArduCAM_wrSensorReg8_8(cam, 0xff, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x04);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x09);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x40);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x00);
            break;
        case Brightness1:
            ArduCAM_wrSensorReg8_8(cam, 0xff, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x04);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x09);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x30);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x00);
            break;    
        case Brightness0:
            ArduCAM_wrSensorReg8_8(cam, 0xff, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x04);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x09);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x20);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x00);
            break;
        case Brightness_1:
            ArduCAM_wrSensorReg8_8(cam, 0xff, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x04);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x09);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x10);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x00);
            break;
        case Brightness_2:
            ArduCAM_wrSensorReg8_8(cam, 0xff, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x04);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x09);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x00);
            break;    
    }
}

void ArduCAM_OV2640_set_Color_Saturation(ArduCAM* cam, uint8_t Color_Saturation) {
    switch(Color_Saturation) {
        case Saturation2:
            ArduCAM_wrSensorReg8_8(cam, 0xff, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x02);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x03);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x68);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x68);
            break;
        case Saturation1:
            ArduCAM_wrSensorReg8_8(cam, 0xff, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x02);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x03);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x58);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x58);
            break;
        case Saturation0:
            ArduCAM_wrSensorReg8_8(cam, 0xff, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x02);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x03);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x48);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x48);
            break;
        case Saturation_1:
            ArduCAM_wrSensorReg8_8(cam, 0xff, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x02);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x03);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x38);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x38);
            break;
        case Saturation_2:
            ArduCAM_wrSensorReg8_8(cam, 0xff, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x02);
            ArduCAM_wrSensorReg8_8(cam, 0x7c, 0x03);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x28);
            ArduCAM_wrSensorReg8_8(cam, 0x7d, 0x28);
            break;    
    }
}


void ArduCAM_OV2640_set_Light_Mode(ArduCAM* cam, uint8_t Light_Mode) {
    switch(Light_Mode) {
        case Auto:
            ArduCAM_wrSensorReg8_8(cam, 0xff, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0xc7, 0x00); // AWB on
            break;
        case Sunny:
            ArduCAM_wrSensorReg8_8(cam, 0xff, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0xc7, 0x40); // AWB off
            ArduCAM_wrSensorReg8_8(cam, 0xcc, 0x5e);
            ArduCAM_wrSensorReg8_8(cam, 0xcd, 0x41);
            ArduCAM_wrSensorReg8_8(cam, 0xce, 0x54);
            break;
        case Cloudy:
            ArduCAM_wrSensorReg8_8(cam, 0xff, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0xc7, 0x40); // AWB off
            ArduCAM_wrSensorReg8_8(cam, 0xcc, 0x65);
            ArduCAM_wrSensorReg8_8(cam, 0xcd, 0x41);
            ArduCAM_wrSensorReg8_8(cam, 0xce, 0x4f);
            break;
        case Office:
            ArduCAM_wrSensorReg8_8(cam, 0xff, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0xc7, 0x40); // AWB off
            ArduCAM_wrSensorReg8_8(cam, 0xcc, 0x52);
            ArduCAM_wrSensorReg8_8(cam, 0xcd, 0x41);
            ArduCAM_wrSensorReg8_8(cam, 0xce, 0x66);
            break;
        case Home:
            ArduCAM_wrSensorReg8_8(cam, 0xff, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0xc7, 0x40); // AWB off
            ArduCAM_wrSensorReg8_8(cam, 0xcc, 0x42);
            ArduCAM_wrSensorReg8_8(cam, 0xcd, 0x3f);
            ArduCAM_wrSensorReg8_8(cam, 0xce, 0x71);
            break;
        default:
            ArduCAM_wrSensorReg8_8(cam, 0xff, 0x00);
            ArduCAM_wrSensorReg8_8(cam, 0xc7, 0x00); // AWB on
            break;
    }
}


// OV2640 JPEG 사이즈 설정 함수
void ArduCAM_OV2640_set_JPEG_size(ArduCAM* cam, uint8_t size) {
    switch(size) {
        case OV2640_160x120:
            ArduCAM_wrSensorRegs8_8(cam, OV2640_160x120_JPEG);
            break;
        case OV2640_176x144:
            ArduCAM_wrSensorRegs8_8(cam, OV2640_176x144_JPEG);
            break;
        case OV2640_320x240:
            ArduCAM_wrSensorRegs8_8(cam, OV2640_320x240_JPEG);
            break;
        case OV2640_352x288:
            ArduCAM_wrSensorRegs8_8(cam, OV2640_352x288_JPEG);
            break;
        case OV2640_640x480:
            ArduCAM_wrSensorRegs8_8(cam, OV2640_640x480_JPEG);
            break;
        case OV2640_800x600:
            ArduCAM_wrSensorRegs8_8(cam, OV2640_800x600_JPEG);
            break;
        case OV2640_1024x768:
            ArduCAM_wrSensorRegs8_8(cam, OV2640_1024x768_JPEG);
            break;
        case OV2640_1280x1024:
            ArduCAM_wrSensorRegs8_8(cam, OV2640_1280x1024_JPEG);
            break;
        case OV2640_1600x1200:
            ArduCAM_wrSensorRegs8_8(cam, OV2640_1600x1200_JPEG);
            break;
        default:
            ArduCAM_wrSensorRegs8_8(cam, OV2640_320x240_JPEG);
            break;
    }
}

// 데이터 포맷 설정 함수
void ArduCAM_set_format(ArduCAM* cam, uint8_t fmt) {
    if (fmt == BMP) {
        cam->m_fmt = BMP;
    } else if (fmt == RAW) {
        cam->m_fmt = RAW;
    } else {
        cam->m_fmt = JPEG;
    }
}

// UART RX 인터럽트 핸들러
unsigned char usart_symbol = 0;
unsigned char usart_Command = 0;

void on_uart_rx() {
    while (uart_is_readable(UART_ID)) {
        usart_Command = uart_getc(UART_ID);
        usart_symbol = 1;
    }
}

// ArduCAM 초기화 함수
void ArduCAM_init(void) {
    // I2C 초기화
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);
    bi_decl(bi_2pins_with_func(PIN_SDA, PIN_SCL, GPIO_FUNC_I2C));

    // SPI 초기화
    spi_init(SPI_PORT, 4 * 1000 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    // make the SPI pins available to picotool
    bi_decl(bi_3pins_with_func(PIN_MISO, PIN_MOSI, PIN_SCK, GPIO_FUNC_SPI));
}

// OV5642 JPEG 사이즈 설정 함수
void ArduCAM_OV5642_set_JPEG_size(ArduCAM* cam, uint8_t size) {
    switch(size) {
        case OV5642_320x240:
            ArduCAM_wrSensorRegs16_8(cam, ov5642_320x240);
            break;
        case OV5642_640x480:
            ArduCAM_wrSensorRegs16_8(cam, ov5642_640x480);
            break;
        case OV5642_1024x768:
            ArduCAM_wrSensorRegs16_8(cam, ov5642_1024x768);
            break;
        case OV5642_1280x960:
            ArduCAM_wrSensorRegs16_8(cam, ov5642_1280x960);
            break;
        case OV5642_1600x1200:
            ArduCAM_wrSensorRegs16_8(cam, ov5642_1600x1200);
            break;
        case OV5642_2048x1536:
            ArduCAM_wrSensorRegs16_8(cam, ov5642_2048x1536);
            break;
        case OV5642_2592x1944:
            ArduCAM_wrSensorRegs16_8(cam, ov5642_2592x1944);
            break;
        default:
            ArduCAM_wrSensorRegs16_8(cam, ov5642_320x240);
            break;
    }
}

void ArduCAM_OV5642_set_Light_Mode(ArduCAM* cam, uint8_t Light_Mode) {
    switch(Light_Mode) {
        case Advanced_AWB:
            ArduCAM_wrSensorReg16_8(cam, 0x3406, 0x00);
            ArduCAM_wrSensorReg16_8(cam, 0x5192, 0x04);
            ArduCAM_wrSensorReg16_8(cam, 0x5191, 0xf8);
            ArduCAM_wrSensorReg16_8(cam, 0x518d, 0x26);
            ArduCAM_wrSensorReg16_8(cam, 0x518f, 0x42);
            ArduCAM_wrSensorReg16_8(cam, 0x518e, 0x2b);
            ArduCAM_wrSensorReg16_8(cam, 0x5190, 0x42);
            ArduCAM_wrSensorReg16_8(cam, 0x518b, 0xd0);
            ArduCAM_wrSensorReg16_8(cam, 0x518c, 0xbd);
            ArduCAM_wrSensorReg16_8(cam, 0x5187, 0x18);
            ArduCAM_wrSensorReg16_8(cam, 0x5188, 0x18);
            ArduCAM_wrSensorReg16_8(cam, 0x5189, 0x56);
            ArduCAM_wrSensorReg16_8(cam, 0x518a, 0x5c);
            ArduCAM_wrSensorReg16_8(cam, 0x5186, 0x1c);
            ArduCAM_wrSensorReg16_8(cam, 0x5181, 0x50);
            ArduCAM_wrSensorReg16_8(cam, 0x5184, 0x20);
            ArduCAM_wrSensorReg16_8(cam, 0x5182, 0x11);
            ArduCAM_wrSensorReg16_8(cam, 0x5183, 0x00);
            break;
        case Simple_AWB:
            ArduCAM_wrSensorReg16_8(cam, 0x3406, 0x00);
            ArduCAM_wrSensorReg16_8(cam, 0x5183, 0x80);
            ArduCAM_wrSensorReg16_8(cam, 0x5191, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5192, 0x00);
            break;
        case Manual_day:
            ArduCAM_wrSensorReg16_8(cam, 0x3406, 0x01);
            ArduCAM_wrSensorReg16_8(cam, 0x3400, 0x07);
            ArduCAM_wrSensorReg16_8(cam, 0x3401, 0x32);
            ArduCAM_wrSensorReg16_8(cam, 0x3402, 0x04);
            ArduCAM_wrSensorReg16_8(cam, 0x3403, 0x00);
            ArduCAM_wrSensorReg16_8(cam, 0x3404, 0x05);
            ArduCAM_wrSensorReg16_8(cam, 0x3405, 0x36);
            break;
        case Manual_A:
            ArduCAM_wrSensorReg16_8(cam, 0x3406, 0x01);
            ArduCAM_wrSensorReg16_8(cam, 0x3400, 0x04);
            ArduCAM_wrSensorReg16_8(cam, 0x3401, 0x88);
            ArduCAM_wrSensorReg16_8(cam, 0x3402, 0x04);
            ArduCAM_wrSensorReg16_8(cam, 0x3403, 0x00);
            ArduCAM_wrSensorReg16_8(cam, 0x3404, 0x08);
            ArduCAM_wrSensorReg16_8(cam, 0x3405, 0xb6);
            break;
        case Manual_cwf:
            ArduCAM_wrSensorReg16_8(cam, 0x3406, 0x01);
            ArduCAM_wrSensorReg16_8(cam, 0x3400, 0x06);
            ArduCAM_wrSensorReg16_8(cam, 0x3401, 0x13);
            ArduCAM_wrSensorReg16_8(cam, 0x3402, 0x04);
            ArduCAM_wrSensorReg16_8(cam, 0x3403, 0x00);
            ArduCAM_wrSensorReg16_8(cam, 0x3404, 0x07);
            ArduCAM_wrSensorReg16_8(cam, 0x3405, 0xe2);
            break;
        case Manual_cloudy:
            ArduCAM_wrSensorReg16_8(cam, 0x3406, 0x01);
            ArduCAM_wrSensorReg16_8(cam, 0x3400, 0x07);
            ArduCAM_wrSensorReg16_8(cam, 0x3401, 0x88);
            ArduCAM_wrSensorReg16_8(cam, 0x3402, 0x04);
            ArduCAM_wrSensorReg16_8(cam, 0x3403, 0x00);
            ArduCAM_wrSensorReg16_8(cam, 0x3404, 0x05);
            ArduCAM_wrSensorReg16_8(cam, 0x3405, 0x00);
            break;
        default:
            break;
    }
}



void ArduCAM_OV5642_set_Brightness(ArduCAM* cam, uint8_t Brightness) {
    switch(Brightness) {
        case Brightness4:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5589, 0x40);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x04);
            ArduCAM_wrSensorReg16_8(cam, 0x558a, 0x00);
            break;
        case Brightness3:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5589, 0x30);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x04);
            ArduCAM_wrSensorReg16_8(cam, 0x558a, 0x00);
            break;
        case Brightness2:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5589, 0x20);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x04);
            ArduCAM_wrSensorReg16_8(cam, 0x558a, 0x00);
            break;
        case Brightness1:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5589, 0x10);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x04);
            ArduCAM_wrSensorReg16_8(cam, 0x558a, 0x00);
            break;
        case Brightness0:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5589, 0x00);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x04);
            ArduCAM_wrSensorReg16_8(cam, 0x558a, 0x00);
            break;
        case Brightness_1:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5589, 0x10);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x04);
            ArduCAM_wrSensorReg16_8(cam, 0x558a, 0x08);
            break;
        case Brightness_2:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5589, 0x20);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x04);
            ArduCAM_wrSensorReg16_8(cam, 0x558a, 0x08);
            break;
        case Brightness_3:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5589, 0x30);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x04);
            ArduCAM_wrSensorReg16_8(cam, 0x558a, 0x08);
            break;
        case Brightness_4:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5589, 0x40);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x04);
            ArduCAM_wrSensorReg16_8(cam, 0x558a, 0x08);
            break;
    }
}

void ArduCAM_OV5642_set_Contrast(ArduCAM* cam, uint8_t Contrast) {
    switch(Contrast) {
        case Contrast4:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x04);
            ArduCAM_wrSensorReg16_8(cam, 0x5587, 0x30);
            ArduCAM_wrSensorReg16_8(cam, 0x5588, 0x30);
            ArduCAM_wrSensorReg16_8(cam, 0x558a, 0x00);
            break;
        case Contrast3:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x04);
            ArduCAM_wrSensorReg16_8(cam, 0x5587, 0x2c);
            ArduCAM_wrSensorReg16_8(cam, 0x5588, 0x2c);
            ArduCAM_wrSensorReg16_8(cam, 0x558a, 0x00);
            break;
        case Contrast2:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x04);
            ArduCAM_wrSensorReg16_8(cam, 0x5587, 0x28);
            ArduCAM_wrSensorReg16_8(cam, 0x5588, 0x28);
            ArduCAM_wrSensorReg16_8(cam, 0x558a, 0x00);
            break;
        case Contrast1:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x04);
            ArduCAM_wrSensorReg16_8(cam, 0x5587, 0x24);
            ArduCAM_wrSensorReg16_8(cam, 0x5588, 0x24);
            ArduCAM_wrSensorReg16_8(cam, 0x558a, 0x00);
            break;
        case Contrast0:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x04);
            ArduCAM_wrSensorReg16_8(cam, 0x5587, 0x20);
            ArduCAM_wrSensorReg16_8(cam, 0x5588, 0x20);
            ArduCAM_wrSensorReg16_8(cam, 0x558a, 0x00);
            break;
        case Contrast_1:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x04);
            ArduCAM_wrSensorReg16_8(cam, 0x5587, 0x1C);
            ArduCAM_wrSensorReg16_8(cam, 0x5588, 0x1C);
            ArduCAM_wrSensorReg16_8(cam, 0x558a, 0x00);
            break;
        case Contrast_2:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x04);
            ArduCAM_wrSensorReg16_8(cam, 0x5587, 0x18);
            ArduCAM_wrSensorReg16_8(cam, 0x5588, 0x18);
            ArduCAM_wrSensorReg16_8(cam, 0x558a, 0x00);
            break;
        case Contrast_3:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x04);
            ArduCAM_wrSensorReg16_8(cam, 0x5587, 0x14);
            ArduCAM_wrSensorReg16_8(cam, 0x5588, 0x14);
            ArduCAM_wrSensorReg16_8(cam, 0x558a, 0x00);
            break;
        case Contrast_4:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x04);
            ArduCAM_wrSensorReg16_8(cam, 0x5587, 0x10);
            ArduCAM_wrSensorReg16_8(cam, 0x5588, 0x10);
            ArduCAM_wrSensorReg16_8(cam, 0x558a, 0x00);
            break;
    }
}

void ArduCAM_OV5642_set_hue(ArduCAM* cam, uint8_t degree) {
    switch(degree) {
        case degree_180:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x01);
            ArduCAM_wrSensorReg16_8(cam, 0x5581, 0x80);
            ArduCAM_wrSensorReg16_8(cam, 0x5582, 0x00);
            ArduCAM_wrSensorReg16_8(cam, 0x558a, 0x32);
            break;
        case degree_150:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x01);
            ArduCAM_wrSensorReg16_8(cam, 0x5581, 0x6f);
            ArduCAM_wrSensorReg16_8(cam, 0x5582, 0x40);
            ArduCAM_wrSensorReg16_8(cam, 0x558a, 0x32);
            break;
        case degree_120:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x01);
            ArduCAM_wrSensorReg16_8(cam, 0x5581, 0x40);
            ArduCAM_wrSensorReg16_8(cam, 0x5582, 0x6f);
            ArduCAM_wrSensorReg16_8(cam, 0x558a, 0x32);
            break;
        case degree_90:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x01);
            ArduCAM_wrSensorReg16_8(cam, 0x5581, 0x00);
            ArduCAM_wrSensorReg16_8(cam, 0x5582, 0x80);
            ArduCAM_wrSensorReg16_8(cam, 0x558a, 0x02);
            break;
        case degree_60:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x01);
            ArduCAM_wrSensorReg16_8(cam, 0x5581, 0x40);
            ArduCAM_wrSensorReg16_8(cam, 0x5582, 0x6f);
            ArduCAM_wrSensorReg16_8(cam, 0x558a, 0x02);
            break;
        case degree_30:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x01);
            ArduCAM_wrSensorReg16_8(cam, 0x5581, 0x6f);
            ArduCAM_wrSensorReg16_8(cam, 0x5582, 0x40);
            ArduCAM_wrSensorReg16_8(cam, 0x558a, 0x02);
            break;
        case degree_0:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x01);
            ArduCAM_wrSensorReg16_8(cam, 0x5581, 0x80);
            ArduCAM_wrSensorReg16_8(cam, 0x5582, 0x00);
            ArduCAM_wrSensorReg16_8(cam, 0x558a, 0x01);
            break;
        case degree30:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x01);
            ArduCAM_wrSensorReg16_8(cam, 0x5581, 0x6f);
            ArduCAM_wrSensorReg16_8(cam, 0x5582, 0x40);
            ArduCAM_wrSensorReg16_8(cam, 0x558a, 0x01);
            break;
        case degree60:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x01);
            ArduCAM_wrSensorReg16_8(cam, 0x5581, 0x40);
            ArduCAM_wrSensorReg16_8(cam, 0x5582, 0x6f);
            ArduCAM_wrSensorReg16_8(cam, 0x558a, 0x01);
            break;
        case degree90:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x01);
            ArduCAM_wrSensorReg16_8(cam, 0x5581, 0x00);
            ArduCAM_wrSensorReg16_8(cam, 0x5582, 0x80);
            ArduCAM_wrSensorReg16_8(cam, 0x558a, 0x31);
            break;
        case degree120:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x01);
            ArduCAM_wrSensorReg16_8(cam, 0x5581, 0x40);
            ArduCAM_wrSensorReg16_8(cam, 0x5582, 0x6f);
            ArduCAM_wrSensorReg16_8(cam, 0x558a, 0x31);
            break;
        case degree150:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x01);
            ArduCAM_wrSensorReg16_8(cam, 0x5581, 0x6f);
            ArduCAM_wrSensorReg16_8(cam, 0x5582, 0x40);
            ArduCAM_wrSensorReg16_8(cam, 0x558a, 0x31);
            break;
    }
}

void ArduCAM_OV5642_set_Special_effects(ArduCAM* cam, uint8_t Special_effect) {
    switch(Special_effect) {
        case Bluish:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x18);
            ArduCAM_wrSensorReg16_8(cam, 0x5585, 0xa0);
            ArduCAM_wrSensorReg16_8(cam, 0x5586, 0x40);
            break;
        case Greenish:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x18);
            ArduCAM_wrSensorReg16_8(cam, 0x5585, 0x60);
            ArduCAM_wrSensorReg16_8(cam, 0x5586, 0x60);
            break;
        case Reddish:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x18);
            ArduCAM_wrSensorReg16_8(cam, 0x5585, 0x80);
            ArduCAM_wrSensorReg16_8(cam, 0x5586, 0xc0);
            break;
        case BW:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x18);
            ArduCAM_wrSensorReg16_8(cam, 0x5585, 0x80);
            ArduCAM_wrSensorReg16_8(cam, 0x5586, 0x80);
            break;
        case Negative:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x40);
            break;
        case Sepia:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0xff);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x18);
            ArduCAM_wrSensorReg16_8(cam, 0x5585, 0x40);
            ArduCAM_wrSensorReg16_8(cam, 0x5586, 0xa0);
            break;
        case Normal:
            ArduCAM_wrSensorReg16_8(cam, 0x5001, 0x7f);
            ArduCAM_wrSensorReg16_8(cam, 0x5580, 0x00);
            break;
    }
}




void ArduCAM_OV5642_set_Exposure_level(ArduCAM* cam, uint8_t level) {
    switch(level) {
        case Exposure_17_EV:
            ArduCAM_wrSensorReg16_8(cam, 0x3a0f, 0x10);
            ArduCAM_wrSensorReg16_8(cam, 0x3a10, 0x08);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1b, 0x10);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1e, 0x08);
            ArduCAM_wrSensorReg16_8(cam, 0x3a11, 0x20);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1f, 0x10);
            break;
        case Exposure_13_EV:
            ArduCAM_wrSensorReg16_8(cam, 0x3a0f, 0x18);
            ArduCAM_wrSensorReg16_8(cam, 0x3a10, 0x10);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1b, 0x18);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1e, 0x10);
            ArduCAM_wrSensorReg16_8(cam, 0x3a11, 0x30);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1f, 0x10);
            break;
        case Exposure_10_EV:
            ArduCAM_wrSensorReg16_8(cam, 0x3a0f, 0x20);
            ArduCAM_wrSensorReg16_8(cam, 0x3a10, 0x18);
            ArduCAM_wrSensorReg16_8(cam, 0x3a11, 0x41);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1b, 0x20);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1e, 0x18);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1f, 0x10);
            break;
        case Exposure_07_EV:
            ArduCAM_wrSensorReg16_8(cam, 0x3a0f, 0x28);
            ArduCAM_wrSensorReg16_8(cam, 0x3a10, 0x20);
            ArduCAM_wrSensorReg16_8(cam, 0x3a11, 0x51);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1b, 0x28);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1e, 0x20);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1f, 0x10);
            break;
        case Exposure_03_EV:
            ArduCAM_wrSensorReg16_8(cam, 0x3a0f, 0x30);
            ArduCAM_wrSensorReg16_8(cam, 0x3a10, 0x28);
            ArduCAM_wrSensorReg16_8(cam, 0x3a11, 0x61);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1b, 0x30);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1e, 0x28);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1f, 0x10);
            break;
        case Exposure_default:
            ArduCAM_wrSensorReg16_8(cam, 0x3a0f, 0x38);
            ArduCAM_wrSensorReg16_8(cam, 0x3a10, 0x30);
            ArduCAM_wrSensorReg16_8(cam, 0x3a11, 0x61);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1b, 0x38);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1e, 0x30);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1f, 0x10);
            break;
        case Exposure03_EV:
            ArduCAM_wrSensorReg16_8(cam, 0x3a0f, 0x40);
            ArduCAM_wrSensorReg16_8(cam, 0x3a10, 0x38);
            ArduCAM_wrSensorReg16_8(cam, 0x3a11, 0x71);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1b, 0x40);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1e, 0x38);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1f, 0x10);
            break;
        case Exposure07_EV:
            ArduCAM_wrSensorReg16_8(cam, 0x3a0f, 0x48);
            ArduCAM_wrSensorReg16_8(cam, 0x3a10, 0x40);
            ArduCAM_wrSensorReg16_8(cam, 0x3a11, 0x80);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1b, 0x48);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1e, 0x40);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1f, 0x20);
            break;
        case Exposure10_EV:
            ArduCAM_wrSensorReg16_8(cam, 0x3a0f, 0x50);
            ArduCAM_wrSensorReg16_8(cam, 0x3a10, 0x48);
            ArduCAM_wrSensorReg16_8(cam, 0x3a11, 0x90);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1b, 0x50);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1e, 0x48);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1f, 0x20);
            break;
        case Exposure13_EV:
            ArduCAM_wrSensorReg16_8(cam, 0x3a0f, 0x58);
            ArduCAM_wrSensorReg16_8(cam, 0x3a10, 0x50);
            ArduCAM_wrSensorReg16_8(cam, 0x3a11, 0x91);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1b, 0x58);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1e, 0x50);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1f, 0x20);
            break;
        case Exposure17_EV:
            ArduCAM_wrSensorReg16_8(cam, 0x3a0f, 0x60);
            ArduCAM_wrSensorReg16_8(cam, 0x3a10, 0x58);
            ArduCAM_wrSensorReg16_8(cam, 0x3a11, 0xa0);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1b, 0x60);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1e, 0x58);
            ArduCAM_wrSensorReg16_8(cam, 0x3a1f, 0x20);
            break;
    }
}


void ArduCAM_OV5642_set_Sharpness(ArduCAM* cam, uint8_t Sharpness) {
    switch(Sharpness) {
        case Auto_Sharpness_default:
            ArduCAM_wrSensorReg16_8(cam, 0x530A, 0x00);
            ArduCAM_wrSensorReg16_8(cam, 0x530c, 0x00);
            ArduCAM_wrSensorReg16_8(cam, 0x530d, 0x0c);
            ArduCAM_wrSensorReg16_8(cam, 0x5312, 0x40);
            break;
        case Auto_Sharpness1:
            ArduCAM_wrSensorReg16_8(cam, 0x530A, 0x00);
            ArduCAM_wrSensorReg16_8(cam, 0x530c, 0x04);
            ArduCAM_wrSensorReg16_8(cam, 0x530d, 0x18);
            ArduCAM_wrSensorReg16_8(cam, 0x5312, 0x20);
            break;
        case Auto_Sharpness2:
            ArduCAM_wrSensorReg16_8(cam, 0x530A, 0x00);
            ArduCAM_wrSensorReg16_8(cam, 0x530c, 0x08);
            ArduCAM_wrSensorReg16_8(cam, 0x530d, 0x30);
            ArduCAM_wrSensorReg16_8(cam, 0x5312, 0x10);
            break;
        case Manual_Sharpnessoff:
            ArduCAM_wrSensorReg16_8(cam, 0x530A, 0x08);
            ArduCAM_wrSensorReg16_8(cam, 0x531e, 0x00);
            ArduCAM_wrSensorReg16_8(cam, 0x531f, 0x00);
            break;
        case Manual_Sharpness1:
            ArduCAM_wrSensorReg16_8(cam, 0x530A, 0x08);
            ArduCAM_wrSensorReg16_8(cam, 0x531e, 0x04);
            ArduCAM_wrSensorReg16_8(cam, 0x531f, 0x04);
            break;
        case Manual_Sharpness2:
            ArduCAM_wrSensorReg16_8(cam, 0x530A, 0x08);
            ArduCAM_wrSensorReg16_8(cam, 0x531e, 0x08);
            ArduCAM_wrSensorReg16_8(cam, 0x531f, 0x08);
            break;
        case Manual_Sharpness3:
            ArduCAM_wrSensorReg16_8(cam, 0x530A, 0x08);
            ArduCAM_wrSensorReg16_8(cam, 0x531e, 0x0c);
            ArduCAM_wrSensorReg16_8(cam, 0x531f, 0x0c);
            break;
        case Manual_Sharpness4:
            ArduCAM_wrSensorReg16_8(cam, 0x530A, 0x08);
            ArduCAM_wrSensorReg16_8(cam, 0x531e, 0x0f);
            ArduCAM_wrSensorReg16_8(cam, 0x531f, 0x0f);
            break;
        case Manual_Sharpness5:
            ArduCAM_wrSensorReg16_8(cam, 0x530A, 0x08);
            ArduCAM_wrSensorReg16_8(cam, 0x531e, 0x1f);
            ArduCAM_wrSensorReg16_8(cam, 0x531f, 0x1f);
            break;
    }
}


void ArduCAM_OV5642_set_Mirror_Flip(ArduCAM* cam, uint8_t Mirror_Flip) {
    uint8_t reg_val;
    switch(Mirror_Flip) {
        case MIRROR:
            ArduCAM_rdSensorReg16_8(cam, 0x3818, &reg_val);
            reg_val = reg_val & 0x9F;
            ArduCAM_wrSensorReg16_8(cam, 0x3818, reg_val);
            ArduCAM_rdSensorReg16_8(cam, 0x3621, &reg_val);
            reg_val = reg_val | 0x20;
            ArduCAM_wrSensorReg16_8(cam, 0x3621, reg_val);
            break;
        case FLIP:
            ArduCAM_rdSensorReg16_8(cam, 0x3818, &reg_val);
            reg_val = reg_val | 0x20;
            reg_val = reg_val & 0xBF;
            ArduCAM_wrSensorReg16_8(cam, 0x3818, reg_val);
            ArduCAM_rdSensorReg16_8(cam, 0x3621, &reg_val);
            reg_val = reg_val | 0x20;
            ArduCAM_wrSensorReg16_8(cam, 0x3621, reg_val);
            break;
        case MIRROR_FLIP:
            ArduCAM_rdSensorReg16_8(cam, 0x3818, &reg_val);
            reg_val = reg_val | 0x60;
            reg_val = reg_val & 0xFF;
            ArduCAM_wrSensorReg16_8(cam, 0x3818, reg_val);
            ArduCAM_rdSensorReg16_8(cam, 0x3621, &reg_val);
            reg_val = reg_val & 0xDF;
            ArduCAM_wrSensorReg16_8(cam, 0x3621, reg_val);
            break;
        case Normal:
            ArduCAM_rdSensorReg16_8(cam, 0x3818, &reg_val);
            reg_val = reg_val & 0xDF;
            ArduCAM_wrSensorReg16_8(cam, 0x3818, reg_val);
            ArduCAM_rdSensorReg16_8(cam, 0x3621, &reg_val);
            reg_val = reg_val & 0xDF;
            ArduCAM_wrSensorReg16_8(cam, 0x3621, reg_val);
            break;
    }
}


void ArduCAM_OV5642_set_Compress_quality(ArduCAM* cam, uint8_t quality) {
    switch(quality) {
        case high_quality:
            ArduCAM_wrSensorReg16_8(cam, 0x4407, 0x02);
            break;
        case default_quality:
            ArduCAM_wrSensorReg16_8(cam, 0x4407, 0x04);
            break;
        case low_quality:
            ArduCAM_wrSensorReg16_8(cam, 0x4407, 0x08);
            break;
    }
}


void ArduCAM_OV5642_Test_Pattern(ArduCAM* cam, uint8_t Pattern) {
    switch(Pattern) {
        case Color_bar:
            ArduCAM_wrSensorReg16_8(cam, 0x503d, 0x80);
            ArduCAM_wrSensorReg16_8(cam, 0x503e, 0x00);
            break;
        case Color_square:
            ArduCAM_wrSensorReg16_8(cam, 0x503d, 0x85);
            ArduCAM_wrSensorReg16_8(cam, 0x503e, 0x12);
            break;
        case BW_square:
            ArduCAM_wrSensorReg16_8(cam, 0x503d, 0x85);
            ArduCAM_wrSensorReg16_8(cam, 0x503e, 0x1a);
            break;
        case DLI:
            ArduCAM_wrSensorReg16_8(cam, 0x4741, 0x04);
            break;
    }
}
