#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/clocks.h"
#include "ArduCAM.h"
#include "hardware/irq.h"
#include "ov2640_regs.h"
#include "tusb.h"
#include "pico/mutex.h"

static mutex_t usb_mutex;

void SerialUsb(uint8_t* buffer, uint32_t length);
int SerialUSBAvailable(void);
int SerialUsbRead(void);

#define BMPIMAGEOFFSET 66
uint8_t bmp_header[BMPIMAGEOFFSET] = {
    0x42, 0x4D, 0x36, 0x58, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 0x00, 0x28, 0x00,
    0x00, 0x00, 0x40, 0x01, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x01, 0x00, 0x10, 0x00, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x58, 0x02, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0xE0, 0x07, 0x00, 0x00, 0x1F, 0x00,
    0x00, 0x00
};

const uint8_t CS = 9;
bool is_header = false;
int mode = 0;
uint8_t start_capture = 0;

ArduCAM myCAM;

void read_fifo_burst(ArduCAM* cam);

int main() {
    uint8_t vid, pid;
    uint8_t cameraCommand;
    stdio_init_all();
    tusb_init();
    sleep_ms(3000);

    // Initialize ArduCAM
    ArduCAM_Init_Model(&myCAM);  // 기본 초기화
    ArduCAM_InitWithModelAndCS(&myCAM, OV2640, CS);  // 모델과 CS 지정하여 초기화
    ArduCAM_init();
    



    gpio_init(CS);
    gpio_set_dir(CS, GPIO_OUT);
    gpio_put(CS, 1);

    // Reset the CPLD
    ArduCAM_write_reg(&myCAM, 0x07, 0x80);
    sleep_ms(100);
    ArduCAM_write_reg(&myCAM, 0x07, 0x00);
    sleep_ms(100);

    // Check SPI bus
    while (1) {

        ArduCAM_write_reg(&myCAM, ARDUCHIP_TEST1, 0x55);
        
        cameraCommand = ArduCAM_read_reg(&myCAM, ARDUCHIP_TEST1);
        printf("TEST RESULT %02X\r\n", cameraCommand);
        if (cameraCommand != 0x55) {
            printf("SPI interface Error!");
            sleep_ms(1000);
            continue;
        } else {
            printf("ACK CMD SPI interface OK.END");
            break;
        }
    }

    // Check if the camera module type is OV2640
    while (1) {
        ArduCAM_wrSensorReg8_8(&myCAM, 0xff, 0x01);
        ArduCAM_rdSensorReg8_8(&myCAM, OV2640_CHIPID_HIGH, &vid);
        ArduCAM_rdSensorReg8_8(&myCAM, OV2640_CHIPID_LOW, &pid);
        if ((vid != 0x26) && ((pid != 0x41) || (pid != 0x42))) {
            printf("Can't find OV2640 module!");
            sleep_ms(1000);
            continue;
        } else {
            printf("OV2640 detected.END");
            break;
        }
    }

    // Change to JPEG capture mode and initialize the OV2640 module
    ArduCAM_set_format(&myCAM, JPEG);
    ArduCAM_InitCAM(&myCAM);
    ArduCAM_OV2640_set_JPEG_size(&myCAM, OV2640_320x240);
    sleep_ms(1000);
    ArduCAM_clear_fifo_flag(&myCAM);

    while (1) {
        if (SerialUSBAvailable()) {
            uint8_t usart_Command = SerialUsbRead();
            switch (usart_Command) {
                case 0:
                    ArduCAM_OV2640_set_JPEG_size(&myCAM, OV2640_160x120);
                    sleep_ms(1000);
                    printf("ACK CMD switch to OV2640_160x120 END");
                    break;
                case 1:
                    ArduCAM_OV2640_set_JPEG_size(&myCAM, OV2640_176x144);
                    sleep_ms(1000);
                    printf("ACK CMD switch to OV2640_176x144 END");
                    break;
                case 2:
                    ArduCAM_OV2640_set_JPEG_size(&myCAM, OV2640_320x240);
                    sleep_ms(1000);
                    printf("ACK CMD switch to OV2640_320x240 END");
                    break;
                case 3:
                    ArduCAM_OV2640_set_JPEG_size(&myCAM, OV2640_352x288);
                    sleep_ms(1000);
                    printf("ACK CMD switch to OV2640_352x288 END");
                    break;
                case 4:
                    ArduCAM_OV2640_set_JPEG_size(&myCAM, OV2640_640x480);
                    sleep_ms(1000);
                    printf("ACK CMD switch to OV2640_640x480 END");
                    break;
                case 5:
                    ArduCAM_OV2640_set_JPEG_size(&myCAM, OV2640_800x600);
                    sleep_ms(1000);
                    printf("ACK CMD switch to OV2640_800x600 END");
                    break;
                case 6:
                    ArduCAM_OV2640_set_JPEG_size(&myCAM, OV2640_1024x768);
                    sleep_ms(1000);
                    printf("ACK CMD switch to OV2640_1024x768 END");
                    break;
                case 7:
                    ArduCAM_OV2640_set_JPEG_size(&myCAM, OV2640_1280x1024);
                    sleep_ms(1000);
                    printf("ACK CMD switch to OV2640_1280x1024 END");
                    break;
                case 8:
                    ArduCAM_OV2640_set_JPEG_size(&myCAM, OV2640_1600x1200);
                    sleep_ms(1000);
                    printf("ACK CMD switch to OV2640_1600x1200 END");
                    break;
                case 0x10:
                    mode = 1;
                    start_capture = 1;
                    printf("ACK CMD CAM start single shoot. END");
                    break;
                case 0x11:
                    ArduCAM_set_format(&myCAM, JPEG);
                    ArduCAM_InitCAM(&myCAM);
                    break;
                case 0x20:
                    mode = 2;
                    start_capture = 2;
                    printf("ACK CMD CAM start video streaming. END");
                    break;
                case 0x30:
                    mode = 3;
                    start_capture = 3;
                    printf("ACK CMD CAM start single shoot. END");
                    break;
                case 0x31:
                    ArduCAM_set_format(&myCAM, BMP);
                    ArduCAM_InitCAM(&myCAM);
                    ArduCAM_wrSensorReg16_8(&myCAM, 0x3818, 0x81);
                    ArduCAM_wrSensorReg16_8(&myCAM, 0x3621, 0xA7);
                    break;
                case 0x40:
                    ArduCAM_OV2640_set_Light_Mode(&myCAM, Auto);
                    printf("ACK CMD Set to Auto END");
                    break;
                case 0x41:
                    ArduCAM_OV2640_set_Light_Mode(&myCAM, Sunny);
                    printf("ACK CMD Set to Sunny END");
                    break;
                case 0x42:
                    ArduCAM_OV2640_set_Light_Mode(&myCAM, Cloudy);
                    printf("ACK CMD Set to Cloudy END");
                    break;
                case 0x43:
                    ArduCAM_OV2640_set_Light_Mode(&myCAM, Office);
                    printf("ACK CMD Set to Office END");
                    break;
                case 0x44:
                    ArduCAM_OV2640_set_Light_Mode(&myCAM, Home);
                    printf("ACK CMD Set to Home END");
                    break;
                case 0x50:
                    ArduCAM_OV2640_set_Color_Saturation(&myCAM, Saturation2);
                    printf("ACK CMD Set to Saturation+2 END");
                    break;
                case 0x51:
                    ArduCAM_OV2640_set_Color_Saturation(&myCAM, Saturation1);
                    printf("ACK CMD Set to Saturation+1 END");
                    break;
                case 0x52:
                    ArduCAM_OV2640_set_Color_Saturation(&myCAM, Saturation0);
                    printf("ACK CMD Set to Saturation+0 END");
                    break;
                case 0x53:
                    ArduCAM_OV2640_set_Color_Saturation(&myCAM, Saturation_1);
                    printf("ACK CMD Set to Saturation-1 END");
                    break;
                case 0x54:
                    ArduCAM_OV2640_set_Color_Saturation(&myCAM, Saturation_2);
                    printf("ACK CMD Set to Saturation-2 END");
                    break;
                case 0x60:
                    ArduCAM_OV2640_set_Brightness(&myCAM, Brightness2);
                    printf("ACK CMD Set to Brightness+2 END");
                    break;
                case 0x61:
                    ArduCAM_OV2640_set_Brightness(&myCAM, Brightness1);
                    printf("ACK CMD Set to Brightness+1 END");
                    break;
                case 0x62:
                    ArduCAM_OV2640_set_Brightness(&myCAM, Brightness0);
                    printf("ACK CMD Set to Brightness+0 END");
                    break;
                case 0x63:
                    ArduCAM_OV2640_set_Brightness(&myCAM, Brightness_1);
                    printf("ACK CMD Set to Brightness-1 END");
                    break;
                case 0x64:
                    ArduCAM_OV2640_set_Brightness(&myCAM, Brightness_2);
                    printf("ACK CMD Set to Brightness-2 END");
                    break;
                case 0x70:
                    ArduCAM_OV2640_set_Contrast(&myCAM, Contrast2);
                    printf("ACK CMD Set to Contrast+2 END");
                    break;
                case 0x71:
                    ArduCAM_OV2640_set_Contrast(&myCAM, Contrast1);
                    printf("ACK CMD Set to Contrast+1 END");
                    break;
                case 0x72:
                    ArduCAM_OV2640_set_Contrast(&myCAM, Contrast0);
                    printf("ACK CMD Set to Contrast+0 END");
                    break;
                case 0x73:
                    ArduCAM_OV2640_set_Contrast(&myCAM, Contrast_1);
                    printf("ACK CMD Set to Contrast-1 END");
                    break;
                case 0x74:
                    ArduCAM_OV2640_set_Contrast(&myCAM, Contrast_2);
                    printf("ACK CMD Set to Contrast-2 END");
                    break;
                case 0x80:
                    ArduCAM_OV2640_set_Special_effects(&myCAM, Antique);
                    printf("ACK CMD Set to Antique END");
                    break;
                                case 0x81:
                    ArduCAM_OV2640_set_Special_effects(&myCAM, Bluish);
                    printf("ACK CMD Set to Bluish END");
                    break;
                case 0x82:
                    ArduCAM_OV2640_set_Special_effects(&myCAM, Greenish);
                    printf("ACK CMD Set to Greenish END");
                    break;
                case 0x83:
                    ArduCAM_OV2640_set_Special_effects(&myCAM, Reddish);
                    printf("ACK CMD Set to Reddish END");
                    break;
                case 0x84:
                    ArduCAM_OV2640_set_Special_effects(&myCAM, BW);
                    printf("ACK CMD Set to BW END");
                    break;
                case 0x85:
                    ArduCAM_OV2640_set_Special_effects(&myCAM, Negative);
                    printf("ACK CMD Set to Negative END");
                    break;
                case 0x86:
                    ArduCAM_OV2640_set_Special_effects(&myCAM, BWnegative);
                    printf("ACK CMD Set to BWnegative END");
                    break;
                case 0x87:
                    ArduCAM_OV2640_set_Special_effects(&myCAM, Normal);
                    printf("ACK CMD Set to Normal END");
                    break;
            }
        }

        if (mode == 1) {
            if (start_capture == 1) {
                ArduCAM_flush_fifo(&myCAM);
                ArduCAM_clear_fifo_flag(&myCAM);
                ArduCAM_start_capture(&myCAM);
                start_capture = 0;
            }
            if (ArduCAM_get_bit(&myCAM, ARDUCHIP_TRIG, CAP_DONE_MASK)) {
                printf("ACK CMD CAM Capture Done. END");
                read_fifo_burst(&myCAM);
                ArduCAM_clear_fifo_flag(&myCAM);
            }
        } else if (mode == 2) {
            while (1) {
                if (SerialUSBAvailable()) {
                    uint8_t usart_Command = SerialUsbRead();
                }
                if (usart_Command == 0x21) {
                    start_capture = 0;
                    mode = 0;
                    printf("ACK CMD CAM stop video streaming. END");
                    break;
                }
                switch (usart_Command) {
                    case 0x40:
                        ArduCAM_OV2640_set_Light_Mode(&myCAM, Auto);
                        printf("ACK CMD Set to Auto END");
                        break;
                    case 0x41:
                        ArduCAM_OV2640_set_Light_Mode(&myCAM, Sunny);
                        printf("ACK CMD Set to Sunny END");
                        break;
                    case 0x42:
                        ArduCAM_OV2640_set_Light_Mode(&myCAM, Cloudy);
                        printf("ACK CMD Set to Cloudy END");
                        break;
                    case 0x43:
                        ArduCAM_OV2640_set_Light_Mode(&myCAM, Office);
                        printf("ACK CMD Set to Office END");
                        break;
                    case 0x44:
                        ArduCAM_OV2640_set_Light_Mode(&myCAM, Home);
                        printf("ACK CMD Set to Home END");
                        break;
                    case 0x50:
                        ArduCAM_OV2640_set_Color_Saturation(&myCAM, Saturation2);
                        printf("ACK CMD Set to Saturation+2 END");
                        break;
                    case 0x51:
                        ArduCAM_OV2640_set_Color_Saturation(&myCAM, Saturation1);
                        printf("ACK CMD Set to Saturation+1 END");
                        break;
                    case 0x52:
                        ArduCAM_OV2640_set_Color_Saturation(&myCAM, Saturation0);
                        printf("ACK CMD Set to Saturation+0 END");
                        break;
                    case 0x53:
                        ArduCAM_OV2640_set_Color_Saturation(&myCAM, Saturation_1);
                        printf("ACK CMD Set to Saturation-1 END");
                        break;
                    case 0x54:
                        ArduCAM_OV2640_set_Color_Saturation(&myCAM, Saturation_2);
                        printf("ACK CMD Set to Saturation-2 END");
                        break;
                    case 0x60:
                        ArduCAM_OV2640_set_Brightness(&myCAM, Brightness2);
                        printf("ACK CMD Set to Brightness+2 END");
                        break;
                    case 0x61:
                        ArduCAM_OV2640_set_Brightness(&myCAM, Brightness1);
                        printf("ACK CMD Set to Brightness+1 END");
                        break;
                    case 0x62:
                        ArduCAM_OV2640_set_Brightness(&myCAM, Brightness0);
                        printf("ACK CMD Set to Brightness+0 END");
                        break;
                    case 0x63:
                        ArduCAM_OV2640_set_Brightness(&myCAM, Brightness_1);
                        printf("ACK CMD Set to Brightness-1 END");
                        break;
                    case 0x64:
                        ArduCAM_OV2640_set_Brightness(&myCAM, Brightness_2);
                        printf("ACK CMD Set to Brightness-2 END");
                        break;
                    case 0x70:
                        ArduCAM_OV2640_set_Contrast(&myCAM, Contrast2);
                        printf("ACK CMD Set to Contrast+2 END");
                        break;
                    case 0x71:
                        ArduCAM_OV2640_set_Contrast(&myCAM, Contrast1);
                        printf("ACK CMD Set to Contrast+1 END");
                        break;
                    case 0x72:
                        ArduCAM_OV2640_set_Contrast(&myCAM, Contrast0);
                        printf("ACK CMD Set to Contrast+0 END");
                        break;
                    case 0x73:
                        ArduCAM_OV2640_set_Contrast(&myCAM, Contrast_1);
                        printf("ACK CMD Set to Contrast-1 END");
                        break;
                    case 0x74:
                        ArduCAM_OV2640_set_Contrast(&myCAM, Contrast_2);
                        printf("ACK CMD Set to Contrast-2 END");
                        break;
                    case 0x80:
                        ArduCAM_OV2640_set_Special_effects(&myCAM, Antique);
                        printf("ACK CMD Set to Antique END");
                        break;
                    case 0x81:
                        ArduCAM_OV2640_set_Special_effects(&myCAM, Bluish);
                        printf("ACK CMD Set to Bluish END");
                        break;
                    case 0x82:
                        ArduCAM_OV2640_set_Special_effects(&myCAM, Greenish);
                        printf("ACK CMD Set to Greenish END");
                        break;
                    case 0x83:
                        ArduCAM_OV2640_set_Special_effects(&myCAM, Reddish);
                        printf("ACK CMD Set to Reddish END");
                        break;
                    case 0x84:
                        ArduCAM_OV2640_set_Special_effects(&myCAM, BW);
                        printf("ACK CMD Set to BW END");
                        break;
                    case 0x85:
                        ArduCAM_OV2640_set_Special_effects(&myCAM, Negative);
                        printf("ACK CMD Set to Negative END");
                        break;
                    case 0x86:
                        ArduCAM_OV2640_set_Special_effects(&myCAM, BWnegative);
                        printf("ACK CMD Set to BWnegative END");
                        break;
                    case 0x87:
                        ArduCAM_OV2640_set_Special_effects(&myCAM, Normal);
                        printf("ACK CMD Set to Normal END");
                        break;
                }
                if (start_capture == 2) {
                    ArduCAM_flush_fifo(&myCAM);
                    ArduCAM_clear_fifo_flag(&myCAM);
                    ArduCAM_start_capture(&myCAM);
                    start_capture = 0;
                }
                if (ArduCAM_get_bit(&myCAM, ARDUCHIP_TRIG, CAP_DONE_MASK)) {
                    read_fifo_burst(&myCAM);
                    start_capture = 2;
                }
            }
        } else if (mode == 3) {
            if (start_capture == 3) {
                ArduCAM_flush_fifo(&myCAM);
                ArduCAM_clear_fifo_flag(&myCAM);
                ArduCAM_start_capture(&myCAM);
                start_capture = 0;
            }
            if (ArduCAM_get_bit(&myCAM, ARDUCHIP_TRIG, CAP_DONE_MASK)) {
                printf("ACK CMD CAM Capture Done.");
                uint8_t temp;
                uint32_t length = ArduCAM_read_fifo_length(&myCAM);
                if (length >= MAX_FIFO_SIZE) {
                    printf("ACK CMD Over size.");
                    ArduCAM_clear_fifo_flag(&myCAM);
                    return 0;
                }
                if (length == 0) {
                    printf("ACK CMD Size is 0.");
                    ArduCAM_clear_fifo_flag(&myCAM);
                    return 0;
                }
                uint8_t symbol[2] = {0xff, 0xaa};
                ArduCAM_CS_LOW(&myCAM);
                ArduCAM_set_fifo_burst(&myCAM);
                SerialUsb(symbol, sizeof(symbol));
                SerialUsb(bmp_header, BMPIMAGEOFFSET);
                spi_read_blocking(SPI_PORT, BURST_FIFO_READ, &temp, 1);
                for (int i = 0; i < 240; i++) {
                    for (int j = 0; j < 320; j++) {
                        spi_read_blocking(SPI_PORT, BURST_FIFO_READ, &temp, 1);
                        SerialUsb(&temp, 1);
                        sleep_us(12);
                    }
                }
                symbol[0] = 0xbb;
                symbol[1] = 0xcc;
                SerialUsb(symbol, sizeof(symbol));
                ArduCAM_CS_HIGH(&myCAM);
                ArduCAM_clear_fifo_flag(&myCAM);
            }
        }
    }
    return 0;
}

void read_fifo_burst(ArduCAM* cam) {
    uint32_t length = ArduCAM_read_fifo_length(cam);
    uint8_t* imageBuf = (uint8_t*)malloc(length * sizeof(uint8_t));
    ArduCAM_CS_LOW(cam);
    ArduCAM_set_fifo_burst(cam);
    spi_read_blocking(SPI_PORT, BURST_FIFO_READ, imageBuf, length);
    ArduCAM_CS_HIGH(cam);
    SerialUsb(imageBuf, length);
    free(imageBuf);
}



void SerialUsb(uint8_t* buf,uint32_t length)
{
    static uint64_t last_avail_time;
    int i = 0;
    if (tud_cdc_connected()) 
    {
        for (int i = 0; i < length;) 
        {
            int n = length - i;
            int avail = tud_cdc_write_available();
            if (n > avail) n = avail;
            if (n) 
            {
                int n2 = tud_cdc_write(buf + i, n);
                tud_task();
                tud_cdc_write_flush();
                i += n2;
                last_avail_time = time_us_64();
            } 
            else 
            {
                tud_task();
                tud_cdc_write_flush();
                if (!tud_cdc_connected() ||
                    (!tud_cdc_write_available() && time_us_64() > last_avail_time + 1000000 /* 1 second */)) {
                    break;
                }
            }
        }
    } 
    else 
    {
        // reset our timeout
        last_avail_time = 0;
    }
}

int SerialUSBAvailable(void)
{
  return tud_cdc_available();
} 

int SerialUsbRead(void) 
{
  if (tud_cdc_connected() && tud_cdc_available()) 
  {
    return tud_cdc_read_char();
  }
  return -1;
}