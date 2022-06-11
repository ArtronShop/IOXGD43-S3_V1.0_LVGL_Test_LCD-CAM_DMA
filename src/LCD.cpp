#include "Arduino.h"
#include "PinCofigs.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "soc/lcd_cam_reg.h"
#include "soc/lcd_cam_struct.h"
#include "hal/lcd_ll.h"

#include "lvgl.h"

#define LCD_WIDTH 800
#define LCD_HEIGHT 480
// #define LCD_PIXEL_CLOCK_HZ (400 * 1000)
#define LCD_PIXEL_CLOCK_HZ (5 * 1000 * 1000)
#define LCD_CLOCK_SRC LCD_CLK_SRC_PLL160M
    // .clk_src = LCD_CLK_SRC_XTAL,
    // .clk_src = LCD_CLK_SRC_PLL160M,

#define LCD_ACTIVE() digitalWrite(LCD_CS_PIN, LOW)
#define LCD_INACTIVE() digitalWrite(LCD_CS_PIN, HIGH)

#define LCD_BACKLIGHT_ON() digitalWrite(LCD_BL_PIN, HIGH)
#define LCD_BACKLIGHT_OFF() digitalWrite(LCD_BL_PIN, LOW)

static const uint32_t LCD_DATA_PIN_MASK = ~(
  (1UL << LCD_D0_PIN) |
  (1UL << LCD_D1_PIN) |
  (1UL << LCD_D2_PIN) |
  (1UL << LCD_D3_PIN) |
  (1UL << LCD_D4_PIN) |
  (1UL << LCD_D5_PIN) |
  (1UL << LCD_D6_PIN) |
  (1UL << LCD_D7_PIN)
);

void DirectIOWrite(uint8_t data) {
  GPIO.out = (GPIO.out & LCD_DATA_PIN_MASK) | 
                (
                  (((data >> 0) & 0x01) << LCD_D0_PIN) |
                  (((data >> 1) & 0x01) << LCD_D1_PIN) |
                  (((data >> 2) & 0x01) << LCD_D2_PIN) |
                  (((data >> 3) & 0x01) << LCD_D3_PIN) |
                  (((data >> 4) & 0x01) << LCD_D4_PIN) |
                  (((data >> 5) & 0x01) << LCD_D5_PIN) |
                  (((data >> 6) & 0x01) << LCD_D6_PIN) |
                  (((data >> 7) & 0x01) << LCD_D7_PIN)
                ) |
                (1 << LCD_DE_PIN) // Send data with DE HIGH*/
            ;
  GPIO.out_w1tc = 1 << LCD_DE_PIN; // DE set to LOW
}

void WriteComm(uint8_t data) {
  digitalWrite(LCD_DC_PIN, LOW);
  DirectIOWrite(data);
}

void WriteData(uint8_t data) {
  digitalWrite(LCD_DC_PIN, HIGH);
  DirectIOWrite(data);
}

void SwitchToGPIOMode() {
  pinMode(LCD_D0_PIN, OUTPUT);
  pinMode(LCD_D1_PIN, OUTPUT);
  pinMode(LCD_D2_PIN, OUTPUT);
  pinMode(LCD_D3_PIN, OUTPUT);
  pinMode(LCD_D4_PIN, OUTPUT);
  pinMode(LCD_D5_PIN, OUTPUT);
  pinMode(LCD_D6_PIN, OUTPUT);
  pinMode(LCD_D7_PIN, OUTPUT);

  pinMode(LCD_DE_PIN, OUTPUT);
}

void SwitchToLCDCAMMode() {
  // Data bus
  pinMatrixOutAttach(LCD_D0_PIN, LCD_DATA_OUT0_IDX, false, false);
  pinMatrixOutAttach(LCD_D1_PIN, LCD_DATA_OUT1_IDX, false, false);
  pinMatrixOutAttach(LCD_D2_PIN, LCD_DATA_OUT2_IDX, false, false);
  pinMatrixOutAttach(LCD_D3_PIN, LCD_DATA_OUT3_IDX, false, false);
  pinMatrixOutAttach(LCD_D4_PIN, LCD_DATA_OUT4_IDX, false, false);
  pinMatrixOutAttach(LCD_D5_PIN, LCD_DATA_OUT5_IDX, false, false);
  pinMatrixOutAttach(LCD_D6_PIN, LCD_DATA_OUT6_IDX, false, false);
  pinMatrixOutAttach(LCD_D7_PIN, LCD_DATA_OUT7_IDX, false, false);

  // Control
  pinMatrixOutAttach(LCD_DE_PIN,  LCD_PCLK_IDX, false, false);
  // pinMatrixOutAttach(LCD_DC_PIN, LCD_DC_IDX, false, false);
  // pinMatrixOutAttach(LCD_CS_PIN, LCD_CS_IDX, false, false);
}

// LCD-CAM module
esp_lcd_panel_io_handle_t io_handle = NULL;

volatile bool sendToLCDOK = false;
extern lv_disp_drv_t disp_drv;

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx) {
  lv_disp_flush_ready(&disp_drv);
  LCD_INACTIVE();
  return false;
}

void LCD_init() {
  pinMode(LCD_CS_PIN, OUTPUT);
  LCD_INACTIVE();

  SwitchToGPIOMode();

  pinMode(LCD_DC_PIN, OUTPUT);
  pinMode(LCD_BL_PIN, OUTPUT);

  // Set state of LCD pin
  digitalWrite(LCD_D0_PIN, LOW);
  digitalWrite(LCD_D1_PIN, LOW);
  digitalWrite(LCD_D2_PIN, LOW);
  digitalWrite(LCD_D3_PIN, LOW);
  digitalWrite(LCD_D4_PIN, LOW);
  digitalWrite(LCD_D5_PIN, LOW);
  digitalWrite(LCD_D6_PIN, LOW);
  digitalWrite(LCD_D7_PIN, LOW);
  digitalWrite(LCD_DC_PIN, HIGH); 

  LCD_BACKLIGHT_ON();

  // LCD setup
  LCD_ACTIVE();

  WriteComm(0x2D);
  for (int i = 0; i <= 63; i++) {
    WriteData(i * 8);
  }

  for (int i = 0; i <= 63; i++) {
    WriteData(i * 4);
  }

  for (int i = 0; i <= 63; i++) {
    WriteData(i * 8);
  }

  WriteComm(0xB9); //Set_EXTC
  WriteData(0xFF);
  WriteData(0x83);
  WriteData(0x69);

  WriteComm(0xB1);  //Set Power 
  WriteData(0x85);
	WriteData(0x00);
	WriteData(0x34);
	WriteData(0x0A);
	WriteData(0x00);
	WriteData(0x0F);
	WriteData(0x0F);
	WriteData(0x2A);
	WriteData(0x32);
	WriteData(0x3F);
	WriteData(0x3F);
	WriteData(0x01); //update VBIAS
	WriteData(0x23);
	WriteData(0x01);
	WriteData(0xE6);
	WriteData(0xE6);
	WriteData(0xE6);
	WriteData(0xE6);
	WriteData(0xE6);

  WriteComm(0xB2); // SET Display 480x800
  WriteData(0x00);
  WriteData(0x20);
  WriteData(0x0A);
  WriteData(0x0A);
  WriteData(0x70);
  WriteData(0x00);
  WriteData(0xFF);
  WriteData(0x00);
  WriteData(0x00);
  WriteData(0x00);
  WriteData(0x00);
  WriteData(0x03);
  WriteData(0x03);
  WriteData(0x00);
  WriteData(0x01);

  WriteComm(0xB4); // SET Display 480x800
  WriteData(0x00);
  WriteData(0x18);
  WriteData(0x80);
  WriteData(0x10);
  WriteData(0x01);
  WriteComm(0xB6); // SET VCOM
  WriteData(0x2C);
  WriteData(0x2C);

  WriteComm(0xD5); //SET GIP
  WriteData(0x00);
  WriteData(0x05);
  WriteData(0x03);
  WriteData(0x00);
  WriteData(0x01);
  WriteData(0x09);
  WriteData(0x10);
  WriteData(0x80);
  WriteData(0x37);
  WriteData(0x37);
  WriteData(0x20);
  WriteData(0x31);
  WriteData(0x46);
  WriteData(0x8A);
  WriteData(0x57);
  WriteData(0x9B);
  WriteData(0x20);
  WriteData(0x31);
  WriteData(0x46);
  WriteData(0x8A);
  WriteData(0x57);
  WriteData(0x9B);
  WriteData(0x07);
  WriteData(0x0F);
  WriteData(0x02);
  WriteData(0x00);
  WriteComm(0xE0); //SET GAMMA
  WriteData(0x00);
  WriteData(0x08);
  WriteData(0x0D);
  WriteData(0x2D);
  WriteData(0x34);
  WriteData(0x3F);
  WriteData(0x19);
  WriteData(0x38);
  WriteData(0x09);
  WriteData(0x0E);
  WriteData(0x0E);
  WriteData(0x12);
  WriteData(0x14);
  WriteData(0x12);
  WriteData(0x14);
  WriteData(0x13);
  WriteData(0x19);
  WriteData(0x00);
  WriteData(0x08);

  WriteData(0x0D);
  WriteData(0x2D);
  WriteData(0x34);
  WriteData(0x3F);
  WriteData(0x19);
  WriteData(0x38);
  WriteData(0x09);
  WriteData(0x0E);
  WriteData(0x0E);
  WriteData(0x12);
  WriteData(0x14);
  WriteData(0x12);
  WriteData(0x14);
  WriteData(0x13);
  WriteData(0x19);
  WriteComm(0xC1); //set DGC
  WriteData(0x01); //enable DGC function
  WriteData(0x02); //SET R-GAMMA
  WriteData(0x08);
  WriteData(0x12);
  WriteData(0x1A);
  WriteData(0x22);
  WriteData(0x2A);
  WriteData(0x31);
  WriteData(0x36);
  WriteData(0x3F);
  WriteData(0x48);
  WriteData(0x51);
  WriteData(0x58);
  WriteData(0x60);
  WriteData(0x68);
  WriteData(0x70);
  WriteData(0x78);
  WriteData(0x80);
  WriteData(0x88);
  WriteData(0x90);
  WriteData(0x98);
  WriteData(0xA0);
  WriteData(0xA7);
  WriteData(0xAF);
  WriteData(0xB6);
  WriteData(0xBE);
  WriteData(0xC7);
  WriteData(0xCE);
  WriteData(0xD6);
  WriteData(0xDE);
  WriteData(0xE6);
  WriteData(0xEF);
  WriteData(0xF5);
  WriteData(0xFB);
  WriteData(0xFC);
  WriteData(0xFE);
  WriteData(0x8C);
  WriteData(0xA4);
  WriteData(0x19);
  WriteData(0xEC);
  WriteData(0x1B);
  WriteData(0x4C);

  WriteData(0x40);
  WriteData(0x02); //SET G-Gamma
  WriteData(0x08);
  WriteData(0x12);
  WriteData(0x1A);
  WriteData(0x22);
  WriteData(0x2A);
  WriteData(0x31);
  WriteData(0x36);
  WriteData(0x3F);
  WriteData(0x48);
  WriteData(0x51);
  WriteData(0x58);
  WriteData(0x60);
  WriteData(0x68);
  WriteData(0x70);
  WriteData(0x78);
  WriteData(0x80);
  WriteData(0x88);
  WriteData(0x90);
  WriteData(0x98);
  WriteData(0xA0);
  WriteData(0xA7);
  WriteData(0xAF);
  WriteData(0xB6);
  WriteData(0xBE);
  WriteData(0xC7);
  WriteData(0xCE);
  WriteData(0xD6);
  WriteData(0xDE);
  WriteData(0xE6);
  WriteData(0xEF);
  WriteData(0xF5);
  WriteData(0xFB);
  WriteData(0xFC);
  WriteData(0xFE);
  WriteData(0x8C);
  WriteData(0xA4);
  WriteData(0x19);
  WriteData(0xEC);
  WriteData(0x1B);
  WriteData(0x4C);
  WriteData(0x40);
  WriteData(0x02); //SET B-Gamma
  WriteData(0x08);
  WriteData(0x12);
  WriteData(0x1A);
  WriteData(0x22);
  WriteData(0x2A);
  WriteData(0x31);
  WriteData(0x36);
  WriteData(0x3F);
  WriteData(0x48);
  WriteData(0x51);
  WriteData(0x58);
  WriteData(0x60);
  WriteData(0x68);
  WriteData(0x70);
  WriteData(0x78);

  WriteData(0x80);
  WriteData(0x88);
  WriteData(0x90);
  WriteData(0x98);
  WriteData(0xA0);
  WriteData(0xA7);
  WriteData(0xAF);
  WriteData(0xB6);
  WriteData(0xBE);
  WriteData(0xC7);
  WriteData(0xCE);
  WriteData(0xD6);
  WriteData(0xDE);
  WriteData(0xE6);
  WriteData(0xEF);
  WriteData(0xF5);
  WriteData(0xFB);
  WriteData(0xFC);
  WriteData(0xFE);
  WriteData(0x8C);
  WriteData(0xA4);
  WriteData(0x19);
  WriteData(0xEC);
  WriteData(0x1B);
  WriteData(0x4C);
  WriteData(0x40);
  WriteComm(0x3A); //Set COLMOD
  WriteData(0x55);
  WriteComm(0x11); //Sleep Out
  delay(120);
  WriteComm(0x29); //Display On
		
	delay(10);

  WriteComm(0x3A); //pixel format setting
  WriteData(0x55); 
	WriteComm(0x36); //pixel format setting
  WriteData(0x00); 				
		 	             
  WriteComm(0x11);  
  delay(120);
  WriteComm(0x29);  //Display on 

  WriteComm(0x36); // Rotation
  WriteData(0x60);
  LCD_INACTIVE();

  delay(10);

  Serial.println("Setup with Direct IO OK !");

  ESP_LOGI(TAG, "Initialize Intel 8080 bus");
  esp_lcd_i80_bus_handle_t i80_bus = NULL;
  esp_lcd_i80_bus_config_t bus_config = {
    .dc_gpio_num = 45,
    .wr_gpio_num = LCD_DE_PIN,
    .clk_src = LCD_CLOCK_SRC,
    // .clk_src = LCD_CLK_SRC_XTAL,
    // .clk_src = LCD_CLK_SRC_PLL160M,
    .data_gpio_nums = {
      LCD_D0_PIN,
      LCD_D1_PIN,
      LCD_D2_PIN,
      LCD_D3_PIN,
      LCD_D4_PIN,
      LCD_D5_PIN,
      LCD_D6_PIN,
      LCD_D7_PIN,
    },
    .bus_width = 8,
    .max_transfer_bytes = LCD_WIDTH * 40 * sizeof(uint16_t),
  };
  ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));
  esp_lcd_panel_io_i80_config_t io_config = {
    .cs_gpio_num = -1,
    .pclk_hz = LCD_PIXEL_CLOCK_HZ,
    .trans_queue_depth = 10,
    .on_color_trans_done = notify_lvgl_flush_ready,
    .user_ctx = NULL,
    .lcd_cmd_bits = 8,
    .lcd_param_bits = 8,
    .dc_levels = {
      .dc_idle_level = 0,
      .dc_cmd_level = 0,
      .dc_dummy_level = 0,
      .dc_data_level = 1,
    },
    .flags = {
      .swap_color_bytes = 1,
      .pclk_active_neg = 1,
      .pclk_idle_low = 1,
    }
  };
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));
  Serial.println("LCD-CAM setup OK !");
}

void LCD_drawBitmap(int x_start, int y_start, int x_end, int y_end, const void* color_data) {
  static bool lock = false;

  SwitchToGPIOMode();

  LCD_ACTIVE();
  WriteComm(0x2A);
  WriteData((x_start >> 8) & 0xFF);
  WriteData(x_start & 0xFF);
  WriteData((x_end >> 8) & 0xFF);
  WriteData(x_end & 0xFF); 		
  WriteComm(0x2B);
  WriteData((y_start >> 8) & 0xFF);
  WriteData(y_start & 0xFF);
  WriteData((y_end >> 8) & 0xFF);
  WriteData(y_end & 0xFF); 		
  
  WriteComm(0x2C);
  digitalWrite(LCD_DC_PIN, HIGH);

  SwitchToLCDCAMMode();
  
  // transfer frame buffer
  size_t len = ((x_end - x_start + 1) * (y_end - y_start + 1) * 2) - 1;
  esp_lcd_panel_io_tx_color(io_handle, *(uint8_t*)(color_data), (const void*)(color_data + 1), len);
}
