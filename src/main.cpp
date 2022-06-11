#include "Arduino.h"
#include "Wire.h"
#include "PinCofigs.h"
#include "LCD.h"
#include <lvgl.h>
#include "lv_demos.h"
#include "FT5216.h"

/*Change to your screen resolution*/
static const uint16_t screenWidth  = 800;
static const uint16_t screenHeight = 480;

static const uint32_t buffer_size = screenWidth * 15 * sizeof(lv_color_t);
static lv_disp_draw_buf_t draw_buf;
lv_color_t *buf;

/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )  {
    LCD_drawBitmap(area->x1, area->y1, area->x2, area->y2, &color_p->full);

    // lv_disp_flush_ready(disp);
}

/*Read the touchpad*/
void my_touchpad_read( lv_indev_drv_t *indev_driver, lv_indev_data_t * data) {
  uint8_t touchPoint = touch.read((uint16_t*)(&data->point.x), (uint16_t*)(&data->point.y));
  if (touchPoint > 0) {
    Serial.printf("X: %d\tY: %d\n", data->point.x, data->point.y);
  }
  data->state = touchPoint > 0 ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
}

lv_disp_drv_t disp_drv;

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400E3);

  LCD_init();
  touch.init();

  lv_init();
  buf = (lv_color_t*) heap_caps_malloc(buffer_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  if (!buf) {
    Serial.println("malloc fail!");
    while(1) delay(10);
  }
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, buffer_size);

  /*Initialize the display*/
  lv_disp_drv_init(&disp_drv);

  /*Change the following line to your display resolution*/
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  /*Initialize the (dummy) input device driver*/
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);
/*
  lv_obj_t *label = lv_label_create(lv_scr_act());
  lv_label_set_text(label, "Hello !, LVGL");
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
*/
  Serial.println("Setup done");

  lv_demo_widgets();
}

void loop() {
  lv_timer_handler(); /* let the GUI do its work */
  delay(100);
}
