#include <Arduino.h>
#include <lvgl.h>
#include <Arduino_GFX_Library.h>
#include <CST816S_Touch.h>
#include <Wire.h>

//=== Pin definitions ===
#define LCD_SCLK  39
#define LCD_MOSI  38
#define LCD_MISO  40
#define LCD_DC    42
#define LCD_CS    45
#define LCD_RST   -1
#define LCD_BL    1
#define TP_SDA    48
#define TP_SCL    47
#define TP_INT    -1
#define TP_RST    -1

// Globals
Arduino_DataBus* bus;
Arduino_GFX* gfx;
CST816S_Touch touch(TP_SDA, TP_SCL, TP_INT, TP_RST);
lv_disp_draw_buf_t draw_buf;
lv_color_t* buf;
lv_disp_drv_t disp_drv;
lv_indev_drv_t indev_drv;
uint32_t lastTouch = 0;

// GUI elements
lv_obj_t *slider, *label;

void my_disp_flush(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* color_p) {
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t*)color_p,
                          area->x2 - area->x1 + 1, area->y2 - area->y1 + 1);
  lv_disp_flush_ready(disp);
}

void my_touch_read(lv_indev_drv_t* drv, lv_indev_data_t* data) {
  uint16_t x, y;
  touch.read();
 if (touch.available()) {
  uint16_t x = touch.getX();
  uint16_t y = touch.getY();
  data->point.x = x;
  data->point.y = y;
  data->state = LV_INDEV_STATE_PRESSED;
  lastTouch = millis();
} else {
  data->state = LV_INDEV_STATE_RELEASED;
}
  data->continue_reading = false; // No need to read again immediately
  if (x > gfx->width() || y > gfx->height()) {
    data->point.x = 0;
    data->point.y = 0;
  }
  Serial.printf("Touch: %d, %d\n", x, y);
}

void setup() {
  Serial.begin(115200);
  Serial.println("LVGL Touch UI Starting");

  bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCLK, LCD_MOSI, LCD_MISO);
  gfx = new Arduino_ST7789(bus, LCD_RST, 1, true, 240, 320);

  if (!gfx->begin()) Serial.println("Display init failed");
  gfx->fillScreen(BLACK);

  lv_init();
  uint32_t screenW = gfx->width(), screenH = gfx->height();
  buf = (lv_color_t*)malloc(screenW * screenH * sizeof(lv_color_t));
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenW * screenH);

  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = screenW;
  disp_drv.ver_res = screenH;
  disp_drv.draw_buf = &draw_buf;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.direct_mode = true;
  lv_disp_drv_register(&disp_drv);

  // Touch setup
  Wire.begin(TP_SDA, TP_SCL);  // start I2C
  touch.begin();
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touch_read;
  lv_indev_drv_register(&indev_drv);

  // UI
  lv_obj_t* scr = lv_scr_act();
  slider = lv_slider_create(scr);
  label = lv_label_create(scr);
  lv_slider_set_range(slider, 0, 100);
  lv_obj_set_size(slider, 200, 20);
  lv_obj_set_pos(slider, 20, 150);
  lv_obj_set_pos(label, 20, 120);
  lv_obj_add_event_cb(slider, [](lv_event_t* e) {
    int v = lv_slider_get_value(slider);
    lv_label_set_text_fmt(label, "BRI: %d%%", v);
  }, LV_EVENT_VALUE_CHANGED, nullptr);
  lv_slider_set_value(slider, 50, LV_ANIM_OFF);
  lv_label_set_text(label, "BRI: 50%");
}

void loop() {
  lv_timer_handler();
  if (millis() - lastTouch > 20000) {
    gfx->fillScreen(BLACK);
    lastTouch = millis();
  }
  delay(5);
}
