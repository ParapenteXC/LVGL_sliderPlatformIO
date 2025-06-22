// brightness slider with 20 sec screen cutoff 2 min deep sleep

#include <Arduino.h>
#include <lvgl.h>
#include <demos/lv_demos.h>
#include <Arduino_GFX_Library.h>
#include "bsp_cst816.h"

#define EXAMPLE_PIN_NUM_LCD_SCLK  39
#define EXAMPLE_PIN_NUM_LCD_MOSI  38
#define EXAMPLE_PIN_NUM_LCD_MISO  40
#define EXAMPLE_PIN_NUM_LCD_DC    42
#define EXAMPLE_PIN_NUM_LCD_RST  -1
#define EXAMPLE_PIN_NUM_LCD_CS    45
#define EXAMPLE_PIN_NUM_LCD_BL    1
#define EXAMPLE_PIN_NUM_TP_SDA    48
#define EXAMPLE_PIN_NUM_TP_SCL    47
#define BOOT_BUTTON_PIN           0
#define BUZZER_PIN                2     // 
#define LONG_PRESS_DURATION       2000  // in milliseconds

unsigned long bootPressStart  =   0;    //
bool bootHeld                 =   false;//
bool screenOn                 =   true; //

bool backlightOn              =   true;
int  lastButtonState          =   1;
int  gbl_valu                 =   80;
unsigned long lastTouchTime   =   0;
bool isDimmed                 =   false;

#define LEDC_FREQ                 5000
#define LEDC_TIMER_10_BIT         10

#define EXAMPLE_LCD_ROTATION      1
#define EXAMPLE_LCD_H_RES         240
#define EXAMPLE_LCD_V_RES         320

/* More data bus class: https://github.com/moononournation/Arduino_GFX/wiki/Data-Bus-Class */
Arduino_DataBus *bus = new Arduino_ESP32SPI(
  EXAMPLE_PIN_NUM_LCD_DC /* DC */, EXAMPLE_PIN_NUM_LCD_CS /* CS */,
  EXAMPLE_PIN_NUM_LCD_SCLK /* SCK */, EXAMPLE_PIN_NUM_LCD_MOSI /* MOSI */, EXAMPLE_PIN_NUM_LCD_MISO /* MISO */);

/* More display class: https://github.com/moononournation/Arduino_GFX/wiki/Display-Class */
Arduino_GFX *gfx = new Arduino_ST7789(
  bus, EXAMPLE_PIN_NUM_LCD_RST /* RST */, EXAMPLE_LCD_ROTATION /* rotation */, true /* IPS */,
  EXAMPLE_LCD_H_RES /* width */, EXAMPLE_LCD_V_RES /* height */);


uint32_t screenWidth;
uint32_t screenHeight;
uint32_t bufSize;
lv_disp_draw_buf_t draw_buf;
lv_color_t *disp_draw_buf;
lv_disp_drv_t disp_drv;

lv_obj_t *label_brightness;

lv_timer_t *brightness_timer = NULL;

void slider_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_VALUE_CHANGED)
    {
        // 
        lv_obj_t *slider = lv_event_get_target(e);
        int value = lv_slider_get_value(slider);
        // printf("Slider value: %d\n", value);

        lv_label_set_text_fmt(label_brightness, "%d %%", value);
        // bsp_lcd_brightness_set(value);
        ledcWrite(EXAMPLE_PIN_NUM_LCD_BL , (1 << LEDC_TIMER_10_BIT) / 100 * value);
        // 
        lv_event_stop_bubbling(e);
        gbl_valu = value;
      screenOn = true;
    }
}

void check_boot_button() {
  int btnState = digitalRead(BOOT_BUTTON_PIN);   //read pin and load into "btnState"

  if (btnState == LOW) {          // if pressed  pull down buttn = pressed
    if (!bootHeld) {              // and not already in long press condition
      bootPressStart = millis();  // restart button press count
      bootHeld = true;
    } else if (millis() - bootPressStart > LONG_PRESS_DURATION) {
      Serial.println("Long press detected. Going to deep sleep...");
      // Prepare for deep sleep
      esp_deep_sleep_start();
    }

  } else {
    //                              ###screen on/off toggle
    if (bootHeld && millis() - bootPressStart < LONG_PRESS_DURATION) { 
      // Short press → toggle screen
      screenOn = !screenOn;
      if (screenOn) {
        ledcAttach(EXAMPLE_PIN_NUM_LCD_BL , LEDC_FREQ, LEDC_TIMER_10_BIT);  // return to slider brightness
        ledcWrite(EXAMPLE_PIN_NUM_LCD_BL , (1 << LEDC_TIMER_10_BIT) / 100 * gbl_valu);
      } else {
        ledcWrite(EXAMPLE_PIN_NUM_LCD_BL , 0);                              // Turn off LDC Backlight
      }
    }
    bootHeld = false;
  }
}

void lvgl_brightness_ui_init(lv_obj_t *parent)
{
    lv_obj_t *obj = lv_obj_create(parent);
    lv_obj_set_size(obj, lv_pct(90), lv_pct(50));
    lv_obj_align(obj, LV_ALIGN_CENTER, 0, 0); 
    // 
    lv_obj_t *slider = lv_slider_create(obj);

    // 
    lv_slider_set_range(slider, 1, 100);          
    lv_slider_set_value(slider, 80, LV_ANIM_OFF); 

    // 
    lv_obj_set_size(slider, lv_pct(90), 20);    
    lv_obj_align(slider, LV_ALIGN_CENTER, 0, 0); 

    lv_obj_set_style_pad_top(obj, 20, 0);
    lv_obj_set_style_pad_bottom(obj, 20, 0);
    // lv_obj_set_style_pad_left(parent, 50, 0);
    // lv_obj_set_style_pad_right(parent, 50, 0);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_GESTURE_BUBBLE);
    // 
    lv_obj_add_event_cb(slider, slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    label_brightness = lv_label_create(obj);
    lv_label_set_text(label_brightness, "80%");
    lv_obj_align(label_brightness, LV_ALIGN_TOP_MID, 0, 0);

}


#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char *buf) {
  Serial.printf(buf);
  Serial.flush();
}
#endif

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
  lv_disp_flush_ready(disp_drv);
}

/*Read the touchpad*/
void my_touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {
  uint16_t touchpad_x;
  uint16_t touchpad_y;
  bsp_touch_read();
  if (bsp_touch_get_coordinates(&touchpad_x, &touchpad_y)) {
    data->point.x = touchpad_x;
    data->point.y = touchpad_y;
    data->state = LV_INDEV_STATE_PRESSED;
    lastTouchTime = millis(); // 🕒 Reset the timer

    if (!screenOn || isDimmed) {
      screenOn = true;
      isDimmed = false;
      ledcWrite(EXAMPLE_PIN_NUM_LCD_BL , (1 << LEDC_TIMER_10_BIT) / 100 * gbl_valu);
    }
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  // Serial.setDebugOutput(true);
  // while(!Serial);
  Serial.println("Arduino_GFX LVGL_Arduino_v7 example ");
  String LVGL_Arduino = String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();
  Serial.println(LVGL_Arduino);

#ifdef GFX_EXTRA_PRE_INIT
  GFX_EXTRA_PRE_INIT();
#endif

  // Init Display
  if (!gfx->begin()) {
    Serial.println("gfx->begin() failed!");
  }
  gfx->fillScreen(BLACK);

//get the button state check if its different
#ifdef EXAMPLE_PIN_NUM_LCD_BL
  // pinMode(EXAMPLE_PIN_NUM_LCD_BL, OUTPUT);
  ledcAttach(EXAMPLE_PIN_NUM_LCD_BL , LEDC_FREQ, LEDC_TIMER_10_BIT);
  ledcWrite(EXAMPLE_PIN_NUM_LCD_BL , (1 << LEDC_TIMER_10_BIT) / 100 * 80);
  screenOn = HIGH;
#endif

   //Init touch device
   //touch_init(gfx->width(), gfx->height(), gfx->getRotation());
  Wire.begin(EXAMPLE_PIN_NUM_TP_SDA, EXAMPLE_PIN_NUM_TP_SCL);
  bsp_touch_init(&Wire, gfx->getRotation(), gfx->width(), gfx->height());
  lv_init();

#if LV_USE_LOG != 0
  lv_log_register_print_cb(my_print); /* register print function for debugging */
#endif

  screenWidth = gfx->width();
  screenHeight = gfx->height();

  bufSize = screenWidth * screenHeight;


  disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!disp_draw_buf) {
    // remove MALLOC_CAP_INTERNAL flag try again
    disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_8BIT);
  }

  if (!disp_draw_buf) {
    Serial.println("LVGL disp_draw_buf allocate failed!");
  } else {
    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, bufSize);

    /* Initialize the display */
    lv_disp_drv_init(&disp_drv);
    /* Change the following line to your display resolution */
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.direct_mode = true;

    lv_disp_drv_register(&disp_drv);

    /* Initialize the (dummy) input device driver */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    /* Option 3: Or try out a demo. Don't forget to enable the demos in lv_conf.h. E.g. LV_USE_DEMOS_WIDGETS*/
    // lv_demo_widgets();
    lvgl_brightness_ui_init(lv_scr_act());
    // lv_demo_benchmark();
    // lv_demo_keypad_encoder();
    // lv_demo_music();
    // lv_demo_stress();
  }

  Serial.println("Setup done");
}

void loop() {
  lv_timer_handler(); /* let the GUI do its work */
  //handleBootButton(); // 👈 Check for toggle
  check_boot_button();  // call it every loop

#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(0, 0, (uint16_t *)disp_draw_buf, screenWidth, screenHeight);
#else
  gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)disp_draw_buf, screenWidth, screenHeight);
#endif

  // 💤 Auto-dim after 20 seconds of no touch
  if (screenOn && !isDimmed && millis() - lastTouchTime > 20000) {
    ledcWrite(EXAMPLE_PIN_NUM_LCD_BL , 0); // turn off backlight
    isDimmed = true;
  }

  // 💤 Auto-deep sleep after 2 min of inactivity
 if (isDimmed && millis() - lastTouchTime > 120000) {
    esp_deep_sleep_start();
  }
delay(isDimmed ? 100 : 5);
}
