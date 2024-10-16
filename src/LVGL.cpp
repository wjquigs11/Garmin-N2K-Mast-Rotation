#ifdef CYD
#include <lvgl.h>
#include <TFT_eSPI.h>
#include "windparse.h"
#include "wind-bus.h"
#include "BoatData.h"
#include "compass.h"

// object-oriented classes
#include "logto.h"
#ifdef N2K
#include "n2k.h"
#endif

#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320

#define DRAW_BUF_SIZE (SCREEN_WIDTH * SCREEN_HEIGHT / 10 * (LV_COLOR_DEPTH / 8))
uint32_t draw_buf[DRAW_BUF_SIZE / 4];

extern tBoatData *pBD;

// If logging is enabled, it will inform the user about what is happening in the library
void log_print(lv_log_level_t level, const char * buf) {
  LV_UNUSED(level);
  Serial.println(buf);
  Serial.flush();
}

// placeholder for adding GPIO button controls later
static uint32_t last_key = 0;
static bool key_pressed = false;
#define BUTTON_PIN 22

static void button_read(lv_indev_t * drv, lv_indev_data_t *data) {
    uint32_t act_time = millis();
    if(act_time - last_key > 200) {
        last_key = act_time;
        key_pressed = digitalRead(BUTTON_PIN) == LOW;
    }
    data->state = key_pressed ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
    data->key = LV_KEY_ENTER; // Or any other key code you want
}

lv_obj_t *data_label, *rotation_label;
lv_obj_t *screen;
lv_style_t style;
lv_style_t style_large_font;
lv_obj_t *needle_line;

static void set_needle_line_value(void * obj, int32_t v) {
    lv_scale_set_line_needle_value((lv_obj_t *)obj, needle_line, 60, v);
}

int32_t Needle = 0;

lv_coord_t center_x = SCREEN_WIDTH * 3 / 10;  // Half of the scale width
lv_coord_t center_y = SCREEN_WIDTH * 3 / 10;  // Half of the scale height
lv_coord_t radius = SCREEN_WIDTH * 3 / 10;  // Same as half of the scale width/height
#define SCALE_BOT_OFFSET 30
#define yellow 0xFFFF00

void update_needle_position(int16_t value) {
    // Calculate the angle in radians
    float angle_rad = (value + 180) * (M_PI / 180);  // Convert to radians
    
    lv_coord_t end_x = center_x - (lv_coord_t)(sin(angle_rad) * radius);
    lv_coord_t end_y = center_y - (lv_coord_t)(cos(angle_rad) * radius);
        static lv_point_precise_t line_points[] = {{0, 0}, {0, 0}};
    line_points[0].x = center_x;
    line_points[0].y = center_y;
    line_points[1].x = end_x;
    line_points[1].y = end_y;
        
    //Serial.printf("value %d end x, y = %ld, %ld\n", value, end_x, end_y);
    lv_line_set_points(needle_line, line_points, 2);
}

void lv_create_main_gui(void) {
  screen = lv_screen_active();
  Serial.printf("create main gui 0x%x\n", screen);
  lv_obj_set_style_bg_color(screen, lv_color_black(), 0);
  lv_style_init(&style);
  lv_style_set_text_color(&style, lv_color_hex(yellow)); // Yellow
  lv_style_init(&style_large_font);
  lv_style_set_text_font(&style_large_font, &lv_font_montserrat_16);  
  //lv_style_set_text_font(&style_large_font, &lv_font_unscii_16);  
  data_label = lv_label_create(screen);
  lv_obj_add_style(data_label, &style, 0);
  lv_obj_add_style(data_label, &style_large_font, 0);
  lv_label_set_long_mode(data_label, LV_LABEL_LONG_WRAP);    // Breaks the long lines
  lv_obj_set_width(data_label, SCREEN_WIDTH);    // Set smaller width to make the lines wrap

  // create a "scale" which is library speak for gauge
  lv_obj_t *scale_line = lv_scale_create(lv_screen_active());
  lv_obj_set_size(scale_line, SCREEN_WIDTH*6/10, SCREEN_WIDTH*6/10);
  lv_scale_set_mode(scale_line, LV_SCALE_MODE_ROUND_OUTER);
  lv_obj_set_style_bg_opa(scale_line, LV_OPA_TRANSP, 0);
  //lv_obj_set_style_bg_opa(scale_line, LV_OPA_50, 0); // uncomment to see whole circle

  lv_obj_set_style_arc_color(scale_line, lv_color_hex(yellow), LV_PART_MAIN);
  lv_obj_set_style_arc_width(scale_line, 4, LV_PART_MAIN);
  // Set the tick color to yellow
  lv_obj_set_style_line_color(scale_line, lv_color_hex(yellow), LV_PART_MAIN);
  lv_obj_set_style_line_color(scale_line, lv_color_hex(yellow), LV_PART_ITEMS);
  lv_obj_set_style_text_color(scale_line, lv_color_hex(yellow), 0);

  lv_obj_set_style_radius(scale_line, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_clip_corner(scale_line, true, 0);
  //lv_obj_align(scale_line, LV_ALIGN_BOTTOM_MID, LV_PCT(2), 0);
  lv_obj_align(scale_line, LV_ALIGN_BOTTOM_MID, 0, -SCALE_BOT_OFFSET);
  lv_scale_set_label_show(scale_line, false);
  lv_obj_set_style_text_color(scale_line, lv_color_hex(yellow), 0);
  lv_obj_set_style_line_color(scale_line, lv_color_hex(yellow), LV_PART_MAIN);
  lv_obj_set_style_line_color(scale_line, lv_color_hex(yellow), LV_PART_ITEMS);
  lv_obj_set_style_line_color(scale_line, lv_color_hex(yellow), LV_PART_INDICATOR);
  lv_obj_set_style_length(scale_line, 5, LV_PART_ITEMS);
  lv_obj_set_style_length(scale_line, 10, LV_PART_INDICATOR);
  lv_scale_set_range(scale_line, 90, -90);

  lv_scale_set_angle_range(scale_line, 180);
  lv_scale_set_rotation(scale_line, 0);

  // create label for scale (rotation)
  rotation_label = lv_label_create(lv_screen_active());
  lv_label_set_text(rotation_label, "0");  // Initial text
  lv_obj_set_style_text_color(rotation_label, lv_color_hex(yellow), 0);
  lv_obj_align_to(rotation_label, scale_line, LV_ALIGN_BOTTOM_MID, -10, SCALE_BOT_OFFSET);  

  // Create the needle line
  needle_line = lv_line_create(scale_line);
  lv_obj_set_style_line_color(needle_line, lv_color_hex(yellow), LV_PART_MAIN);
  lv_obj_set_style_line_width(needle_line, 3, LV_PART_MAIN);  // Thinner line
  lv_obj_set_style_line_rounded(needle_line, true, LV_PART_MAIN);

  // Position the needle at 0
  update_needle_position(0);
}

char bufB[512], bufS[128]; // Buffer for formatting text

void LVGLdataWindDebug() {
    unsigned long uptime = millis() / 1000;
    //Serial.printf("LVGLdataWindDebug %lu\n", uptime);
    //int rotateout = map((Needle++ % 180),0,180,-90,90);
    sprintf(bufB, "Uptime: %lu\n", uptime % 10000);
    sprintf(bufS, "Wifi: %s\n", (WiFi.status() == WL_CONNECTED) ? WiFi.SSID().substring(0,7).c_str() : "----");
    strcat(bufB, bufS);
#ifdef N2K
    sprintf(bufS, "N2K: %d:%d\n", n2k::num_n2k_messages % 10000, 0);
    strcat(bufB, bufS);
    sprintf(bufS, "Wind: %d:%d\n", n2k::num_wind_messages % 10000, 0);
    strcat(bufB, bufS);
    sprintf(bufS, "AWS:%2.1f\n", n2k::windSpeedKnots);
    strcat(bufB, bufS);
    sprintf(bufS, "AWA:%2.0f\n", n2k::windAngleDegrees);
    strcat(bufB, bufS);
#endif
    lv_label_set_text(data_label, bufB);

    // update label at bottom of gauge
#ifdef N2K
    snprintf(bufB, sizeof(bufB), "%0.2f", n2k::rotateout);
    lv_label_set_text(rotation_label, bufB);
    update_needle_position(n2k::rotateout);
#endif
    //if (uptime % 60 == 0) {
    //    logTo::logToAll("uptime: " + String(uptime) + " heap: " + String(ESP.getFreeHeap()));
    //}

    // Trigger LVGL to update the display
    lv_task_handler();
}

void setupLVGL() {
  String LVGL_Arduino = String("LVGL Library Version: ") + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();
  //Serial.begin(115200); delay(500);
  //Serial.println("Starting LVGL");
  Serial.println(LVGL_Arduino);
  
  // Start LVGL
  lv_init();
  // Register print function for debugging
  lv_log_register_print_cb(log_print);

  // Create a display object
  lv_display_t *disp;
  // Initialize the TFT display using the TFT_eSPI library
  disp = lv_tft_espi_create(SCREEN_WIDTH, SCREEN_HEIGHT, draw_buf, sizeof(draw_buf));
  lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_180);

  // Initialize an LVGL input device object (Touchscreen)
#if 0
  lv_indev_t * indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  // Set the callback function to read Touchscreen input
  lv_indev_set_read_cb(indev, button_read);
#endif
  // Function to draw the GUI (text, buttons and sliders)
  lv_create_main_gui();
  Serial.println("main gui online");
  //LVGLdataWindDebug();
}

void loopLVGL() {
  lv_task_handler();  // let the GUI do its work
  lv_tick_inc(100);     // tell LVGL how much time has passed
  delay(100);           // let this time pass
  //LVGLdataWindDebug();
}
#endif
