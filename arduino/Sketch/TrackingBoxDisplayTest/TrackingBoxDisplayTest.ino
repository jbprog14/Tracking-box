/*
 * TrackingBoxMain2.ino – minimal standalone sketch
 * that proves the Waveshare 3.7" (ED037TC1) e-ink works.
 *
 * Wiring is fixed in DEV_Config.h (SPI on 13/14/15 + 25/26/27).
 */

#include "DEV_Config.h"
#include "EPD.h"
#include "GUI_Paint.h"
#include "fonts.h"

static UBYTE *frame = nullptr;

void setup() {
  Serial.begin(115200);
  Serial.println("\n[TrackingBoxMain2] booting…");

  // Low-level init
  DEV_Module_Init();
  EPD_3IN7_4Gray_Init();

  // Allocate full buffer (same maths as Waveshare demo)
  UWORD size = ((EPD_3IN7_WIDTH % 4 == 0) ? (EPD_3IN7_WIDTH / 4)
                                           : (EPD_3IN7_WIDTH / 4 + 1))
               * EPD_3IN7_HEIGHT;
  frame = (UBYTE *)malloc(size);
  if (!frame) {
    Serial.println("Framebuffer malloc failed!");
    while (1);
  }
  Paint_NewImage(frame, EPD_3IN7_WIDTH, EPD_3IN7_HEIGHT, 270, WHITE);
  Paint_SetScale(4);  // 4-gray mode

  // Draw demo screen
  Paint_Clear(WHITE);
  Paint_DrawString_EN(10, 10, (char *)"TRACKING BOX", &Font20, WHITE, BLACK);
  Paint_DrawString_EN(10, 40, (char *)"Display OK", &Font16, WHITE, BLACK);
  Paint_DrawLine(10, 60, 450, 60, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);  // separator
  Paint_DrawString_EN(10, 80, (char *)"Sample content", &Font12, WHITE, BLACK);

  // Push to panel & sleep
  EPD_3IN7_4Gray_Display(frame);
  Serial.println("Screen drawn – going to deep-sleep.");
  EPD_3IN7_Sleep();

  esp_deep_sleep_start();
}

void loop() {
  // never reached
} 