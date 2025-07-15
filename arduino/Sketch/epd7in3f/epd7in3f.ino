#include "DEV_Config.h"
#include "EPD_7in3f.h"
#include "GUI_Paint.h"
#include "ImageData.h"
#include <stdlib.h>

void setup()
{
    Serial.begin(115200);
    Serial.println("EPD_7IN3F_test Demo");

    // Hardware init
    if (DEV_Module_Init() != 0) {
        Serial.println("Hardware init failed");
        return;
    }

    Serial.println("e-Paper Init and Clear...");
    EPD_7IN3F_Init();
    EPD_7IN3F_Clear();

    // Create a new image cache
    UBYTE *BlackImage;
    UWORD Imagesize = ((EPD_7IN3F_WIDTH % 8 == 0) ? (EPD_7IN3F_WIDTH / 8) : (EPD_7IN3F_WIDTH / 8 + 1)) * EPD_7IN3F_HEIGHT;
    if ((BlackImage = (UBYTE *)malloc(Imagesize)) == NULL) {
        Serial.println("Failed to allocate memory...");
        return;
    }
    Paint_NewImage(BlackImage, EPD_7IN3F_WIDTH, EPD_7IN3F_HEIGHT, 0, WHITE);
    Paint_SetScale(7);
    Paint_Clear(WHITE);  // Set background to white

    // Draw black text on white background
    Paint_DrawString_EN(20, 20, "Fast Refresh", &Font24, WHITE, BLACK);
    Paint_DrawString_EN(20, 60, "Black on White", &Font24, WHITE, BLACK);

    // Push image to display
    EPD_7IN3F_Display(BlackImage);
    delay(3000);  // Reduced delay for faster testing

    // Optional: Sleep to preserve screen and save power
    EPD_7IN3F_Sleep();
    free(BlackImage);
    BlackImage = NULL;
}

void loop()
{
    // Nothing here
}
