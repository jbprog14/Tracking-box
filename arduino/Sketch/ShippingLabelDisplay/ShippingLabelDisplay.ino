#include "DEV_Config.h"
#include "EPD.h"
#include "GUI_Paint.h"
#include "qrcode.h"
#include <stdlib.h>

// Device configuration
const char* DEVICE_ID = "box_001";  // Change this for each device
const char* BASE_URL = "https://tracking-box.pages.dev/qr/";

// Shipping label data
struct ShippingLabel {
    const char* senderName = "Thanhkh Technologies Ltd";
    const char* senderAddress = "11 No.12 Dong Rd, Tsuen Wan, Hong Kong";
    const char* shipToLabel = "Ship To:";
    const char* recipientName = "Jane Doe";
    const char* recipientAddress = "Manila, Philippines";
    const char* weight = "10.00KG";
    const char* routingCode = "0522";
    const char* postalCode = "8992 0843 67";
    const char* trackingNumber = "ZH-0522";
    const char* barcodeNumber = "052289920843672077";
    const char* serviceType = "D-B2C";
    const char* service = "SERVICE";
    const char* leftBarcode = "0522";
    const char* rightBarcode = "2077";
} label;

// Function to generate QR code URL
void generateQRUrl(char* buffer, const char* deviceId) {
    sprintf(buffer, "%s%s", BASE_URL, deviceId);
}

// Function to draw QR code at specified position
void drawQRCode(int xPos, int yPos, const char* text, int scale) {
    QRCode qrcode;
    uint8_t qrcodeData[qrcode_getBufferSize(3)];
    qrcode_initText(&qrcode, qrcodeData, 3, ECC_LOW, text);
    
    // Draw white background for QR code
    int qrSize = qrcode.size * scale;
    Paint_DrawRectangle(xPos - 5, yPos - 5, xPos + qrSize + 5, yPos + qrSize + 5, 
                       EPD_7IN3F_WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    
    // Draw QR code modules
    for (uint8_t y = 0; y < qrcode.size; y++) {
        for (uint8_t x = 0; x < qrcode.size; x++) {
            if (qrcode_getModule(&qrcode, x, y)) {
                Paint_DrawRectangle(xPos + (x * scale), yPos + (y * scale),
                                  xPos + ((x + 1) * scale), yPos + ((y + 1) * scale),
                                  EPD_7IN3F_BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
            }
        }
    }
}

// Function to draw MAXICODE placeholder
void drawMaxicode(int xPos, int yPos, int size) {
    // Draw white background
    Paint_DrawRectangle(xPos, yPos, xPos + size, yPos + size, 
                       EPD_7IN3F_WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    
    // Draw black border
    Paint_DrawRectangle(xPos, yPos, xPos + size, yPos + size, 
                       EPD_7IN3F_BLACK, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);
    
    // Draw hexagonal pattern placeholder
    int centerX = xPos + size/2;
    int centerY = yPos + size/2;
    int radius = size/3;
    
    // Draw concentric circles as placeholder for MAXICODE
    Paint_DrawCircle(centerX, centerY, radius, EPD_7IN3F_BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    Paint_DrawCircle(centerX, centerY, radius/2, EPD_7IN3F_BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    Paint_DrawCircle(centerX, centerY, radius/4, EPD_7IN3F_WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
}

// Function to draw barcode - standard width with proper bars
void drawBarcode(int xPos, int yPos, int width, int height, const char* data) {
    // Draw barcode background
    Paint_DrawRectangle(xPos, yPos, xPos + width, yPos + height, 
                       EPD_7IN3F_WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    
    // Calculate bar width
    int dataLen = strlen(data);
    int totalBars = (dataLen * 7) + 6;
    int barWidth = width / totalBars;
    if (barWidth < 2) barWidth = 2;
    if (barWidth > 4) barWidth = 4;
    
    int currentX = xPos + 10;
    
    // Start guard pattern - thick bars
    Paint_DrawRectangle(currentX, yPos, currentX + barWidth * 2, yPos + height, 
                       EPD_7IN3F_BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    currentX += barWidth * 3;
    Paint_DrawRectangle(currentX, yPos, currentX + barWidth * 2, yPos + height, 
                       EPD_7IN3F_BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    currentX += barWidth * 3;
    
    // Draw data bars - spread across full width
    for(int i = 0; data[i] != '\0' && currentX < (xPos + width - 30); i++) {
        int digit = data[i] - '0';
        if(digit >= 0 && digit <= 9) {
            // Create unique pattern for each digit with thicker bars
            if(digit & 1) {
                // Thick black bar
                Paint_DrawRectangle(currentX, yPos, currentX + barWidth * 2, yPos + height, 
                                   EPD_7IN3F_BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
                currentX += barWidth * 2;
            } else {
                // Thin black bar
                Paint_DrawRectangle(currentX, yPos, currentX + barWidth, yPos + height, 
                                   EPD_7IN3F_BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
                currentX += barWidth;
            }
            
            // Space
            currentX += barWidth;
            
            if(digit & 2) {
                // Extra thick bar for certain digits
                Paint_DrawRectangle(currentX, yPos, currentX + barWidth * 3, yPos + height, 
                                   EPD_7IN3F_BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
                currentX += barWidth * 3;
            } else {
                // Medium bar
                Paint_DrawRectangle(currentX, yPos, currentX + barWidth * 2, yPos + height, 
                                   EPD_7IN3F_BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
                currentX += barWidth * 2;
            }
            
            // Space between digits
            currentX += barWidth;
            
            if(digit > 5) {
                // Additional bar for higher digits
                Paint_DrawRectangle(currentX, yPos, currentX + barWidth * 2, yPos + height, 
                                   EPD_7IN3F_BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
                currentX += barWidth * 2;
            }
            
            // Inter-character gap
            currentX += barWidth * 2;
        }
    }
    
    // Fill remaining space with pattern if needed
    while(currentX < (xPos + width - 30)) {
        Paint_DrawRectangle(currentX, yPos, currentX + barWidth, yPos + height, 
                           EPD_7IN3F_BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        currentX += barWidth * 2;
    }
    
    // End guard pattern - thick bars
    currentX = xPos + width - 25;
    Paint_DrawRectangle(currentX, yPos, currentX + barWidth * 2, yPos + height, 
                       EPD_7IN3F_BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    currentX += barWidth * 3;
    Paint_DrawRectangle(currentX, yPos, currentX + barWidth * 2, yPos + height, 
                       EPD_7IN3F_BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
}

void setup() {
    printf("Shipping Label Display - Full Layout\r\n");
    printf("Device ID: %s\r\n", DEVICE_ID);
    printf("Display: Waveshare 7.3inch E-ink (800x480)\r\n");
    
    DEV_Module_Init();
    
    printf("Initializing E-ink display...\r\n");
    EPD_7IN3F_Init();
    
    EPD_7IN3F_Clear(EPD_7IN3F_WHITE);
    DEV_Delay_ms(1000);
    
    // Create FULL image buffer for complete 800x480 display
    UBYTE *BlackImage;
    UDOUBLE Imagesize = ((EPD_7IN3F_WIDTH % 2 == 0)? (EPD_7IN3F_WIDTH / 2 ): (EPD_7IN3F_WIDTH / 2 + 1)) * EPD_7IN3F_HEIGHT;
    
    if((BlackImage = (UBYTE *)malloc(Imagesize)) == NULL) {
        printf("Failed to allocate full frame buffer!\r\n");
        printf("Make sure PSRAM is enabled for ESP32 WROVER\r\n");
        while (1);
    }
    
    printf("Buffer allocated successfully\r\n");
    Paint_NewImage(BlackImage, EPD_7IN3F_WIDTH, EPD_7IN3F_HEIGHT, 0, EPD_7IN3F_WHITE);
    Paint_SetScale(7);
    
    printf("Drawing shipping label...\r\n");
    Paint_SelectImage(BlackImage);
    Paint_Clear(EPD_7IN3F_WHITE);
    
    // ========== HEADER SECTION (Sender Info) ==========
    // Sender name - left aligned
    Paint_DrawString_EN(40, 20, label.senderName, &Font16, EPD_7IN3F_WHITE,EPD_7IN3F_BLACK);
    
    // Sender address - left aligned, smaller font
    Paint_DrawString_EN(40, 45, label.senderAddress, &Font12, EPD_7IN3F_WHITE,EPD_7IN3F_BLACK);
    
    // Horizontal divider line under header
    Paint_DrawLine(30, 75, EPD_7IN3F_WIDTH - 30, 75, EPD_7IN3F_BLACK, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    
    // ========== MAIN CONTENT AREA ==========
    // Left Column
    int leftMargin = 40;
    int rightColumnX = 480;
    
    // Vertical divider between left and right columns
    Paint_DrawLine(rightColumnX - 20, 85, rightColumnX - 20, 320, EPD_7IN3F_BLACK, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    
    // "Ship To:" label
    Paint_DrawString_EN(leftMargin, 95, label.shipToLabel, &Font16, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
    
    // Recipient name - larger font, no highlight
    Paint_DrawString_EN(leftMargin, 125, label.recipientName, &Font24, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
    
    // Recipient address
    Paint_DrawString_EN(leftMargin, 160, label.recipientAddress, &Font20, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
    
    // Weight - no highlight
    Paint_DrawString_EN(leftMargin, 200, label.weight, &Font24, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
    
    // MAXICODE placeholder
    drawMaxicode(leftMargin, 240, 70);
    
    // Routing code and Postal code
    Paint_DrawString_EN(leftMargin + 90, 250, label.routingCode, &Font20, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
    Paint_DrawString_EN(leftMargin + 90, 275, label.postalCode, &Font16, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
    
    // ========== RIGHT COLUMN ==========
    // Service type box
    Paint_DrawRectangle(rightColumnX, 95, rightColumnX + 100, 135, 
                       EPD_7IN3F_BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    Paint_DrawString_EN(rightColumnX + 15, 105, label.serviceType, &Font16, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
    
    // SERVICE label box
    Paint_DrawRectangle(rightColumnX + 110, 95, rightColumnX + 230, 135, 
                       EPD_7IN3F_BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    Paint_DrawString_EN(rightColumnX + 125, 105, label.service, &Font16, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
    
    // QR Code - positioned lower
    char qrUrl[256];
    generateQRUrl(qrUrl, DEVICE_ID);
    printf("QR Code URL: %s\r\n", qrUrl);
    drawQRCode(rightColumnX + 60, 160, qrUrl, 4);
    
    // ========== TRACKING NUMBER SECTION ==========
    // Horizontal divider above tracking number section
    Paint_DrawLine(30, 330, EPD_7IN3F_WIDTH - 30, 330, EPD_7IN3F_BLACK, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    
    // Large tracking number - centered
    int textWidth = strlen(label.trackingNumber) * 20;  // Font24 approximate width
    int centerX = (EPD_7IN3F_WIDTH - textWidth) / 2;
    Paint_DrawString_EN(centerX, 345, label.trackingNumber, &Font24,EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
    
    // ========== BARCODE SECTION ==========
    // Full width barcode
    int barcodeWidth = EPD_7IN3F_WIDTH - 80;  // Leave 40px margin on each side
    int barcodeX = 40;
    drawBarcode(barcodeX, 385, barcodeWidth, 50, label.barcodeNumber);
    
    // Barcode numbers below - positioned at barcode edges
    Paint_DrawString_EN(barcodeX + 10, 440, label.leftBarcode, &Font16, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
    Paint_DrawString_EN(barcodeX + barcodeWidth - 60, 440, label.rightBarcode, &Font16, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
    
    // ========== DISPLAY FULL FRAME ==========
    printf("Sending full frame to display...\r\n");
    EPD_7IN3F_Display(BlackImage);
    
    printf("Display complete! Label will remain on screen.\r\n");
    printf("QR code links to: %s\r\n", qrUrl);
    
    // Keep display active for testing
    // free(BlackImage);  // Keep commented for testing
}

void loop() {
    // Nothing to do
}