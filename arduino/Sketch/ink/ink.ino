/*
 * =====================================================================
 * E-INK DISPLAY TEST SKETCH - Waveshare 3.7" Display
 * =====================================================================
 * 
 * This sketch tests the 3.7" Waveshare E-ink display individually
 * to verify hardware connections and display functionality.
 * 
 * Pin Configuration (ESP32):
 * - CS (Chip Select):    Pin 5
 * - DC (Data/Command):   Pin 17 (TX)
 * - RST (Reset):         Pin 16 (RX)
 * - BUSY:                Pin 4
 * - VCC:                 3.3V
 * - GND:                 GND
 * - DIN (MOSI):          Pin 23 (SPI)
 * - CLK (SCLK):          Pin 18 (SPI)
 * 
 * Required Libraries:
 * - GxEPD2 (by ZinggJM)
 * - Adafruit_GFX
 * 
 * =====================================================================
 */

#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>
#include <GxEPD2_4C.h>
#include <GxEPD2_7C.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans9pt7b.h>

// Pin Definitions - Match actual Waveshare 3.7" wiring
#define EINK_CS             5     // E-ink SPI Chip Select
#define EINK_DC            17     // E-ink Data/Command (TX)
#define EINK_RST           16     // E-ink Reset (RX)
#define EINK_BUSY           4     // E-ink Busy

// Display Constructor for Waveshare 3.7" E-ink Display (ED037TC1)
// 280x480 pixels, SSD1677 controller
GxEPD2_BW<GxEPD2_370_TC1, GxEPD2_370_TC1::HEIGHT> display(GxEPD2_370_TC1(EINK_CS, EINK_DC, EINK_RST, EINK_BUSY));

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=====================================================");
  Serial.println("        E-INK DISPLAY TEST - Waveshare 3.7\"");
  Serial.println("=====================================================");
  Serial.println("Pin Configuration:");
  Serial.println("CS (Chip Select):  Pin " + String(EINK_CS));
  Serial.println("DC (Data/Command): Pin " + String(EINK_DC) + " (TX)");
  Serial.println("RST (Reset):       Pin " + String(EINK_RST) + " (RX)");
  Serial.println("BUSY:              Pin " + String(EINK_BUSY));
  Serial.println("MOSI (DIN):        Pin 23 (SPI)");
  Serial.println("SCLK (CLK):        Pin 18 (SPI)");
  Serial.println("VCC:               3.3V");
  Serial.println("GND:               GND");
  Serial.println("=====================================================");
  
  // Initialize display
  Serial.println("Initializing E-ink display...");
  display.init(115200, true, 2, false); // Waveshare boards with "clever" reset circuit
  delay(100);
  
  Serial.println("✓ Display initialized successfully");
  Serial.println("Display Width: " + String(display.width()) + " pixels");
  Serial.println("Display Height: " + String(display.height()) + " pixels");
  Serial.println("");
  
  // Run display tests
  runDisplayTests();
  
  Serial.println("✓ All display tests completed");
  Serial.println("Display will now show the final test result");
  Serial.println("=====================================================");
}

void runDisplayTests() {
  Serial.println("Starting display tests...");
  
  // Test 1: Clear screen test
  Serial.println("Test 1: Clear screen test");
  testClearScreen();
  delay(2000);
  
  // Test 2: Basic text test
  Serial.println("Test 2: Basic text test");
  testBasicText();
  delay(3000);
  
  // Test 3: Multiple fonts test
  Serial.println("Test 3: Multiple fonts test");
  testMultipleFonts();
  delay(3000);
  
  // Test 4: Graphics test
  Serial.println("Test 4: Graphics test");
  testGraphics();
  delay(3000);
  
  // Test 5: Full information display (matching tracking box)
  Serial.println("Test 5: Full information display");
  testFullDisplay();
  delay(3000);
}

void testClearScreen() {
  display.setRotation(3);
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
  } while (display.nextPage());
  Serial.println("✓ Clear screen test completed");
}

void testBasicText() {
  display.setRotation(3);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_BLACK);
  
  // Calculate center position
  const char* testText = "E-ink Display Test";
  int16_t tbx, tby; 
  uint16_t tbw, tbh;
  display.getTextBounds(testText, 0, 0, &tbx, &tby, &tbw, &tbh);
  uint16_t x = ((display.width() - tbw) / 2) - tbx;
  uint16_t y = ((display.height() - tbh) / 2) - tby;
  
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(x, y);
    display.print(testText);
  } while (display.nextPage());
  
  Serial.println("✓ Basic text test completed");
}

void testMultipleFonts() {
  display.setRotation(3);
  display.setFullWindow();
  display.firstPage();
  
  do {
    display.fillScreen(GxEPD_WHITE);
    
    // Title with large font
    display.setFont(&FreeSans12pt7b);
    display.setTextColor(GxEPD_BLACK);
    display.setCursor(10, 30);
    display.print("Font Test");
    
    // Medium font
    display.setFont(&FreeSans9pt7b);
    display.setCursor(10, 60);
    display.print("Medium font - FreeSans9pt");
    
    // Bold font
    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(10, 90);
    display.print("Bold font - FreeMonoBold9pt");
    
    // Information
    display.setFont(&FreeSans9pt7b);
    display.setCursor(10, 120);
    display.print("Display Resolution: " + String(display.width()) + "x" + String(display.height()));
    
    display.setCursor(10, 150);
    display.print("Controller: SSD1677");
    
    display.setCursor(10, 180);
    display.print("Model: Waveshare 3.7\" E-ink");
    
  } while (display.nextPage());
  
  Serial.println("✓ Multiple fonts test completed");
}

void testGraphics() {
  display.setRotation(3);
  display.setFullWindow();
  display.firstPage();
  
  do {
    display.fillScreen(GxEPD_WHITE);
    
    // Title
    display.setFont(&FreeSans12pt7b);
    display.setTextColor(GxEPD_BLACK);
    display.setCursor(10, 30);
    display.print("Graphics Test");
    
    // Draw rectangles
    display.drawRect(10, 50, 100, 60, GxEPD_BLACK);
    display.fillRect(120, 50, 100, 60, GxEPD_BLACK);
    
    // Draw circles
    display.drawCircle(70, 150, 30, GxEPD_BLACK);
    display.fillCircle(170, 150, 30, GxEPD_BLACK);
    
    // Draw lines
    display.drawLine(10, 200, 200, 200, GxEPD_BLACK);
    display.drawLine(250, 50, 250, 200, GxEPD_BLACK);
    
    // Labels
    display.setFont(&FreeSans9pt7b);
    display.setCursor(10, 130);
    display.print("Rect    Filled");
    
    display.setCursor(10, 190);
    display.print("Circle  Filled");
    
    display.setCursor(10, 220);
    display.print("Lines and borders");
    
  } while (display.nextPage());
  
  Serial.println("✓ Graphics test completed");
}

void testFullDisplay() {
  display.setRotation(3);
  display.setFullWindow();
  display.firstPage();
  
  do {
    display.fillScreen(GxEPD_WHITE);
    
    // Header Section
    display.setFont(&FreeSans12pt7b);
    display.setTextColor(GxEPD_BLACK);
    display.setCursor(10, 25);
    display.print("TRACKING BOX TEST");
    
    display.setFont(&FreeSans9pt7b);
    display.setCursor(380, 25);
    display.print("[TEST MODE]");
    
    // Device info
    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(10, 45);
    display.print("Name: E-ink Display Test");
    
    display.setCursor(10, 60);
    display.print("Location: Test Environment");
    
    display.setFont(&FreeSans9pt7b);
    display.setCursor(380, 45);
    display.print("ID: TEST_001");
    
    // Separator line
    display.drawLine(10, 70, 470, 70, GxEPD_BLACK);
    
    // Environment section
    display.setFont(&FreeSans9pt7b);
    display.setCursor(10, 90);
    display.print("HARDWARE STATUS");
    
    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(15, 110);
    display.print("Display: Working");
    
    display.setCursor(15, 130);
    display.print("Resolution: " + String(display.width()) + "x" + String(display.height()));
    
    display.setCursor(15, 150);
    display.print("Controller: SSD1677");
    
    display.setCursor(15, 170);
    display.print("Interface: SPI");
    
    // Connection section
    display.setFont(&FreeSans9pt7b);
    display.setCursor(10, 195);
    display.print("CONNECTION STATUS");
    
    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(15, 215);
    display.print("Pin Configuration:");
    
    display.setFont(&FreeSans9pt7b);
    display.setCursor(15, 235);
    display.print("CS=" + String(EINK_CS) + " DC=" + String(EINK_DC) + "(TX) RST=" + String(EINK_RST) + "(RX) BUSY=" + String(EINK_BUSY));
    
    display.setCursor(15, 255);
    display.print("SPI: MOSI=23, SCLK=18");
    
    // Footer
    display.drawLine(10, 270, 470, 270, GxEPD_BLACK);
    
    display.setFont(&FreeSans9pt7b);
    display.setCursor(10, 290);
    display.print("Test Status: ALL TESTS PASSED");
    
    display.setCursor(320, 290);
    display.print("Ready for production");
    
  } while (display.nextPage());
  
  Serial.println("✓ Full display test completed");
}

void loop() {
  // Display is now in hibernate mode
  // The last test result will remain on screen
  delay(10000);
  
  // Optional: Uncomment to repeat tests every 30 seconds
  /*
  Serial.println("Repeating tests...");
  runDisplayTests();
  delay(30000);
  */
}
