/*
 * Arduino TFT_shield Example_02
 * 
 * Author: Kamil Lee (2018)
 * 
 * Comments:
 * This sketch demonstrate text & graphics.
 * Additionaly need Adafruit-GFX, XPT2046_Touchscreen, SPI library's
 */

#include <YATFT.h>                       // Hardware-specific library
#include <SPI.h>                         // Include SPI library
#include <XPT2046_Touchscreen.h>         // Include Touchscreen library
#include <Adafruit_GFX.h>                // Include Adafruit-GFX library
#include <Fonts/FreeSerif9pt7b.h>        // Include Adafruit fonts
#include <Fonts/FreeSerifItalic24pt7b.h>
#include <Fonts/FreeSans24pt7b.h>

// Touchscreen: MOSI=11, MISO=12, SCK=13, CS=2
#define CS_PIN  2
XPT2046_Touchscreen ts(CS_PIN, 255);

YATFT tft(0);

#define  Y_BAR_TOP      (GetMaxY()-50)
#define  Y_BAR_BOT      GetMaxY()
#define  BUTTON1_MASK   0x01
#define  BUTTON2_MASK   0x02
#define  BUTTON3_MASK   0x04
#define  BUTTON4_MASK   0x08
#define  BUTTON5_MASK   0x10

/* 
   If using the shield, all control and data lines are fixed, and
   a simpler declaration can optionally be used:
*/
//uint32_t  total_time;
uint16_t  pos_x[] = {0,0,0,0};
uint16_t  pos_y[] = {0,0,0,0};
uint8_t   pos_x_cnt = 0;
uint8_t   pos_y_cnt = 0;
uint16_t  pos_x_mid = 0;
uint16_t  pos_y_mid = 0;
uint16_t  color_paint = WHITE;
uint8_t   buttons = 0;
uint16_t  Color[4] = {BRIGHTBLUE, BRIGHTGREEN, BRIGHTRED, BRIGHTYELLOW};
uint16_t  Gray[7] = {GRAY0, GRAY1, GRAY2, GRAY3, GRAY4, GRAY5, GRAY6};

/*************************************************************************************************/
void ClearScreen (void)
{
    tft.SetColor(BLACK);   // Set fone color
    tft.ClearDevice();     // Fill all screen
}

void setup()
{
    Serial.begin(115200);  // initialize the serial port
    Serial.println("Arduino TFT_shield Example1!");
    ts.begin();            // Init Touchscreen
    SPI.end();             // Disable SPI for correct work DB2 (SS) pin 
    tft.begin();           // initialize the display

    RefreshWindow();
}

void loop()
{
    uint16_t  x, y;

    // Touch
    // When the SS pin is set as OUTPUT, it can be used as
    // a general purpose output port (it doesn't influence
    // SPI operations).
    SPI.begin();
    if (ts.touched())
    {
        TS_Point p = ts.getPoint();
            
        Serial.print(F("Pressure = "));
        Serial.print(p.z);
        Serial.print(F(", x = "));
        Serial.print(p.x);
        Serial.print(F(", y = "));
        Serial.print(p.y);
        Serial.println();
        delay(3);     // Delay for filtering

        SPI.end();  // Disable SPI for correct work DB2 (SS) pin 

        // Calculate coordinates x, y from code ADC
        if (p.x < 200) p.x = 200;
        if (p.y < 250) p.y = 250;
#if 0
        x = (uint16_t)(320L - ((uint32_t)p.x - 200L)*10L/115L);
        y = (uint16_t)(((uint32_t)p.y - 250L)/15L);
#else
//      x = (uint16_t)(320L - ((uint32_t)p.y - 200L)*10L/115L);
        x = (uint16_t)(0 + ((uint32_t)p.y - 200L)*10L/115L);
        y = (uint16_t)(((uint32_t)p.x - 250L)/15L);
#endif
        // Filtering 
        pos_x_mid = (pos_x[0] + pos_x[1] + pos_x[2] + pos_x[3])/4;
        pos_y_mid = (pos_y[0] + pos_y[1] + pos_y[2] + pos_y[3])/4;
        pos_x[pos_x_cnt++] = x;
        pos_y[pos_y_cnt++] = y;
        pos_x_cnt &= 0x03;
        pos_y_cnt &= 0x03;

        if (x > (pos_x_mid - 10) && x < (pos_x_mid + 10) && y > (pos_y_mid - 10) && y < (pos_y_mid + 10 )) {
              
            if (y > Y_BAR_TOP && y < Y_BAR_BOT) {
                if (x < 1*(GetMaxX()+1)/5) {  // Touch Bar 1
                    color_paint = Color[0];
                    RefreshTitle();
                } else
                if (x < 2*(GetMaxX()+1)/5) {  // Touch Bar 2
                    color_paint = Color[1];
                    RefreshTitle();
                } else
                if (x < 3*(GetMaxX()+1)/5) {  // Touch Bar 3
                    color_paint = Color[2];
                    RefreshTitle();
                } else
                if (x < 4*(GetMaxX()+1)/5) {  // Touch Bar 4
                    color_paint = Color[3];
                    RefreshTitle();
                } else {                     // Clear screen
                    RefreshWindow();
                }
            } else {
                tft.SetColor(color_paint);
                tft.DrawFillRect(x-1, y-1, x+1, y+1);
            }
        }
    }
    SPI.end();  // Disable SPI for correct work DB2 (SS) pin 

    ScanKey();
}

void  RefreshWindow(void)
{
    color_paint = WHITE;
    ClearScreen();
    for (uint8_t i = 0; i < 4; i++) {
        tft.SetColor(Color[i]);
        tft.DrawFillRect((i+1)*((GetMaxX()+1)/5), Y_BAR_TOP, (i)*((GetMaxX()+1)/5), Y_BAR_BOT);
    }
    RefreshTitle();
    tft.SetColor(WHITE);
    tft.OutTextXY(GetMaxX() - 50, GetMaxY() - 45, "Clear");
    tft.OutTextXY(GetMaxX() - 55, GetMaxY() - 25, "screen");
}

void  RefreshTitle(void)
{
    tft.SetColor(color_paint);
    tft.SetFont(&FreeSerif9pt7b);
    tft.OutTextXY(3, 20, "Touch     color     bar     and     screen     or     press     key.");
}

void  ScanKey(void)
{
    static uint8_t buttons_last = 0;

    buttons = tft.scanButtons();

    if (buttons != buttons_last) {
        if (buttons & BUTTON1_MASK) {  // Bar 1
            color_paint = Color[0];
            RefreshTitle();
        }
        if (buttons & BUTTON2_MASK) {  // Bar 2
            color_paint = Color[1];
            RefreshTitle();
        }
        if (buttons & BUTTON3_MASK) {  // Bar 3
            color_paint = Color[2];
            RefreshTitle();
        }
        if (buttons & BUTTON4_MASK) {  // Bar 4
            color_paint = Color[3];
            RefreshTitle();
        }
        if (buttons & BUTTON5_MASK) {  // Clear screen
            RefreshWindow();
        }
    }
    buttons_last = buttons;
}
