/*
 * Arduino TFT_shield Example_01
 * 
 * Author: Kamil Lee (2018)
 * 
 * Comments:
 * This sketch demonstrate text & graphics.
 * Additionaly need Adafruit-GFX library
 */

#include <YATFT.h>                       // Hardware-specific library
#include <Adafruit_GFX.h>                // Include Adafruit-GFX library
#include <Fonts/FreeSerif9pt7b.h>        // Include Adafruit fonts
#include <Fonts/FreeSerifItalic24pt7b.h>
#include <Fonts/FreeSans24pt7b.h>

YATFT tft(0);

uint32_t  total_time;
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
    tft.begin();           // initialize the display
}

void loop()
{
    uint16_t  x, y, x2, y2, mask_gray;
    uint16_t  i;

    ClearScreen();

    // Fonts
    tft.SetColor(BRIGHTBLUE);
    tft.SetFont(NULL);
    tft.OutTextXY(5, 5, "Demonstration of work with the TFT display.");
    tft.SetColor(BRIGHTGREEN);
    tft.SetFont(&FreeSerif9pt7b);
    tft.OutTextXY(5, 20, "The   example   uses   fonts   from   Adafruit.");
    tft.SetFont(&FreeSerifItalic24pt7b);
    tft.SetColor(BRIGHTCYAN);
    tft.OutTextXY(5, 45, "3,5''");
    tft.SetColor(BRIGHTRED);
    tft.OutTextXY(90, 45, "QVGA");
    tft.SetColor(BRIGHTMAGENTA);
    tft.OutTextXY(230, 45, "disp.");
    tft.SetColor(BRIGHTYELLOW);
    tft.SetFont(&FreeSans24pt7b);
    tft.OutTextXY(5, 100, "A R D U I N O      +  T F T");

    tft.SetFont(NULL);
    for (i = 0; i < 7; i++)
    {
        tft.SetColor(Gray[i]);
        tft.OutTextXY(5, 170+10*i, "Demonstration of work with the TFT display.");
    }

    delay(3000);
    ClearScreen();

    // Circle
    tft.SetColor(BRIGHTRED);
    for (i = 10; i < GetMaxY()>>1; i += 10) {
        tft.DrawCirc(GetMaxX()>>1, GetMaxY()>>1, i);
    }

    delay(1000);

    // DrawFillCircle & DrawFillRect
    tft.SetColor(BRIGHTRED);
    tft.DrawFillCirc(GetMaxX()>>1,GetMaxY()>>1,110);
    tft.SetColor(BRIGHTCYAN);
    tft.DrawFillRect(GetMaxX()/2-77,GetMaxY()/2-77, GetMaxX()/2+77,GetMaxY()/2+77);
    tft.SetColor(BRIGHTGREEN);
    tft.DrawFillCirc(GetMaxX()>>1,GetMaxY()>>1,77);
    tft.SetColor(BRIGHTMAGENTA);
    tft.DrawFillRect(GetMaxX()/2-54,GetMaxY()/2-54, GetMaxX()/2+54,GetMaxY()/2+54);
    tft.SetColor(BRIGHTBLUE);
    tft.DrawFillCirc(GetMaxX()>>1,GetMaxY()>>1,54);
    tft.SetColor(BRIGHTYELLOW);
    tft.DrawFillRect(GetMaxX()/2-37,GetMaxY()/2-37, GetMaxX()/2+37,GetMaxY()/2+37);

    delay(1000);
    ClearScreen();

    // Arc
    ClearScreen();
    tft.SetColor(BRIGHTBLUE);
    tft.DrawArc((GetMaxX()>>1)-60,(GetMaxY()>>1)-60,(GetMaxX()>>1)+60,(GetMaxY()>>1)+60,20,30,0xFF);
    tft.SetColor(BRIGHTGREEN);
    tft.DrawArc((GetMaxX()>>1)-40,(GetMaxY()>>1)-40,(GetMaxX()>>1)+40,(GetMaxY()>>1)+40,20,30,0xFF);
    tft.SetColor(BRIGHTRED);
    tft.DrawArc((GetMaxX()>>1)-20,(GetMaxY()>>1)-20,(GetMaxX()>>1)+20,(GetMaxY()>>1)+20,20,30,0xFF);

    delay(1000);

    tft.SetColor(BRIGHTBLUE);
    tft.DrawFillBevel((GetMaxX()>>1)-60,(GetMaxY()>>1)-60,(GetMaxX()>>1)+60,(GetMaxY()>>1)+60,30);
    tft.SetColor(BRIGHTGREEN);
    tft.DrawFillBevel((GetMaxX()>>1)-40,(GetMaxY()>>1)-40,(GetMaxX()>>1)+40,(GetMaxY()>>1)+40,30);
    tft.SetColor(BRIGHTRED);
    tft.DrawFillBevel((GetMaxX()>>1)-20,(GetMaxY()>>1)-20,(GetMaxX()>>1)+20,(GetMaxY()>>1)+20,30);

    delay(1000);
    ClearScreen();

    for (i = 0; i < 4; i++) {
        tft.SetColor(Color[i]);
        tft.DrawArc((GetMaxX()>>1),(GetMaxY()>>1)-50,(GetMaxX()>>1),(GetMaxY()>>1)+50,50,60,0x11<<i);
    }
    for (i = 0; i < 4; i++) {
        tft.SetColor(Color[i]);
        tft.DrawArc((GetMaxX()>>1),(GetMaxY()>>1)-30,(GetMaxX()>>1),(GetMaxY()>>1)+30,35,45,0x11<<i);
    }
    for (i = 0; i < 4; i++) {
        tft.SetColor(Color[i]);
        tft.DrawArc((GetMaxX()>>1),(GetMaxY()>>1),(GetMaxX()>>1),(GetMaxY()>>1),20,30,0x11<<i);
    }

    delay(1000);
    ClearScreen();

    // Draw 1000 random lines
    for (i = 0; i < 1000; i++) {
        tft.SetColor(random(65535));
        x  = random(320);
        y  = random(240);
        x2 = random(320);
        y2 = random(240);
        tft.DrawLine(x, y, x2, y2);
    }
    delay(1000);
}
