/*
 * Arduino TFT_shield_SD2TFT example
 * 
 * Author: Kamil Lee (2018)
 * 
 * Comments:
 * This sketch shows jpeg pictures and rgb video on TFT screen
 */

#include <YATFT.h>   // Hardware-specific library
#include <util/yasdc.h>

YATFT tft(0);
SDC   sdc;

/*************************************************************************************************/
void  setup(void)
{
#if defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__) 
    pinMode(13, INPUT); pinMode(12, INPUT); pinMode(11, INPUT); // set pins to input
#endif

    Serial.begin(115200);  // initialize the serial port
    Serial.println(F("TFT_shield_SD2TFT example!"));

    tft.begin();           // initialize the display
    tft.SetColor(BLACK);
    tft.ClearDevice();     // Clear screen

    Serial.print(F("FS Init... "));
    uint8_t state = sdc.FSInit();
    if (state == 0)  Serial.println(F("unsuccess."));
    else             Serial.println(F("success."));
    sdc.currentFileType = DEMO_FILE_TYPE_RGB;
}

void  loop(void)
{
    if(-1 != sdc.FindFirst("*.*", ATTR_ARCHIVE, &sdc.nextFile))
    {
        do
        {
            if (sdc.GetFileType(sdc.nextFile.filename) == DEMO_FILE_TYPE_JPEG)
            {
                if (sdc.currentFileType != DEMO_FILE_TYPE_JPEG)
                {
                    // Set YUV mode to display JPEG
                    tft.SetColor(0x007F); // Black in YUV
                    tft.ClearDevice();
                    tft.SetYUV();         // Switching shows a little green flicker
                    sdc.currentFileType = DEMO_FILE_TYPE_JPEG;
                }
                Serial.print(F("View JPEG image: "));
                Serial.println(sdc.nextFile.filename);

                FSFILE * jpeg_file;
                JPEG_DECODE  jpeg_decode;
                jpeg_file = sdc.FSfopen(sdc.nextFile.filename, "r");
                if (!jpeg_file) { Serial.println(F("Open failed!")); return (FALSE);}
                jpeg_decode.stream = (void *)jpeg_file;
                sdc.JPEGReadFromSD(&jpeg_decode, 0, 0, GetMaxX()+1, GetMaxY()+1);
                uint8_t err = sdc.FSfclose(jpeg_file);
                if (err) { Serial.println(F("Close failed!")); return err;}

                delay(1000);
            }
            else if (sdc.GetFileType(sdc.nextFile.filename) == DEMO_FILE_TYPE_RGB)
            {
                if (sdc.currentFileType != DEMO_FILE_TYPE_RGB)
                {
                    // Set RGB mode to display RGB
                    tft.SetColor(BLACK);  // Black in RGB is dark green in YUV
                    tft.ClearDevice();
                    tft.SetRGB();         // Switching shows a little green flicker
                    sdc.currentFileType = DEMO_FILE_TYPE_RGB;
                }
                Serial.print(F("Play RGB video: "));
                Serial.println(sdc.nextFile.filename);

                FSFILE * pFile = sdc.FSfopen(sdc.nextFile.filename, "rb");
                if (!pFile) { Serial.println(F("Open failed!")); return (FALSE);}
                tft.SetColor(BLACK); tft.ClearDevice();
                sdc.RGBReadFromSD(pFile, NULL);
                tft.SetColor(BLACK); tft.ClearDevice();
                sdc.FSfclose(pFile);
                delay(1000);
            }
        } while(-1 != sdc.FindNext(&sdc.nextFile));
    }
}
