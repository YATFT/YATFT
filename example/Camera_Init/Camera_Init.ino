/*
 * Arduino TFT_shield_v1.00 Camera OV7670 initialization example
 * 
 * Author: Kamil Lee (2019)
 * 
 * Comments: Camera OV7670 initialization
 * 
 * 
 */

#include <YATFT.h> // Hardware-specific library
#include <util/yacam.h>

#include "ov7670_regs.h"

YATFT tft(0);
CAM   cam;


/*******************************************************************************
*
*******************************************************************************/
void setup() {

    uint8_t state;

    // initialize the serial port
    Serial.begin(115200);
    Serial.println("Camera initialization example!");

    // initialize the display
    tft.begin();

    // Camera OV7670 initialize
    cam.CamInit(&OV7670_VGA[0][0]);
    cam.CamVideoPreview(0, 0, 1, true);
}

void loop()
{
}

