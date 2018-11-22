/******************************************************************************
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY
 * OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR
 * PURPOSE. IN NO EVENT SHALL AUTHOR OR ITS LICENSORS BE LIABLE OR
 * OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,
 * BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT
 * DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL,
 * INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA,
 * COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY
 * CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
 * OR OTHER SIMILAR COSTS.
*******************************************************************************/
#ifdef __AVR__
    #include <avr/pgmspace.h>
#endif // __AVR__
#include "../YATFT.h"
#include "yaosd.h"

///////////////////////////////////////////////////////////////////////////////

void  OSD::OSDInit(uint32_t start_adr, uint16_t width, uint16_t height) {
    if (width < 16) width = 16;
    if (height < 1) height = 1;
    wrReg32(0xCC, start_adr);
    wrReg16(0xD0, 0x0000);
    wrReg16(0xD4, 0x0000);
    wrReg16(0xD8, (width/16-1)); // 16 pixels increment, e.g. 0000h = 16 pixels; 0001h = 32 pixels 
    wrReg16(0xDC, (height-1)); // 1 line increment, e.g. 0000h = 1 line; 0001h = 2 lines
}

void  OSD::OSDBlink(uint16_t total, uint16_t on) {
    wrReg16(0xC4, total);
    wrReg16(0xC8, on);
}

void  OSD::OSDColor1(uint16_t color1) {
    wrReg16(0xE0, color1);
}

void  OSD::OSDColor2(uint16_t color2) {
    wrReg16(0xE4, color2);
}

void  OSD::OSDColor3(uint16_t color3) {
    wrReg16(0xE8, color3);
}

void  OSD::OSDEnable(bool enable) {
    if (enable) wrReg8(0xC0, 0x80);
    else        wrReg8(0xC0, 0x00);
}
