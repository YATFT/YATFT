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
#include "YATFT.h"

///////////////////////////////////////////////////////////////////////////////

#if !defined(__INT_MAX__) || (__INT_MAX__ > 0xFFFF)
 #define pgm_read_pointer(addr) ((void *)pgm_read_dword(addr))
#else
 #define pgm_read_pointer(addr) ((void *)pgm_read_word(addr))
#endif

void  YATFT::begin(uint16_t id) {
    uint8_t  i = 0; gpioStatus = 0; wrReg8(0x250, 0x80);
    reset(); delay(10); wrReg8(0xA0, 0x00); delay(200);
    wrReg8(0xA8, 0x1F); wrReg8(0xA9, 0x80);
    gpio_bl(0); gpio_spena(1); gpio_spclk(1); gpio_spdat(1); gpio_sprst(1);
    gpio_sprst(0); delay(1); gpio_sprst(1);
    spi_setregb(0x00, 0x07); spi_setregb(0x01, 0x18); spi_setregb(0x02, 0x13);
    spi_setregb(0x03, 0xC9); spi_setregb(0x04, 0x46); spi_setregb(0x05, 0x0D);
    spi_setregb(0x06, 0x00); spi_setregb(0x07, 0x02); spi_setregb(0x08, 0x08);
    spi_setregb(0x09, 0x40); spi_setregb(0x0A, 0x88); spi_setregb(0x0B, 0x88);
    spi_setregb(0x0C, 0x20); spi_setregb(0x0D, 0x20); spi_setregb(0x0E, 0x75);
    spi_setregb(0x0F, 0xA4); spi_setregb(0x10, 0x04); spi_setregb(0x11, 0x24);
    spi_setregb(0x12, 0x24); spi_setregb(0x1E, 0x00); spi_setregb(0x20, 0x00);
    gpio_spclk(0); gpio_bl(1); wrReg8(0x126, 0x0A); wrReg8(0x127, 0xB4);
    wrReg8(0x12B, 0xAE); wrReg8(0x126, 0x8A); wrReg8(0x04, 0x00); delay(20);
    wrReg32(0x158, 0x13FFF); wrReg8(0x10, 0x72); wrReg8(0x11, 0x00);
    wrReg8(0x12, 0x32); wrReg8(0x13, 0x07); wrReg8(0x22, 0x00);
    wrReg8(0x23, 0x00); wrReg8(0x20, DISP_HOR_PULSE_WIDTH - 1);
    wrReg8(0x21, 0x00); wrReg8(0x16, 0x44); wrReg8(0x17, 0x00);
    wrReg8(0x14, 0x27); wrReg8(0x18, 0x05); wrReg8(0x19,0x01);
    wrReg8(0x1C, 0xEF); wrReg8(0x1D, 0x00); wrReg8(0x1E, 0x12);
    wrReg8(0x1F, 0x00); wrReg8(0x24, 0x01); wrReg8(0x26, 0x00);
    wrReg8(0x27, 0x00); wrReg8(0xA0, 0x00);
    MainWndEnable(false); MainWndInit(0, 320, 0x04, 0, 1);
    FloatWndInit(253600L, 1, 1, 1, 1, 1, 1);
    FocusWnd(0);
    MainWndEnable(1);
}

uint16_t  YATFT::DrawArc(int16_t xL, int16_t yT, int16_t xR, int16_t yB, int16_t r1, int16_t r2, uint8_t octant) {
    typedef enum { BEGIN, QUAD11, BARRIGHT1, QUAD12, BARRIGHT2, QUAD21, BARLEFT1, QUAD22, BARLEFT2, QUAD31,
                   BARTOP1, QUAD32, BARTOP2, QUAD41, BARBOTTOM1, QUAD42, BARBOTTOM2, CHECK } OCTANTARC_STATES;
    DWORD_VAL  temp;
    static int16_t  y1Limit, y2Limit, x1, x2, y1, y2, err1, err2, x1Cur, y1Cur, y1New, x2Cur, y2Cur, y2New;
    static OCTANTARC_STATES state = BEGIN;
    while(1)
    {
        switch(state)
        {
            case BEGIN:
                 temp.Val = SIN45*r1; y1Limit  = temp.w[1]; temp.Val = SIN45*r2; y2Limit  = temp.w[1];
                 temp.Val = (uint32_t)(ONEP25 -((int32_t)r1<<16)); err1  = (int16_t)(temp.w[1]);
                 temp.Val = (uint32_t)(ONEP25 -((int32_t)r2<<16)); err2  = (int16_t)(temp.w[1]);
                 x1 = r1; x2 = r2; y1 = 0; y2 = 0; x1Cur = x1; y1Cur = y1; y1New = y1;
                 x2Cur = x2; y2Cur = y2; y2New = y2; state = CHECK;
            case CHECK:
arc_check_state:
                 if (y2 > y2Limit) { state = BARRIGHT1; goto arc_draw_width_height_state; }
                 // y1New & y2New records the last y positions
                 y1New = y1; y2New = y2;
                 if (y1 <= y1Limit) {
                     if (err1 > 0) { x1--;  err1 += 5; err1 += (y1-x1)<<1; }
                     else { err1 += 3; err1 += y1<<1; } y1++;
                 } else { y1++; if (x1 < y1)  x1 = y1; }
                 if (err2 > 0) { x2--; err2 += 5; err2 += (y2-x2)<<1; }
                 else { err2 += 3; err2 += y2<<1; }  y2++; state = QUAD11;
                 break;
            case QUAD11:
                 if ((x1Cur != x1) || (x2Cur != x2)) {
                     // 1st octant
                     if (octant&0x01)  DrawFillRect(xR+y2Cur, yT-x2Cur, xR+y1New, yT-x1Cur);
                 } else { state = CHECK; goto arc_check_state; } state = QUAD12;
                 break;
            case QUAD12:  // 2nd octant
                 if (octant&0x02)  DrawFillRect(xR+x1Cur, yT-y1New, xR+x2Cur, yT-y2Cur);  state = QUAD21;
                 break;
            case QUAD21:  // 3rd octant
                 if (octant&0x04)  DrawFillRect(xR+x1Cur, yB+y1Cur, xR+x2Cur, yB+y2New);  state = QUAD22;
                 break;
            case QUAD22:  // 4th octant
                 if (octant&0x08)  DrawFillRect(xR+y1Cur, yB+x1Cur, xR+y2New, yB+x2Cur);  state = QUAD31;
                 break;
            case QUAD31:  // 5th octant
                 if (octant&0x10)  DrawFillRect(xL-y1New, yB+x1Cur, xL-y2Cur, yB+x2Cur);  state = QUAD32;
                 break;
            case QUAD32:  // 6th octant
                 if (octant&0x20)  DrawFillRect(xL-x2Cur, yB+y2Cur, xL-x1Cur, yB+y1New);  state = QUAD41;
                 break;
            case QUAD41:  // 7th octant
                 if (octant&0x40)  DrawFillRect(xL-x2Cur, yT-y2New, xL-x1Cur, yT-y1Cur);  state = QUAD42;
                 break;
            case QUAD42:  // 8th octant
                 if (octant&0x80)  DrawFillRect(xL-y2New, yT-x2Cur, xL-y1Cur, yT-x1Cur);
                 x1Cur = x1; y1Cur = y1; x2Cur = x2; y2Cur = y2; state = CHECK;
                 break;
            case BARRIGHT1: // draw upper right
arc_draw_width_height_state:
                 if ((xR-xL) || (yB-yT)) { if (octant&0x02)  DrawFillRect(xR+r1,yT,xR+r2,(yB+yT)>>1); }
                 else { state = BEGIN; return 1; } state = BARRIGHT2;
                 break;
            case BARRIGHT2:  // draw lower right
                 if (octant&0x04)  DrawFillRect(xR+r1,((yB+yT)>>1), xR+r2, yB);  state = BARBOTTOM1;
                 break;
            case BARBOTTOM1:  // draw left bottom
                 if (octant&0x10)  DrawFillRect(xL, yB+r1, ((xR+xL)>>1),yB+r2);  state = BARBOTTOM2;
                 break;
            case BARBOTTOM2:  // draw right bottom
                 if (octant&0x08)  DrawFillRect(((xR+xL)>>1),yB+r1,xR,yB+r2);  state = BARTOP1;
                 break;
            case BARTOP1:  // draw left top
                 if (xR-xL) { if (octant&0x80)  DrawFillRect(xL, yT-r2, ((xR+xL)>>1),yT-r1);
                     state = BARTOP2;
                 } else state = BARLEFT1; // no width go directly to height bar
                 break;
            case BARTOP2:  // draw right top
                 if (octant&0x01)  DrawFillRect(((xR+xL)>>1),yT-r2,xR,yT-r1);  state = BARLEFT1;
                 break;
            case BARLEFT1:  // draw upper left
                 if (yT-yB) { if (octant&0x40)  DrawFillRect(xL-r2,yT,xL-r1,((yB+yT)>>1));
                     state = BARLEFT2;
                 } else { state = BEGIN;  // no height go back to BEGIN
                     return 1;
                 }
                 break;
            case BARLEFT2:  // draw lower left
                 if (octant&0x20)  DrawFillRect(xL-r2,((yB+yT)>>1),xL-r1,yB);  state = BEGIN; return 1;
        }// end of switch
    }// end of while
}

#define RED8(color16)   (uint8_t) ((color16 & 0xF800) >> 8)
#define GREEN8(color16) (uint8_t) ((color16 & 0x07E0) >> 3)
#define BLUE8(color16)  (uint8_t) ((color16 & 0x001F) << 3)

uint16_t  YATFT::DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
//    uint16_t sw;  if (x1 > x2) { sw = x2; x2 = x1; x1 = sw;} if (y1 > y2) { sw = y2; y2 = y1; y1 = sw;}
    wrReg8(0x1E4, x1 & 0xFF); wrReg8(0x1E5, (x1 >> 8) & 0xFF); wrReg8(0x1E8, y1 & 0xFF);
    wrReg8(0x1E9, (y1 >> 8) & 0xFF); wrReg8(0x1EC, x2 & 0xFF); wrReg8(0x1ED, (x2 >> 8) & 0xFF);
    wrReg8(0x1F0, y2 & 0xFF); wrReg8(0x1F1, (y2 >> 8) & 0xFF); wrReg8(0x1D4, 0);
    wrReg8(0x1D5, 0); wrReg8(0x1D6, 0); wrReg8(0x1F4, 0); wrReg8(0x1F5, 0); wrReg8(0x1F6, 0);
    wrReg8(0x1F8, (GetMaxX() + 1) & 0xFF); wrReg8(0x1F9, ((GetMaxX() + 1) >> 8) & 0xFF);
    wrReg8(0x1D8, (GetMaxY() + 1) & 0xFF); wrReg8(0x1D9, ((GetMaxY() + 1) >> 8) & 0xFF);
    wrReg8(0x1FE, RED8(_color)); wrReg8(0x1FD, GREEN8(_color)); wrReg8(0x1FC, BLUE8(_color));
    wrReg8(0x1DD, 0x00); wrReg8(0x1D1, 0x01); wrReg8(0x1D2, 0x01);
    return (1);
}

void  YATFT::DrawBevel(int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t rad) {
    int16_t  style, type, xLimit, xPos, yPos, error; DWORD_VAL  temp;	
    temp.Val = SIN45*rad; xLimit   = temp.w[1]; temp.Val = (uint32_t)(ONEP25 -((int32_t)rad<<16));
    error = (int16_t)(temp.w[1]); yPos = rad; style = 0; type =1;
    if (rad) {
        for (xPos=0; xPos <=xLimit; xPos++) {
            if ((++style)==_lineType) { type ^=1; style = 0;}
            if (type) {
                PutPixel(x2+xPos, y1-yPos); PutPixel(x2+yPos, y1-xPos); // 1st quadrant
                PutPixel(x2+xPos, y2+yPos); PutPixel(x2+yPos, y2+xPos); // 2nd quadrant
                PutPixel(x1-xPos, y2+yPos); PutPixel(x1-yPos, y2+xPos); // 3rd quadrant
                PutPixel(x1-yPos, y1-xPos); PutPixel(x1-xPos, y1-yPos); // 4th quadrant
            }
            if (error > 0) { yPos--; error += 5+((xPos-yPos)<<1);}
            else  error += 3+(xPos<<1);
        }
    } 
    if (x2-x1)  DrawLine(x1,y1-rad,x2,y1-rad);
    if (y2-y1)  DrawLine(x1-rad,y1,x1-rad,y2);
    if ((x2-x1) || (y2-y1)) { DrawLine(x2+rad,y1,x2+rad,y2); DrawLine(x1,y2+rad,x2,y2+rad);}
}

uint16_t  YATFT::DrawFillBevel(int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t rad) {
    typedef enum { BEGIN, CHECK, Q8TOQ1, Q7TOQ2, Q6TOQ3, Q5TOQ4, WAITFORDONE, FACE} FILLCIRCLE_STATES;
    DWORD_VAL  temp; static int32_t  err; static int16_t  yLimit, xPos, yPos, xCur, yCur, yNew;
    FILLCIRCLE_STATES state = BEGIN;
    while (1) {
        switch(state) {
           case BEGIN:  
                if (!rad) { state = FACE; break; }
                // compute variables
                temp.Val = SIN45*rad;  yLimit   = temp.w[1];
                temp.Val = (uint32_t)(ONEP25 -((int32_t)rad<<16));
                err = (int16_t)(temp.w[1]); xPos = rad; yPos = 0;
                xCur = xPos; yCur = yPos; yNew = yPos; state = CHECK;
           case CHECK:
bevel_fill_check:
                if (yPos>yLimit) { state = FACE; break; }
                // y1New records the last y position
                yNew = yPos;
                // calculate the next value of x and y
                if (err > 0) { xPos--; err += 5+((yPos-xPos)<<1);}
                else  err += 3+(yPos<<1);
                yPos++;	state = Q6TOQ3;
           case Q6TOQ3:
                if (xCur != xPos) {
                    // 6th octant to 3rd octant
                    DrawFillRect(x1-xCur, y2+yCur, x2+xCur, y2+yNew);
                    state = Q5TOQ4;
                    break;
                }
                state = CHECK;
                goto bevel_fill_check;
           case Q5TOQ4:  // 5th octant to 4th octant
                DrawFillRect(x1-yNew, y2+xPos, x2+yNew, y2+xCur);  state = Q8TOQ1;
                break;
           case Q8TOQ1:  // 8th octant to 1st octant
                DrawFillRect(x1-yNew, y1-xCur, x2+yNew, y1-xPos);  state = Q7TOQ2;
                break;
           case Q7TOQ2:  // 7th octant to 2nd octant
                DrawFillRect(x1-xCur, y1-yNew, x2+xCur, y1-yCur);
                // update current values
                xCur = xPos;  yCur = yPos; state = CHECK;
                break;
           case FACE:
                if ((x2-x1)||(y2-y1)) {
                    DrawFillRect(x1-rad, y1, x2+rad, y2); state = WAITFORDONE;
                } else { state = BEGIN; return 1; }
           case WAITFORDONE:
                state = BEGIN;
                return 1;	
        }
    }
}

void  YATFT::DrawPoly(int16_t numPoints, int16_t* polyPoints) {
    int16_t  counter, sx, sy, ex, ey;
    sx = *polyPoints++; sy = *polyPoints++;
    for (counter = 0; counter < numPoints - 1; counter++) {
        ex = *polyPoints++; ey = *polyPoints++; DrawLine(sx,sy,ex,ey); sx = ex; sy = ey;
    }
}

void  YATFT::DrawFillRect(int16_t left, int16_t top, int16_t right, int16_t bottom) {
    uint16_t sw;
    if (left > right) { sw = right; right = left; left = sw;}
    if (top > bottom) { sw = bottom; bottom = top; top = sw;}
    wrReg8(0x1D4, 0); wrReg8(0x1D5, 0); wrReg8(0x1D6, 0); wrReg8(0x1F4, 0);
    wrReg8(0x1F5, 0); wrReg8(0x1F6, 0); wrReg8(0x1F8, (GetMaxX() + 1) & 0xFF);
    wrReg8(0x1F9, ((GetMaxX() + 1) >> 8) & 0xFF); wrReg8(0x1D8, (GetMaxY() + 1) & 0xFF);
    wrReg8(0x1D9, ((GetMaxY() + 1) >> 8) & 0xFF); wrReg8(0x1FE, RED8(_color));
    wrReg8(0x1FD, GREEN8(_color)); wrReg8(0x1FC, BLUE8(_color)); wrReg8(0x1DD, 0x00);
    wrReg8(0x1D1, 0x01); int16_t x,y;
    for (y=top; y<bottom+1; y++) {
        wrReg8(0x1E4, left & 0xFF); wrReg8(0x1E5, (left >> 8) & 0xFF);
        wrReg8(0x1EC, (right) & 0xFF); wrReg8(0x1ED, ((right) >> 8) & 0xFF);
        wrReg8(0x1E8, y & 0xFF); wrReg8(0x1E9, (y >> 8) & 0xFF); wrReg8(0x1F0, y & 0xFF);
        wrReg8(0x1F1, (y >> 8) & 0xFF); wrReg8(0x1D1, 0x01); wrReg8(0x1D2, 0x01);
    }
    return;
}

void  YATFT::PutPixel(int16_t x, int16_t y) {
    int32_t address; address = PAGE_MEM_SIZE*_page + (_line_mem_pitch<<1)*y + (x<<1);
    WR_CD_ACTIVE; RD_IDLE; SetAddress(address); RD_CD_IDLE; WR_ACTIVE; WriteData(_color);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef read8isFunctionalized
  #define read8(x) x=read8fn()
#endif

#ifdef readport8isFunctionalized
  #define readport8(x) x=readport8fn()
#endif

uint8_t  INTRFC::rdReg8(uint16_t r) {
    static  uint8_t x; RD_IDLE; WR_CD_ACTIVE; write8(0x00); write8((r)>>8); write8(r);
    RD_ACTIVE; setReadDir(); WR_CD_IDLE; read8(x); read8(x); RD_IDLE; setWriteDir(); return x;
}

uint16_t  INTRFC::rdReg16(uint16_t r) {
    static WORD_VAL x; RD_IDLE; WR_CD_ACTIVE; write8(0x00); write8((r) >> 8); write8(r);
    RD_ACTIVE; setReadDir(); WR_CD_IDLE; read8(x.v[0]); read8(x.v[0]); read8(x.v[1]);
    RD_IDLE; setWriteDir(); return x.Val;
}

uint32_t  INTRFC::rdReg32(uint16_t r) {
    static DWORD_VAL x; RD_IDLE; WR_CD_ACTIVE; write8(0x00); write8((r) >> 8); write8(r);
    RD_ACTIVE;setReadDir(); WR_CD_IDLE; read8(x.v[0]); read8(x.v[0]); read8(x.v[1]);
    read8(x.v[2]); read8(x.v[3]); RD_IDLE; setWriteDir(); return x.Val;
}

void  INTRFC::GetMemBuff(uint32_t address, uint8_t * buff, uint16_t length) {
    uint8_t  value;  uint16_t  i; SetAddress(address); setReadDir(); WR_CS_CD_IDLE; RD_CS_ACTIVE;
    for (i = 0; i < length; i++) { CS_IDLE; CS_IDLE; CS_ACTIVE; CS_ACTIVE; readport8(*buff); buff++;}
    RD_CS_IDLE; setWriteDir();
}

uint8_t  INTRFC::GetMem(uint32_t address) {
    uint8_t  value;  uint16_t  i; SetAddress(address); setReadDir();
    CS_IDLE; CD_DATA; WR_IDLE; RD_ACTIVE; CS_ACTIVE;
    for (i = 0; i < 1; i++) { CS_IDLE; CS_IDLE; _NOP(); _NOP(); CS_ACTIVE; CS_ACTIVE; value = readport8fn();}
    CS_IDLE; RD_IDLE; setWriteDir(); return value;
}

void  INTRFC::PutMemBuff(uint32_t address, uint8_t * buff, uint16_t length) {
    uint8_t  value;  uint16_t  i; SetAddress(address); RD_CD_IDLE; WR_ACTIVE;
    for (i = 0; i < length; i++) { write8(*buff); buff++;} WR_IDLE; CS_IDLE;
}

    /////////////////////////////////////////////////////////////////////
    // ROTATION MODE

    #if (DISP_ORIENTATION == 0)
        #define WIN_START_ADDR  0ul
        #define ROTATION        0

    #elif (DISP_ORIENTATION == 90)
        #ifndef USE_PALETTE
            #define WIN_START_ADDR  ((((uint32_t) GetMaxX() + 1) >> 1) - 1)
        #else
            #define WIN_START_ADDR  (((((uint32_t) GetMaxX() + 1) * PaletteBpp) >> 5) - 1)
        #endif
        #define ROTATION    1

    #elif (DISP_ORIENTATION == 180)
        #ifndef USE_PALETTE
            #define WIN_START_ADDR  (((((uint32_t) GetMaxX() + 1) * ((uint32_t)GetMaxY() + 1))              >> 1) - 1)
        #else
            #define WIN_START_ADDR  (((((uint32_t) GetMaxX() + 1) * ((uint32_t)GetMaxY() + 1) * PaletteBpp) >> 5) - 1)
        #endif
        #define ROTATION    2

    #elif (DISP_ORIENTATION == 270)
        #ifndef USE_PALETTE
            #define WIN_START_ADDR  ((((uint32_t) GetMaxX() + 1) * GetMaxY()) >> 1)
        #else
            #define WIN_START_ADDR  ((((uint32_t) GetMaxX() + 1) * GetMaxY() * PaletteBpp) >> 5)
        #endif
        #define ROTATION    3
    #endif

void  YATFT::MainWndInit(uint32_t startaddr, uint16_t linewidth, uint16_t bpp, uint8_t orient, uint8_t rgb) {
#if (ROTATION == 0)
    wrReg32(0x74, (uint32_t)startaddr/4+(uint32_t)WIN_START_ADDR);
#else
    wrReg32(0x74, ((uint32_t)WIN_START_ADDR+(uint32_t)startaddr/4));
#endif
    wrReg8(0x70, (rdReg8(0x70)|bpp)); wrReg16(0x78,(uint16_t)linewidth>>1);
    wrReg8(0x71, ((rdReg8(0x71)|0x40|ROTATION))); wrReg8(0x1A4, ((rdReg8(0x1A4)&0xC0)|0x40));
    if(rgb) wrReg8(0x1A4, (rdReg8(0x1A4)&0xC0)|0x40);
    else    wrReg8(0x1A4, rdReg8(0x1A4) & ~0x40);
}

void  YATFT::FloatWndInit(uint32_t startaddr, uint16_t linewidth, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t rgb) {
#if (ROTATION == 0)
    wrReg16(0x84,x>>1); wrReg16(0x8C,((x+width)>>1)-1); wrReg16(0x88,y);
    wrReg16(0x90,y+height-1); wrReg32(0x7C, startaddr>>2);
    wrReg16(0x80,linewidth>>1); wrReg8(0x70, rdReg8(0x70)|4);
    if (rgb)  wrReg8(0x1A4, (rdReg8(0x1A4)&0xC0)|0x80);  //RGB mode, set bit7 '1
    else      wrReg8(0x1A4, rdReg8(0x1A4) & ~0x80);        //YUV mode, set bit7 '0
#else
    #if (ROTATION == 2)
        wrReg16(0x84,x>>1); wrReg16(0x8C,((x+width)>>1)-1); wrReg16(0x88,y);
        wrReg16(0x90,y+height-1); wrReg32(0x7C, (startaddr)>>2);
        wrReg16(0x80,(linewidth)>>1); wrReg8(0x70, rdReg8(0x70)|4);
        if (rgb)  wrReg8(0x1A4, (rdReg8(0x1A4)&0xC0)|0x80);
        else      wrReg8(0x1A4, rdReg8(0x1A4)&~0x80);
    #endif
#endif
}

void  YATFT::FocusWnd(uint8_t wnd) {
    uint16_t  linewidth;
    if (wnd) { _page=1; linewidth=(uint16_t)rdReg8(0x81)<<8;
        linewidth=linewidth|(rdReg8(0x80)&0x00FF);
        linewidth=linewidth<<1; _line_mem_pitch=linewidth;
    } else { _page=0; _line_mem_pitch=LINE_MEM_PITCH;}
}

void  YATFT::SetFont(const GFXfont *f) {
    if(f) { if(!gfxFont) { cursor_y+=6;}}
    else if(gfxFont) { cursor_y-=6;}
    gfxFont=(GFXfont *)f;
}

uint16_t  YATFT::OutText(unsigned char* textString) {
    char  ch; static uint16_t counter = 0;
    while ((unsigned char)(ch = *(textString+counter)) > (unsigned char)15) { OutChar(ch); counter++;}
    counter = 0;
    return 1;
}

void  YATFT::OutChar(unsigned char  ch) {
    GLYPH_ENTRY * pChTable;  uint8_t * pChImage, temp, mask, adv, w;
    int8_t  yo, yA; int16_t  chWidth, xCnt, yCnt, x, y;
    if (!gfxFont) { // 'Classic' built-in font
        chWidth=5; _fontHeight=8; x=GetX(); 
        for (xCnt=0; xCnt<chWidth; xCnt++) {
            y=GetY(); mask=0x01; temp=pgm_read_byte(&font[ch*5+xCnt]);
            for (yCnt=0; yCnt<_fontHeight; yCnt++) {        
                if (temp&mask) { PutPixel(x,y);} y++; mask<<=1;
            } x++;
        } x++;
    } else { // Custom font
        // Character is assumed previously filtered by write() to eliminate
        // newlines, returns, non-printable characters, etc.  Calling
        // drawChar() directly with 'bad' characters of font may cause mayhem!
        ch -= (uint8_t)pgm_read_byte(&gfxFont->first);
        yA =  (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
        GFXglyph *glyph=&(((GFXglyph *)pgm_read_pointer(&gfxFont->glyph))[ch]);
        uint8_t  *bitmap=(uint8_t *)pgm_read_pointer(&gfxFont->bitmap);
        uint16_t bo=pgm_read_word(&glyph->bitmapOffset);
        uint8_t  h=pgm_read_byte(&glyph->height); w=pgm_read_byte(&glyph->width);
        int8_t  xo=pgm_read_byte(&glyph->xOffset); yo=pgm_read_byte(&glyph->yOffset);
        uint8_t  xx, yy, bits = 0, bit = 0; int16_t  xo16 = 0, yo16 = 0;
        x=GetX(); y=GetY()+yA*3/4; 
        for (yy = 0; yy < h; yy++) {
            for (xx = 0; xx < w; xx++) {        
                if (!(bit++&7)) { bits=pgm_read_byte(&bitmap[bo++]);}
                if (bits&0x80) { 
                    PutPixel((int16_t)x+(int16_t)xo+(int16_t)xx, (int16_t)y+(int16_t)yo+(int16_t)yy);
                }
                bits <<= 1;
            }
        }
    } // End classic vs custom font
    // move cursor
    _cursorX = x + w + 1;
}

int16_t  YATFT::GetTextWidth(char * textString, void* font) {
    GFXfont * pHeader = (GFXfont*)font; int16_t   textWidth; char  ch;
    if (font != NULL) { // User Font
        textWidth = 0;
        while((unsigned char)15<(unsigned char)(ch=*textString++)) {
            ch-=(uint8_t)pgm_read_byte(&pHeader->first);
            GFXglyph *glyph=&(((GFXglyph *)pgm_read_pointer(&pHeader->glyph))[ch]);
            uint8_t w=pgm_read_byte(&glyph->width)+1; textWidth+=w;
        }
    } else { // Default Font
        textWidth=0;
        while((unsigned char)15<(unsigned char)(ch = *textString++)) { textWidth+=7;}
    }
    return textWidth;
}

int16_t  YATFT::GetTextHeight(void* font) {
    GFXfont * pHeader = (GFXfont*)font;
    if ((font) != NULL) { return (uint16_t)pgm_read_byte(&pHeader->yAdvance);}
    else { return 8;}
}

uint16_t  YATFT::OutTextXY(int16_t x, int16_t y, char * textString) {
    static uint8_t start = 1; if (start) { MoveTo(x,y); start = 0;}
    if (OutText(textString) == 0) { return 0;}
    else { start = 1; return 1;}
}

/*********************************************************************
*  DS1307
*********************************************************************/
uint8_t  DS1307::isrunning(void) {
    I2C_Init(); I2C_SetID(DS1307_ADDRESS); I2C_Ctl(MASK_I2C_START|MASK_I2C_WRITE);
    I2C_Write(_I2C_address); I2C_Ctl(MASK_I2C_WRITE); I2C_Write(0);
    I2C_Ctl(MASK_I2C_START|MASK_I2C_WRITE); I2C_Write(_I2C_address | 0x01);
    I2C_Ctl(MASK_I2C_READ_WO_ACK); uint8_t ss = I2C_Read();
    I2C_Ctl(MASK_I2C_STOP); I2C_Write(0xFF); return !(ss>>7);
}

bool  DS1307::read(tmElements_t &tm) {
    uint8_t sec; I2C_Init(); I2C_SetID(DS1307_ADDRESS); I2C_Ctl(MASK_I2C_START|MASK_I2C_WRITE);
    I2C_Write(_I2C_address); I2C_Ctl(MASK_I2C_WRITE); I2C_Write(0);
    I2C_Ctl(MASK_I2C_START|MASK_I2C_WRITE); I2C_Write(_I2C_address | 0x01);
    I2C_Ctl(MASK_I2C_READ_W_ACK); sec = I2C_Read(); tm.Second = bcd2dec(sec & 0x7f);   
    I2C_Ctl(MASK_I2C_READ_W_ACK); tm.Minute = bcd2dec(I2C_Read());
    I2C_Ctl(MASK_I2C_READ_W_ACK); tm.Hour   = bcd2dec(I2C_Read() & 0x3f);  // mask assumes 24hr clock
    I2C_Ctl(MASK_I2C_READ_W_ACK); tm.Wday   = bcd2dec(I2C_Read());
    I2C_Ctl(MASK_I2C_READ_W_ACK); tm.Day    = bcd2dec(I2C_Read());
    I2C_Ctl(MASK_I2C_READ_W_ACK); tm.Month  = bcd2dec(I2C_Read());
    I2C_Ctl(MASK_I2C_READ_WO_ACK); tm.Year   = y2kYearToTm((bcd2dec(I2C_Read())));
    I2C_Ctl(MASK_I2C_STOP); I2C_Write(0xFF);
    if (sec & 0x80) return false; // clock is halted
    return true;
}

bool  DS1307::write(tmElements_t &tm) {
    I2C_Init(); I2C_SetID(DS1307_ADDRESS); I2C_Ctl(MASK_I2C_START|MASK_I2C_WRITE);
    I2C_Write(_I2C_address); I2C_Ctl(MASK_I2C_WRITE); I2C_Write(0);
    I2C_Ctl(MASK_I2C_WRITE); I2C_Write((uint8_t)0x80); // Stop the clock. The seconds will be written last
    I2C_Ctl(MASK_I2C_WRITE); I2C_Write(dec2bcd(tm.Minute));
    I2C_Ctl(MASK_I2C_WRITE); I2C_Write(dec2bcd(tm.Hour)); // sets 24 hour format
    I2C_Ctl(MASK_I2C_WRITE); I2C_Write(dec2bcd(tm.Wday));   
    I2C_Ctl(MASK_I2C_WRITE); I2C_Write(dec2bcd(tm.Day));
    I2C_Ctl(MASK_I2C_WRITE); I2C_Write(dec2bcd(tm.Month));
    I2C_Ctl(MASK_I2C_WRITE); I2C_Write(dec2bcd(tmYearToY2k(tm.Year))); 
    I2C_Ctl(MASK_I2C_WRITE); I2C_Write(0x11); I2C_Ctl(MASK_I2C_STOP);
    I2C_Write(0xFF); I2C_Ctl(MASK_I2C_START|MASK_I2C_WRITE);
    I2C_Write(_I2C_address); I2C_Ctl(MASK_I2C_WRITE);
    I2C_Write(0); I2C_Ctl(MASK_I2C_WRITE); I2C_Write(dec2bcd(tm.Second));
    return true;
}


