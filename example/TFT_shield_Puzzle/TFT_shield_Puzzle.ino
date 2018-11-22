/*
 * Arduino TFT_shield_Puzzle example
 * 
 * Author: S. Major (2018)
 * 
 * Comments:
 * Game "Puzzle"
 */

#include <YATFT.h>                       // Hardware-specific library
#include <SPI.h>                         // Include SPI library
#include <XPT2046_Touchscreen.h>         // Include Touchscreen library
#include <util/yasdc.h>
#include <util/yaosd.h>
#include "font_16x20.h"

YATFT   tft(0);
SDC     sdc;
OSD     osd;
INTRFC  ifc;

// Touchscreen: MOSI=11, MISO=12, SCK=13, CS=2
#define CS_PIN  2
XPT2046_Touchscreen ts(CS_PIN, 255);
// TouchScreen type A or B
#define TS_TYPE_A
//#define TS_TYPE_B


#define  SW1_MASK   0x01
#define  SW2_MASK   0x02
#define  SW3_MASK   0x04
#define  SW4_MASK   0x08
#define  SW5_MASK   0x10

// Recomended COLUMN = 4 or 8
#define  COLUMN         4
// Recomended ROW = 3 or 6
#define  ROW            3
#define  SIZE_X         (320/COLUMN)
#define  SIZE_Y         (240/ROW)
#if (COLUMN<=4)
#define  SHIFT          8
#else
#define  SHIFT          4
#endif

#define  OSD_WIDTH   320
#define  OSD_HEIGHT  240

typedef struct {
    uint8_t x;
    uint8_t y;
} MATRIX;
typedef enum {DEMO=0, WORK=1, FINISH=2} MODE;
MODE    mode = DEMO;                          // Current mode
MATRIX  matrix_space   = {COLUMN-1,ROW-1};    // Space cell
MATRIX  matrix_current = {COLUMN-1,ROW-1};    // Current active (moved) cell
MATRIX  matrix_field[COLUMN*ROW];

#define COLORS_MAX 8                          // Max OSD text colors
uint16_t colors[COLORS_MAX] = {0xFFFF,0x0000,0xF800,0x07E0,0x001F,0xFFE0,0x07FF,0xF81F};
uint8_t  num_color_3 = 0;                     // Current OSD text color

uint16_t color_3 = 0xFFFF;
uint16_t color_2 = 0x0000;
uint16_t color_1 = 0x79EF;
bool inverse = false;

/*************************************************************************************************/
void setup(void)
{
#if defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__) 
    pinMode(13, INPUT); pinMode(12, INPUT); pinMode(11, INPUT); // set pins to input
#endif

    Serial.begin(115200);  // initialize the serial port
    Serial.println(F("TFT_shield_Puzzle example!"));
    ts.begin();            // Init Touchscreen
    SPI.end();             // Disable SPI for correct work DB2 (SS) pin 
    tft.begin();           // initialize the display
    tft.SetColor(BLACK);
    tft.ClearDevice();     // Clear screen

    Serial.print(F("FS Init... "));
    uint8_t state = sdc.FSInit();
    if (state == 0)  Serial.println(F("unsuccess."));
    else             Serial.println(F("success."));
    sdc.currentFileType = DEMO_FILE_TYPE_RGB;

    // Init OSD
    ClearOSD(153600L);
    osd.OSDInit(169472, OSD_WIDTH, OSD_WIDTH);
    osd.OSDBlink(0x001F, 0x001F);
    osd.OSDColor1(color_1);
    osd.OSDColor2(color_2);
    osd.OSDColor3(color_3);
    osd.OSDEnable(false);

    if (-1 != sdc.FindFirst("*.*", ATTR_ARCHIVE, &sdc.nextFile))
        NextPicture();
}

void  loop(void)
{
    if (mode == DEMO)   { Demo(); }
    if (mode == WORK)   { Work(); }
    if (mode == FINISH) {  }
    ScanKey();
}

/************************************************************************************
 *  Move cell from (xs,ys) to (xd,yd)
 ************************************************************************************/
uint8_t  MoveCell(int16_t xs, int16_t ys, int16_t xd, int16_t yd)
{
    uint8_t  dbuff[160];
    int16_t pos, from, to, dx, dy, delta;

    if ((xs==xd)&&(ys==yd))  return 0;

    MATRIX  matrix_temp = matrix_field[xd+yd*COLUMN];
    matrix_field[xd+yd*COLUMN] = matrix_field[xs+ys*COLUMN];
    matrix_field[xs+ys*COLUMN] = matrix_temp;

    xs*=SIZE_X; ys*=SIZE_Y; xd*=SIZE_X; yd*=SIZE_Y;
    if (xs != xd) { from = xs; to = xd; delta = (xd - xs)/SIZE_X*SHIFT; dx = delta; dy = 0;}
    if (ys != yd) { from = ys; to = yd; delta = (yd - ys)/SIZE_Y*SHIFT; dy = delta; dx = 0;}

    xd = xs; yd = ys;
    for (pos=from; pos!=to; pos+=delta) {
        xd += dx; yd += dy;
        if(ys > yd) {
            for(uint32_t row=0; row<SIZE_Y; row++) {
                ifc.GetMemBuff((row+(uint32_t)ys)*640+(uint32_t)xs*2, dbuff, SIZE_X*2);
                ifc.PutMemBuff((row+(uint32_t)yd)*640+(uint32_t)xd*2, dbuff, SIZE_X*2);
            }
            if (dy!=0) {
                for(uint8_t i = 0; i<(SIZE_X*2); i++) dbuff[i] = (i&1)*127;
                for(uint8_t i = 0; i<SHIFT; i++)
                    ifc.PutMemBuff(((SIZE_Y+SHIFT-i-1)+(uint32_t)yd)*640+(uint32_t)xd*2, dbuff, SIZE_X*2);
            }
        } else {
            for(uint32_t row=SIZE_Y; row>0; row--) {
                ifc.GetMemBuff((row-1+(uint32_t)ys)*640+(uint32_t)xs*2, dbuff, SIZE_X*2);
                ifc.PutMemBuff((row-1+(uint32_t)yd)*640+(uint32_t)xd*2, dbuff, SIZE_X*2);
                if(dx>0) {
                    for(uint8_t i = 0; i<(SHIFT*2); i++) dbuff[i] = (i&1)*127;
                    ifc.PutMemBuff((row-1+(uint32_t)yd)*640+(uint32_t)xs*2, dbuff, SHIFT*2);
                }
                if(dx<0) {
                    for(uint8_t i = 0; i<(SHIFT*2); i++) dbuff[i] = (i&1)*127;
                    ifc.PutMemBuff((row-1+(uint32_t)yd)*640+(uint32_t)(xd+SIZE_X)*2, dbuff, SHIFT*2);
                }
            }
            if (dy!=0) {
                for(uint8_t i = 0; i<(SIZE_X*2); i++) dbuff[i] = (i&1)*127;
                for(uint8_t i = 0; i<SHIFT; i++)
                    ifc.PutMemBuff(((uint32_t)yd-1-i)*640+(uint32_t)xd*2, dbuff, SIZE_X*2);
            }
        }
        xs = xd; ys = yd;
    }
    return 1;
}

/************************************************************************************
 *  ScanKey:
 *    SW1 - Change OSD text color
 *    SW2 - OSD text hide
 *    SW3 - Start GAME
 *    SW4 - OSD text view
 *    SW5 - Next picture
 ************************************************************************************/
void  ScanKey(void)
{
    static uint8_t buttons_last = 0;
    uint8_t buttons = tft.scanButtons();

    if (buttons != buttons_last) {
        if (buttons & SW1_MASK) { // OSD text color change
            if (++num_color_3 >= COLORS_MAX) num_color_3 = 0;
            osd.OSDColor3(colors[num_color_3]);
        }
        if (buttons & SW2_MASK) {
            osd.OSDEnable(false); // OSD text hide
        }
        if (buttons & SW3_MASK) {
            mode = WORK;          // Start GAME
        }
        if (buttons & SW4_MASK) {
            osd.OSDEnable(true);  // OSD text view
        }
        if (buttons & SW5_MASK) {
            NextPicture();        // View next picture
        }
    }
    buttons_last = buttons;
}

/************************************************************************************
 *  TouchScreen - return touch cell coordinates
 ************************************************************************************/
MATRIX  TouchScreen(void)
{
    MATRIX  matrix;
    uint16_t  x, y;
    // Touch
    // When the SS pin is set as OUTPUT, it can be used as
    // a general purpose output port (it doesn't influence
    // SPI operations).
    SPI.begin();
    if (ts.touched())
    {
        TS_Point p = ts.getPoint();
        // Calculate coordinates x, y from code ADC
        if (p.x < 200) p.x = 200;
        if (p.y < 250) p.y = 250;
#if defined (TS_TYPE_A)
        x = (uint16_t)(320L - ((uint32_t)p.x - 200L)*10L/115L);
        y = (uint16_t)(((uint32_t)p.y - 250L)/15L);
#endif
#if defined (TS_TYPE_B)
        x = (uint16_t)(0 + ((uint32_t)p.y - 200L)*10L/115L);
        y = (uint16_t)(((uint32_t)p.x - 250L)/15L);
#endif
        matrix.x = x/SIZE_X;
        matrix.y = y/SIZE_Y;
    } else {
        matrix.x = 0xFF;
        matrix.y = 0xFF;
    }
    SPI.end();  // Disable SPI for correct work DB2 (SS) pin 
    return  matrix;
}

/************************************************************************************
 *  View next picture
 ************************************************************************************/
void  NextPicture(void)
{
    if(-1 == sdc.FindNext(&sdc.nextFile))
        sdc.FindFirst("*.*", ATTR_ARCHIVE, &sdc.nextFile);
    else
    {
        if (sdc.GetFileType(sdc.nextFile.filename) == DEMO_FILE_TYPE_JPEG)
        {
            // Set YUV mode to display JPEG
            //tft.SetColor(BLACK);   // Black in RGB is dark green in YUV
            tft.SetColor(0x007F);    // Black in YUV
            tft.ClearDevice();
            tft.SetYUV();            // Switching shows a little green flicker
            sdc.currentFileType = DEMO_FILE_TYPE_JPEG;
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
        }
    }
    matrix_space = {COLUMN-1,ROW-1};
    mode = DEMO;

    for (uint8_t row=0; row<ROW; row++) {
        for (uint8_t col=0; col<COLUMN; col++) {
            matrix_field[row*COLUMN + col] = {col, row};
        }
    }

    delay(1000);
}

/************************************************************************************
 *  Demo mode
 ************************************************************************************/
void  Demo(void)
{
    uint8_t randnum;
    static uint8_t randnum_old;
    
    randnum = random(4);
    if (randnum==((randnum_old+2)&0x3)) {
        randnum++;
        randnum&=0x3;
    }
    randnum_old = randnum;

    matrix_current = matrix_space;

    switch (randnum) {
        case 0: // Move cell LEFT
             if (matrix_space.x > 0)          matrix_current = {matrix_space.x-1,matrix_space.y};
             break;
        case 1: // Move cell DOWN
             if (matrix_space.y < (ROW-1))    matrix_current = {matrix_space.x,matrix_space.y+1};
             break;
        case 2: // Move cell RIGHT
             if (matrix_space.x < (COLUMN-1)) matrix_current = {matrix_space.x+1,matrix_space.y};
             break;
        default: // Move cell UP
             if (matrix_space.y > 0)          matrix_current = {matrix_space.x,matrix_space.y-1};
             break;
    }
    MoveCell(matrix_current.x,matrix_current.y,matrix_space.x,matrix_space.y);
    matrix_space = matrix_current;
    RefreshOSD();
}

/************************************************************************************
 *  Work mode
 ************************************************************************************/
void  Work(void)
{
    MATRIX matrix_ts = TouchScreen();
    uint8_t f_refresh = 0;
    uint8_t err = 0;

    if (matrix_ts.x<0xFF) {
        if ( ((matrix_space.x==matrix_ts.x)&&(((matrix_space.y+1)==matrix_ts.y)||(matrix_space.y==(matrix_ts.y+1)))) ||
             ((matrix_space.y==matrix_ts.y)&&(((matrix_space.x+1)==matrix_ts.x)||(matrix_space.x==(matrix_ts.x+1)))) )
        {
            MoveCell(matrix_ts.x,matrix_ts.y,matrix_space.x,matrix_space.y);
            matrix_space = matrix_ts;
            RefreshOSD();
            f_refresh = 1;
        }
    }

    if (f_refresh) {
        for (uint8_t row=0; row<ROW; row++) {
            for (uint8_t col=0; col<COLUMN; col++) {
                if(row==(ROW-1) && col==(COLUMN-1)) {
                } else {
                    if ((matrix_field[row*COLUMN + col].x!=col)||(matrix_field[row*COLUMN + col].y!=row))
                        err = 1;
                }
            }
        }
        f_refresh = 0;
   
        if (err==0) {
            FSFILE * jpeg_file;
            JPEG_DECODE  jpeg_decode;
            jpeg_file = sdc.FSfopen(sdc.nextFile.filename, "r");
            if (!jpeg_file) { Serial.println(F("Open failed!")); return (FALSE);}
            jpeg_decode.stream = (void *)jpeg_file;
            sdc.JPEGReadFromSD(&jpeg_decode, 0, 0, GetMaxX()+1, GetMaxY()+1);
            uint8_t err = sdc.FSfclose(jpeg_file);
            if (err) { Serial.println(F("Close failed!")); return err;}
            mode = FINISH;
            osd.OSDEnable(false); // OSD text hide
        }
    }
}

/************************************************************************************
 *  ClearOSD - clear OSD memmory
 ************************************************************************************/
void  ClearOSD(uint32_t address)
{
    ifc.SetAddress(address);
    for (uint16_t i = 0; i < 9600; i++) {
        ifc.WriteData(0x0000);
    }
}

/************************************************************************************
 *  d16_to_d32 - convert 16-bit mask (1 bit/pixel) to 32-bit LCD data (2 bit/pixel)
 ************************************************************************************/
uint32_t  d16_to_d32(uint16_t in_16)
{
    uint32_t out_32 = 0;
    if(in_16 & 0x8000)  out_32 |= 0x0000000C;
    if(in_16 & 0x4000)  out_32 |= 0x00000003;
    if(in_16 & 0x2000)  out_32 |= 0x000000C0;
    if(in_16 & 0x1000)  out_32 |= 0x00000030;
    if(in_16 & 0x0800)  out_32 |= 0x00000C00;
    if(in_16 & 0x0400)  out_32 |= 0x00000300;
    if(in_16 & 0x0200)  out_32 |= 0x0000C000;
    if(in_16 & 0x0100)  out_32 |= 0x00003000;
    if(in_16 & 0x0080)  out_32 |= 0x000C0000;
    if(in_16 & 0x0040)  out_32 |= 0x00030000;
    if(in_16 & 0x0020)  out_32 |= 0x00C00000;
    if(in_16 & 0x0010)  out_32 |= 0x00300000;
    if(in_16 & 0x0008)  out_32 |= 0x0C000000;
    if(in_16 & 0x0004)  out_32 |= 0x03000000;
    if(in_16 & 0x0002)  out_32 |= 0xC0000000;
    if(in_16 & 0x0001)  out_32 |= 0x30000000;
    return out_32;
}

/************************************************************************************
 *  Print2DigitOSD - print two digit (OSD)
 ************************************************************************************/
void  Print2DigitOSD(uint32_t address, uint16_t shift, uint8_t digit, bool inverse)
{
    for (uint8_t line = 0; line < 20; line++)  {
        ifc.SetAddress(address);
        char ch = digit%10 + '0' - ' ';
        uint16_t * pdata16 = pgm_read_word_near(&font_16x20_sym[ch]);
        uint16_t data16 = pgm_read_word_near(pdata16 + (19 - (uint16_t)line));
        if (inverse) data16 = ~data16;
        ifc.WriteData32(d16_to_d32(data16));
        ch = digit/10 + '0' - ' ';
        pdata16 = pgm_read_word_near(&font_16x20_sym[ch]);
        data16 = pgm_read_word_near(pdata16 + (19 - (uint16_t)line));
        if (inverse) data16 = ~data16;
        ifc.WriteData32(d16_to_d32(data16));
        address += (uint32_t)shift;
    }
}

/************************************************************************************
 *  RefreshOSD - print matrix (OSD)
 ************************************************************************************/
void  RefreshOSD(void)
{
    for (uint8_t row=0; row<ROW; row++) {
        for (uint8_t col=0; col<COLUMN; col++) {
            uint8_t d = matrix_field[row*COLUMN + col].x + matrix_field[row*COLUMN + col].y*COLUMN + 1;

            Print2DigitOSD(153600L + ((ROW-row)*SIZE_Y-(SIZE_Y/2+10))*80 + ((COLUMN-col)*(80/COLUMN)-(80/COLUMN/2+4)), 80, d, inverse);
        }
    }
}
