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
#include "yacodec.h"

#define  SHIFT_ADDR   4096*8

void  CODEC::JPEGSetRegs(uint32_t jpeg_width, uint32_t jpeg_height) {
    uint32_t vpixel, hpixel; uint8_t vpixel_8, hpixel_8;
    uint32_t exp_jpeg_width, exp_jpeg_height; uint32_t line_buf, vert_buf;
    uint32_t raw_jpeg_addr=(uint32_t)0x30000L;
    uint32_t encoded_jpeg_addr=(uint32_t)raw_jpeg_addr+(uint32_t)SHIFT_ADDR;
    char count_h=0; char count_v=0;
    exp_jpeg_width=jpeg_width; exp_jpeg_height=jpeg_height;
    wrReg8(0x402,0x80); wrReg16(0x386,0); wrReg16(0x382,0x308F);
    wrReg16(0x386,0x3687); wrReg8(0x3A0,0x12); wrReg8(0x400,9);
    while(_param.previewCropHSize>=(jpeg_width*=2)) {count_h++;}
    while(_param.previewCropVSize>=(jpeg_height*=2)) {count_v++;}
    wrReg8(0x172,count_h); wrReg8(0x173,count_v);
    if (exp_jpeg_width%16!=0) exp_jpeg_width =((exp_jpeg_width/16)*16)*(1<<count_h);
    else                      exp_jpeg_width =exp_jpeg_width*(1<<count_h);
    if (exp_jpeg_height%8!=0) exp_jpeg_height=((exp_jpeg_height/8)*8)*(1<<count_v);
    else                      exp_jpeg_height=exp_jpeg_height*(1<<count_v);
    line_buf=exp_jpeg_width/(1<<(rdReg8(0x172)+2))-1; wrReg16(0x164,line_buf);
    vert_buf=exp_jpeg_height/(1<<(rdReg8(0x173)))-1; wrReg16(0x168,vert_buf);
    hpixel=exp_jpeg_width; vpixel=exp_jpeg_height; hpixel_8=hpixel&0x00FF;
    wrReg8(0x9F6,hpixel_8); hpixel_8=(hpixel>>8)&0x00FF; wrReg8(0x9F4,hpixel_8);
    vpixel_8=vpixel&0x00FF; wrReg8(0x9F2,vpixel_8); vpixel_8=(vpixel>>8)&0x00FF;
    wrReg8(0x9F0,vpixel_8); uint32_t max_size=0x40000L-encoded_jpeg_addr;
    wrReg32(0x410,raw_jpeg_addr>>2); wrReg32(0x414,encoded_jpeg_addr>>2);
    wrReg8 (0x3A4,0x0F); wrReg32(0x3B0,max_size); wrReg8 (0x380,0x11);
}

void  CODEC::JPEGInit(void) {
    uint16_t  i=0, data_word=0; uint8_t data_byte=0;
    while (data_word!=REG_DEFINE_END) {
        data_word=(uint16_t)pgm_read_byte_near(&reg_init[i][0])<<8;
        data_word|=(uint16_t)pgm_read_byte_near(&reg_init[i][1]);
        data_byte=pgm_read_byte_near(&reg_init[i][2]);
        wrReg8(data_word, data_byte); i++;
    }
}

uint32_t  CODEC::JPEGStart(void) {
    uint8_t status, temp; uint8_t JRStatus0, JStatus1, JStatus0; uint32_t filesize=0;
    wrReg8(0x402,0x80); wrReg16(0x386,0); wrReg16(0x382,0x308F);
    wrReg16(0x386,0x3687); wrReg8(0x3A0,0x12); wrReg8(0x400,9);
    wrReg8(0x160,(rdReg8(0x160)|0x80)); wrReg8(0x402,1); wrReg8(0x38A,1);
    uint32_t i=0;
    while (i<0xFFFF) {
        status=rdReg8(0x404);
        if (status==0) {
            filesize=rdReg32(0x3B4); JStatus0=rdReg8(0x382);
            JStatus1=rdReg8(0x383)&0x02; JRStatus0=rdReg8(0x385)&0x02;
            if (filesize==0||JStatus1!=0x02||JRStatus0!=0x02) {
                i++; if (i==0xFFFF) {return 0;}
            } else {return filesize;}
        }
        i++; if (i==0xFFFF) {return 0;}
    }
}

