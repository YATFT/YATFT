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
#include "yasrl.h"

uint8_t  SRL::SerialFile2JPEGFIFO(uint32_t fifoAddress, uint32_t byteSize) {
    static bool frame = false;
    static bool ff = false;
    static bool d9 = false;
    static bool d8 = false;
    uint16_t shift_word = 0;
    uint16_t shift_prev = 0;
    uint16_t len = 0;
    static bool odd = false;
    uint16_t cnt = 1;
    static bool send_data = false;
    static bool send_end = false;
    SetAddress(fifoAddress);
    while(1) {
        uint16_t d;
        uint16_t d_odd;
        uint16_t sd;
        uint16_t temp;

        if (Serial.available()>0) {

            temp=Serial.read();
            cnt++;
            if (cnt&1) { d |= temp; if (frame==true) send_data = true; }
            else       d = temp<<8;
            if (temp==0xFF) ff = true;
            else {
                if (ff==true) {
                    if (temp==0xD8) { frame = true; cnt = 1; d = 0xFFD8; send_data = true;}
                    if (temp==0xD9) {
                        frame = false;
                        if (cnt&1) { send_data = true;}
                        else { d = 0xD900; cnt++; send_data = true;}
                        send_end = true;
                    }
                    ff = false;
                }
            }
            if (send_data == true) {
                WriteData(d);
                send_data = false;
            }
            if (send_end==true) {
                for (uint16_t i=0; i<=(JPEG_FIFO_SIZE-cnt)/2; i++) WriteData(0);
                send_end = false;
                break;
            }
        }
    }
    return (TRUE);
}

uint8_t  SRL::JPEGReadFromSerial(JPEG_DECODE * jpeg_decode, uint16_t left, uint16_t top, uint16_t right, uint16_t bot, uint32_t size)
{
    int err=TRUE; err=SerialJPEGRegsSetup(jpeg_decode, size); err=SerialJPEGHeader(jpeg_decode);
    if (err == NO_ERR) {
        SerialJPEGResize(jpeg_decode, left, top, right, bot);
        wrReg8(0x382,7); wrReg8(0x383,0x30); wrReg8(0x38A,1);
        err = SerialJPEGData(jpeg_decode);
        wrReg8(0x380,0x10); wrReg8(0x38A,0);
    }
    return err;
}

int  SRL::SerialJPEGRegsSetup(JPEG_DECODE * decode, uint32_t size) {
    uint32_t  sz; int err; decode->bytes_read = 0; wrReg8(0x41E, 0);
    wrReg8(0x3A4, (uint8_t) (JPEG_FIFO_BLK_NUMBER - 1)); decode->fifo_addr = JPEG_FIFO_START_ADDR;
    decode->fifo_addr_end = JPEG_FIFO_START_ADDR + (JPEG_FIFO_BLK_NUMBER * JPEG_FIFO_BLK_SIZE);
    sz = JPEG_FIFO_START_ADDR/4; wrReg32(0x414, sz); wrReg8(0x380, 0x11);
    wrReg8(0x402, 0x80); delay(1); wrReg8(0x402, 0); wrReg8(0x41C, 2); wrReg8(0x41E, 0);
    if (err)  return err; decode->size = size; wrReg32(0x3B8, size);
    wrReg8(0x400, 4); wrReg8(0x360, 1); wrReg8(0x402, 1);
    return err;
}

JPEG_ERR  SRL::SerialJPEGHeader(JPEG_DECODE * decode) {
    uint8_t  cnt = 0;
    while (cnt++ < 250)
    {
        if (rdReg8(0x41E)) return (ERR_DECODE);
        if (rdReg8(0x385) & 0x10) {
            uint16_t   size;
            size = (uint16_t)rdReg8(0x3D8)|((uint16_t)(rdReg8(0x3D8+1)))<<8;
            decode->image_width=size;
            size = (uint16_t)rdReg8(0x3DC)|((uint16_t)(rdReg8(0x3DC+1)))<<8;
            decode->image_height=size; decode->op_mode=rdReg8(0x401);
            return (NO_ERR);
        }
        if (!(rdReg8(0x384) & 0x01)) continue;
        if (decode->bytes_read < decode->size) {
            if (!SerialFile2JPEGFIFO(JPEG_FIFO_START_ADDR, JPEG_FIFO_SIZE)) { }
            decode->bytes_read+=JPEG_FIFO_SIZE;
        } else return (ERR_NO_DATA);
    }
    return (ERR_DECODE);
}

void  SRL::SerialJPEGResize(JPEG_DECODE * decode, uint16_t left, uint16_t top, uint16_t right, uint16_t bot) {
    const uint32_t __align[] = { 8, 16, 16, 32}; uint16_t size; uint8_t div = 1;
    uint16_t _maxX=right-left, _maxY=bot-top;
    if (right<left) _maxX=left-right; if (bot<top) _maxY=top-bot;
    if ((decode->image_height>(_maxY))||(decode->image_width>(_maxX))) {
        div = 2;
        while (div<8) {
            if (((decode->image_height/div)<=_maxY)&&((decode->image_width/div)<=_maxX)) break;
            div<<=1;
        }
    }
    wrReg8(0x36C,div); wrReg8(0x36E,2); decode->display_height=div*(GetMaxY()+1);
    decode->display_width=div*(GetMaxX()+1); uint32_t destAddr; uint32_t x, y;
    x=(right+left-decode->image_width/div)/2; y=(bot+top-decode->image_height/div)/2;
    destAddr = ((y*(GetMaxX()+1)+x)>>1); destAddr &= ~0x0001; wrReg32(0x410, destAddr);
    decode->display_height=decode->display_height&(~(__align[decode->op_mode]-1));
    decode->display_width =decode->display_width &(~(__align[decode->op_mode]-1));
    size = decode->display_width-1; wrReg16(0x364,0); wrReg16(0x368,size);
    size = decode->display_height-1; wrReg16(0x366,0); wrReg16(0x36A,size);
}

JPEG_ERR  SRL::SerialJPEGData(JPEG_DECODE * decode) {
    while (1) {
        if (rdReg8(0x41E)) return (ERR_DECODE);
        if (rdReg8(0x404)&0xF8) return (ERR_DECODE);
        if (rdReg8(0x385)==0x22) return (NO_ERR);
        if (!(rdReg8(0x384)&0x01)) continue;
        if (decode->bytes_read<decode->size) {
            if(!SerialFile2JPEGFIFO(JPEG_FIFO_START_ADDR, JPEG_FIFO_SIZE)) { }
            decode->bytes_read += JPEG_FIFO_SIZE;
        } else  continue;
    }
}
