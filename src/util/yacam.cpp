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
#include "yacam.h"

///////////////////////////////////////////////////////////////////////////////

uint16_t  CAM::CamInit(const uint8_t * camReg) {
    uint8_t *ptr_cam_reg, reg; ptr_cam_reg=(uint8_t*)camReg + 4;
    wrReg8(0x06,(pgm_read_byte(ptr_cam_reg)&0x0F)); wrReg8(0x07,0x97); wrReg8(0x1AC,0x12);
    wrReg8(0x1AD,1); ptr_cam_reg=(uint8_t*)camReg+5; wrReg8(0x1AE,(pgm_read_byte(ptr_cam_reg)&0x06));
    wrReg8(0x172,0); wrReg8(0x173,0); wrReg8(0x161,0); wrReg16(0x162,2); ptr_cam_reg=(uint8_t*)camReg+2;
    wrReg16(0x174,(uint16_t)pgm_read_byte(ptr_cam_reg)<<4); wrReg8(0x194,4); wrReg8(0x1A8,0);
    wrReg8(0x1A9,0); wrReg8(0x1AA,0x80); wrReg8(0x1AB,0x80); ptr_cam_reg=(uint8_t*)camReg+4;
    reg=~(pgm_read_byte(ptr_cam_reg)>>6); wrReg8(0x1AF,reg); delay(2); reg^=0x01; wrReg8(0x1AF,reg);
    delay(2); reg^=0x02; wrReg8(0x1AF,reg); delay(1);
    _param.previewHRatio=1; _param.previewVRatio = 1;
    _param.previewCropHStart=0; _param.previewCropVStart=0; ptr_cam_reg=(uint8_t*)camReg+2; 
    _param.previewCropHSize=(uint16_t)pgm_read_byte(ptr_cam_reg)<<4; ptr_cam_reg++;
    _param.previewCropVSize=(uint16_t)pgm_read_byte(ptr_cam_reg)<<4;
    wrReg8(0x170, _param.previewHRatio); wrReg8(0x171, _param.previewVRatio);
    wrReg16(0x184, _param.previewCropHStart); wrReg16(0x188, _param.previewCropVStart);
    wrReg16(0x18C, _param.previewCropHSize); wrReg16(0x190, _param.previewCropVSize);
    // hardware camera initialization here
    CamHwInit(camReg);
    return 0;
}

uint16_t  CAM::CamHwInit(const uint8_t * camReg) {
    uint16_t err=0; uint8_t i=0, data_b0=0, data_b1=0, data_b2=0; uint8_t *ptr_cam_reg, reg;
    I2C_Init();
    ptr_cam_reg=(uint8_t*)camReg; uint8_t sz_byte=pgm_read_byte_near(ptr_cam_reg); // Size of byte
    if((sz_byte&0x3)==1){
        ptr_cam_reg=(uint8_t*)camReg+1; I2C_SetID(pgm_read_byte_near(ptr_cam_reg)); // I2C address (ID) 
        ptr_cam_reg=(uint8_t*)camReg+6;
        while(data_b0!=0xFF||data_b1!=0xFF) {
            data_b0=pgm_read_byte_near(ptr_cam_reg++);
            data_b1=pgm_read_byte_near(ptr_cam_reg++);
            CamCmd2Byte(data_b0, data_b1); i++;
    }   }
    if((sz_byte&0x3)==2) {
        ptr_cam_reg=(uint8_t*)camReg+1; I2C_SetID(pgm_read_byte_near(ptr_cam_reg)); // I2C address (ID) 
        ptr_cam_reg=(uint8_t*)camReg+6;
        while(data_b0!=0xFF||data_b1!=0xFF||data_b2!=0xFF) {
            data_b0=pgm_read_byte_near(ptr_cam_reg++); data_b1 = pgm_read_byte_near(ptr_cam_reg++);
            data_b2=pgm_read_byte_near(ptr_cam_reg++);
            if(data_b0!=0xFF||data_b1!=0xFF||data_b2!=0xFF) CamCmd3Byte(data_b0, data_b1, data_b2);
            i++;
        }
    }
    return err;
}

uint16_t  CAM::CamCmd2Byte(uint8_t reg, uint8_t cmd) {
    I2C_Ctl(MASK_I2C_START|MASK_I2C_WRITE); I2C_Write(_I2C_address); I2C_Ctl(MASK_I2C_WRITE);
    I2C_Write(reg); I2C_Ctl(MASK_I2C_WRITE); I2C_Write(cmd); I2C_Ctl(MASK_I2C_STOP); I2C_Write(0xFF);
}

uint16_t  CAM::CamCmd3Byte(uint8_t reg1, uint8_t reg2, uint8_t reg3) {
    I2C_Ctl(MASK_I2C_START|MASK_I2C_WRITE); I2C_Write(_I2C_address); I2C_Ctl(MASK_I2C_WRITE);
    I2C_Write(reg1); I2C_Ctl(MASK_I2C_WRITE); I2C_Write(reg2); I2C_Ctl(MASK_I2C_WRITE);
    I2C_Write(reg3); I2C_Ctl(MASK_I2C_STOP); I2C_Write(0xFF);
}

void  CAM::CamVideoPreview(uint16_t x_pos, uint16_t y_pos, uint8_t div, uint8_t on) {
    uint8_t reg;
    wrReg16(0x184, _param.previewCropHStart); wrReg16(0x188, _param.previewCropVStart);
    if(div<1) div=1;
    if(_param.previewCropHSize<640) { _param.previewHRatio=div-1; _param.previewVRatio=div-1;
    } else { _param.previewHRatio=div; _param.previewVRatio=div; }
    wrReg8(0x170, _param.previewHRatio); wrReg8(0x171, _param.previewVRatio);
    if(div>=0x03) div=8; if(div==0x02) div=4; if(div==0x01) div=2; if(div==0x00) div=1;
    wrReg16(0x18C, _param.previewCropHSize/2*div); wrReg16(0x190, _param.previewCropVSize/2*div);
    uint32_t addr=((uint32_t)x_pos+(uint32_t)y_pos*320)>>1;
    wrReg32(0x19C,addr); wrReg32(0x1A0,addr); wrReg32(0x1B0,addr); wrReg32(0x1B4,addr);
    if(on) {wrReg8(0x160, rdReg8(0x160)&~(1<<3)); wrReg8(0x160, rdReg8(0x160)|0x12);}
    else {wrReg8(0x160, rdReg8(0x160)&~0x12);}
}
