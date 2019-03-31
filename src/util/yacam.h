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
#ifndef _YACAM_H_
#define _YACAM_H_

/*********************************************************************
* Class Camera
*********************************************************************/
class  CAM:public INTRFC {
  public:
    uint16_t    CamInit(const uint8_t * camReg);
    uint16_t    CamHwInit(const uint8_t * camReg);
    uint16_t    CamCmd2Byte(uint8_t reg, uint8_t cmd);
    uint16_t    CamCmd3Byte(uint8_t reg1, uint8_t reg2, uint8_t reg3);
    void        CamVideoPreview(uint16_t x0, uint16_t y0, uint8_t div_x, uint8_t on);

};

#endif // _YACAM_H_
