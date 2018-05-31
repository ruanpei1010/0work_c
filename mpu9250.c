#include "stm32f10x.h"
#include "include.h"


sint16 mpu_buff[7];
sint16 gyro_zero_xyz[3];
//_iq15 gyro_xyz_q15[3];    //弧度 Q15

sint16 watch[3];

uint8 init_mpu9250(void)
{
  
	uint8 dat;
  uint8 re;
  
  /*
   *寄存器
   */
	spi_write_reg(PWR_MGMT_1, 0x00);	//解除休眠状态
	spi_write_reg(SMPLRT_DIV, 0x07);
	spi_write_reg(CONFIG, 0x06);
	spi_write_reg(GYRO_CONFIG, 0x18);		// 正负 2000deg/s
	spi_write_reg(ACCEL_CONFIG, 0x01);
	delay_ms(100) ;
	dat = 0x80 ;
	spi_write_reg(PWR_MGMT_1,dat);
	delay_ms(50);
	dat = 0x00 ;
	spi_write_reg(PWR_MGMT_1,dat);
  delay_ms(50);
	spi_write_reg(PWR_MGMT_1,dat) ;
	dat = 0x01 ; 
	spi_write_reg(SMPLRT_DIV,dat) ;
	dat = 0x86 ;
	spi_write_reg(CONFIG,dat) ;
	dat = 0x10 ;								// 陀螺仪量程 ±1000dps
	spi_write_reg(GYRO_CONFIG,dat) ;
	dat = 0x10 ;								// 加速度计量程 ±8g
	spi_write_reg(ACCEL_CONFIG,dat) ;
	
	dat = 0x00;
	spi_write_reg(0x38,dat);

  /*
   */
   do{
     delay_ms(30);
    re =spi_read_reg(WHO_AM_I);
     
   }while(re != 0x71);
  
  return 0;
}



/**************************************
SPI 接收数据
参数:	
		reg 寄存器地址,
		dat 数据
**************************************/
unsigned char spi_read_reg(unsigned char reg_add)
{
	unsigned char dat;
	
  MPU_CS_L;
	reg_add |= 0x80 ;
	spi_rw(reg_add);
	dat = spi_rw(0xff);
	MPU_CS_H;
	
  return dat ;
}

/**************************************
SPI 发送数据
参数:	reg 寄存器地址,
			dat 数据
**************************************/
void spi_write_reg(unsigned char reg_add,unsigned char data)
{
	MPU_CS_L;
	
  reg_add &= 0x7f;
	spi_rw(reg_add);
	spi_rw(data);
	
	MPU_CS_H ;
}

/**************************************
SPI 读所有数据
********************/
void spi_read_data(signed short int data[])
{
  unsigned char add = 0x3B | 0x80;
  unsigned char buff[14];
	unsigned char i;
	
	MPU_CS_L ;
  
	spi_rw(add);
	
  for(i=0;i<14;i++)
  {
    buff[i] =spi_rw(0xff);
  }
	MPU_CS_H ;
	
  for( i=0;i<7;i++)
  {
    data[i] =((short signed int)buff[i*2] << 8 ) | (( short signed int)buff[i*2+1]);
  }

}

void gyro_zero(void)
{
  for(int t=0; t<6 ;t++)
  {
    sint16 add[3]={0};
   
    sint16 max=-2000, min=2000;
    for(int i=0;i<128;i++)
    {
			spi_read_data(mpu_buff);
      
      for(int j=0;j<3;j++)
      {
        add[j] +=mpu_buff[j+4];
        if(mpu_buff[j+4] > max)
        {
          max =mpu_buff[j+4];
        }
        if(mpu_buff[j+4] < min)
        {
          min=mpu_buff[j+4];
        }
      }
      delay_ms(5);
    }
    if(max - min <200)
    {
      gyro_zero_xyz[0] =add[0] >> 7;
      gyro_zero_xyz[1] =add[1] >> 7;
      gyro_zero_xyz[2] =add[2] >> 7;
      break;
    }
  }
}

/************************************************************
 * 读陀螺仪
 * angleNow ：1ms转动的角度
 ************************************************************/
//void get_angle(void)
//{
//  //16位 ±1000dps
//  sint16 gryo_temp[3];
//  spi_read_data(mpu_buff);

//  for(int i=0;i<3;i++)
//  {
//    gryo_temp[i] =mpu_buff[4+i]-gyro_zero_xyz[i];
//    
//    gryo_temp[i] = _IQ15mpy(gryo_temp[i] ,572);
//    gyro_xyz_q15[i] +=gryo_temp[i];
//    
//    if(gyro_xyz_q15[i] > _IQ15(PI))
//    {
//      gyro_xyz_q15[i] -=_IQ15(PI2);
//    }
//    if(gyro_xyz_q15[i] < _IQ15(-PI))
//    {
//      gyro_xyz_q15[i] +=_IQ15(PI2);
//    }
//  }
//}






