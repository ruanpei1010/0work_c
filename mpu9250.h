#ifndef _MPU9250_H__
#define _MPU9250_H__


typedef struct mpu_data
{
	signed short axc_x;
	signed short axc_y;
	signed short axc_z;
	signed short temp;
	signed int   gyro_x;
	signed short gyro_y;
	signed short gyro_z;
	// AK8975
	signed short mag_x;
	signed short mag_y;
	signed short mag_z;
	
}MPU_DATA ;



#define ADDR_MPU9250				0xD0

#define	SMPLRT_DIV					0x19	//陀螺仪采样率,典型值:0x07(125Hz)
#define	CONFIG							0x1A	//低通滤波频率,典型值:0x06(5Hz)
#define	GYRO_CONFIG					0x1B	//陀螺仪自检及测量范围,典型值:0x18(不自检,2000deg/s)
#define	ACCEL_CONFIG	      0x1C	//加速度计自检、测量范围及高通滤波频率，典型值:0x01(不自检,2G,5Hz)
#define	ACCEL_XOUT_H	      0x3B
#define	ACCEL_XOUT_L	      0x3C
#define	ACCEL_YOUT_H	      0x3D
#define	ACCEL_YOUT_L	      0x3E
#define	ACCEL_ZOUT_H	      0x3F
#define	ACCEL_ZOUT_L	      0x40
#define	TEMP_OUT_H					0x41
#define	TEMP_OUT_L					0x42
#define	GYRO_XOUT_H					0x43
#define	GYRO_XOUT_L					0x44
#define	GYRO_YOUT_H					0x45
#define	GYRO_YOUT_L					0x46
#define	GYRO_ZOUT_H					0x47
#define	GYRO_ZOUT_L					0x48
#define	PWR_MGMT_1					0x6B	//电源管理,典型值:0x00(正常启用)
#define	WHO_AM_I	        	0x75	//IIC地址寄存器(默认数值0x71,只读)


uint8 init_mpu9250(void);
unsigned char spi_read_reg(unsigned char reg_add);
void spi_write_reg(unsigned char reg_add,unsigned char data);
void spi_read_data(signed short int data[]);
void gyro_zero(void);
void get_angle(void);


#endif





