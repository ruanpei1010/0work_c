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

#define	SMPLRT_DIV					0x19	//�����ǲ�����,����ֵ:0x07(125Hz)
#define	CONFIG							0x1A	//��ͨ�˲�Ƶ��,����ֵ:0x06(5Hz)
#define	GYRO_CONFIG					0x1B	//�������Լ켰������Χ,����ֵ:0x18(���Լ�,2000deg/s)
#define	ACCEL_CONFIG	      0x1C	//���ٶȼ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ:0x01(���Լ�,2G,5Hz)
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
#define	PWR_MGMT_1					0x6B	//��Դ����,����ֵ:0x00(��������)
#define	WHO_AM_I	        	0x75	//IIC��ַ�Ĵ���(Ĭ����ֵ0x71,ֻ��)


uint8 init_mpu9250(void);
unsigned char spi_read_reg(unsigned char reg_add);
void spi_write_reg(unsigned char reg_add,unsigned char data);
void spi_read_data(signed short int data[]);
void gyro_zero(void);
void get_angle(void);


#endif





