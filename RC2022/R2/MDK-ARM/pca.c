#include "pca.h"
//#include "i2c.h"
//#include "delay.h"
#include "math.h"
#include "iic.h"
void pca_write(u8 adrr,u8 data)//��PCAд����,adrrd��ַ,data����
{ 
	  I2C_Start();
	
		I2C_SendByte(pca_adrr);
		I2C_WaitAck();
	
		I2C_SendByte(adrr);
	  I2C_WaitAck();
			
	  I2C_Stop();
}

u8 pca_read(u8 adrr)//��PCA������
{
	u8 data;
	I2C_Start();
	
	I2C_SendByte(pca_adrr);
	I2C_WaitAck();
	
	I2C_SendByte(adrr);
	I2C_WaitAck();
	
	I2C_Start();
	
	I2C_SendByte(pca_adrr|0x01);
	I2C_WaitAck();
	
	data=I2C_ReadByte(0);
	I2C_Stop();
	
	return data;
}


void pca_setfreq(float freq)//����PWMƵ��
{
		u8 prescale,oldmode,newmode;
		double prescaleval;
		freq *= 0.92; 
		prescaleval = 25000000; //25 000 000 ???Ϊɶ��25Mhz
		prescaleval /= 4096;
		prescaleval /= freq;
		prescaleval -= 1;
		prescale =floor(prescaleval + 0.5f);

		oldmode = pca_read(pca_mode1);
	
		newmode = (oldmode&0x7F) | 0x10; // sleep
	
		pca_write(pca_mode1, newmode); // go to sleep
	
		pca_write(pca_pre, prescale); // set the prescaler
	
		pca_write(pca_mode1, oldmode);
		//delay_ms(2);
	  osDelay(2);
		pca_write(pca_mode1, oldmode | 0xa1); 
}

void pca_setpwm(u8 num, u32 on, u32 off)
{
		off=(u32)(102+off*2.28);
		pca_write(LED0_ON_L+4*num,on);
		pca_write(LED0_ON_H+4*num,on>>8);
		pca_write(LED0_OFF_L+4*num,off);
		pca_write(LED0_OFF_H+4*num,off>>8);
}

void pca_down(u8 num)
{
		pca_write(LED0_ON_L+4*num,0);
		pca_write(LED0_ON_H+4*num,0>>8);
		pca_write(LED0_OFF_L+4*num,0);
		pca_write(LED0_OFF_H+4*num,0>>8);
}
/*num:���PWM�������0~15��on:PWM��������ֵ0~4096,off:PWM�½�����ֵ0~4096
һ��PWM���ڷֳ�4096�ݣ���0��ʼ+1�������Ƶ�onʱ����Ϊ�ߵ�ƽ������������offʱ
����Ϊ�͵�ƽ��ֱ������4096���¿�ʼ�����Ե�on������0ʱ������ʱ,��on����0ʱ��
off/4096��ֵ����PWM��ռ�ձȡ�*/

/*
	�������ã���ʼ�����������
	������1.PWMƵ��
		  2.��ʼ������Ƕ�
*/
void PCA_MG9XX_Init(float hz,u8 angle)
{
	
	
	//IIC_Init(); ��ʼ��IIC
	//MX_I2C1_Init();
	I2C_InitGPIO();
	pca_write(pca_mode1,0x0);
	pca_setfreq(hz);//����PWMƵ�� 
	//��ʼ�����  16·���ȫΪ0�� 
	pca_down(0);pca_down(1);pca_down(2);pca_down(3);pca_down(4);pca_down(5);pca_down(6);pca_down(7);
	pca_down(8);pca_down(9);pca_down(10);pca_down(11);pca_down(12);pca_down(13);pca_down(14);pca_down(15);
//	pca_setpwm(0,0,off);pca_setpwm(1,0,off);pca_setpwm(2,0,off);pca_setpwm(3,0,off);
//	pca_setpwm(4,0,off);pca_setpwm(5,0,off);pca_setpwm(6,0,off);pca_setpwm(7,0,off);
//	pca_setpwm(8,0,off);pca_setpwm(9,0,off);pca_setpwm(10,0,off);pca_setpwm(11,0,off);
//	pca_setpwm(12,0,off);pca_setpwm(13,0,off);pca_setpwm(14,0,off);pca_setpwm(15,0,off);
	
	
	//delay_ms(300);
	osDelay(300);
}
