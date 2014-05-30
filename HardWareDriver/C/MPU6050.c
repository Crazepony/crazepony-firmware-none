
 /*    
  *      ____                      _____                  +---+
  *     / ___\                     / __ \                 | R |
  *    / /                        / /_/ /                 +---+
  *   / /   ________  ____  ___  / ____/___  ____  __   __
  *  / /  / ___/ __ `/_  / / _ \/ /   / __ \/ _  \/ /  / /
  * / /__/ /  / /_/ / / /_/  __/ /   / /_/ / / / / /__/ /
  * \___/_/   \__,_/ /___/\___/_/    \___ /_/ /_/____  /
  *                                                 / /
  *                                            ____/ /
  *                                           /_____/
  *                                       
  *  Crazyfile control firmware                                        
  *  Copyright (C) 2011-2014 Crazepony-II                                        
  *
  *  This program is free software: you can redistribute it and/or modify
  *  it under the terms of the GNU General Public License as published by
  *  the Free Software Foundation, in version 3.
  *
  *  This program is distributed in the hope that it will be useful,
  *  but WITHOUT ANY WARRANTY; without even the implied warranty of
  *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  *  GNU General Public License for more details.
  * 
  * You should have received a copy of the GNU General Public License
  * along with this program. If not, see <http://www.gnu.org/licenses/>.
  *
  *
  * debug.c - Debugging utility functions
  *
  */
#include "MPU6050.h"
#include "IIC.h"
#include "extern_variable.h"


//MPU6050驱动程序，最关键的是IIC接口要可以用（上拉电阻很重要）
//程序配置参考数据手册
//最后修改：2014-01-29

uint8_t 	mpu6050_buffer[14];					
S_INT16_XYZ 	GYRO_OFFSET,ACC_OFFSET;			
uint8_t GYRO_OFFSET_OK = 1;
uint8_t ACC_OFFSET_OK = 1;
S_INT16_XYZ 	MPU6050_ACC_LAST,MPU6050_GYRO_LAST;		


//内部延时
static void MPU6050_Delay(unsigned long time)
{
   long i;
   for(i=0; i<time; i++)
   {
     
   }
}


//将iic读取到得数据分拆,放入相应数组
void MPU6050_Dataanl(void)
{
    MPU6050_ACC_LAST.X=((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) - ACC_OFFSET.X;
    MPU6050_ACC_LAST.Y=((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]) - ACC_OFFSET.Y;
    MPU6050_ACC_LAST.Z=((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]) - ACC_OFFSET.Z;
    //这里跳过温度ADC，详细见参考手册
    MPU6050_GYRO_LAST.X=((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]) - GYRO_OFFSET.X;
    MPU6050_GYRO_LAST.Y=((((int16_t)mpu6050_buffer[10]) << 8) | mpu6050_buffer[11]) - GYRO_OFFSET.Y;
    MPU6050_GYRO_LAST.Z=((((int16_t)mpu6050_buffer[12]) << 8) | mpu6050_buffer[13]) - GYRO_OFFSET.Z;

    if(!GYRO_OFFSET_OK)
    {
        static int32_t	tempgx=0,tempgy=0,tempgz=0;
        static uint8_t cnt_g=0;

        if(cnt_g==0)
        {
            GYRO_OFFSET.X=0;
            GYRO_OFFSET.Y=0;
            GYRO_OFFSET.Z=0;
            tempgx = 0;
            tempgy = 0;
            tempgz = 0;
            cnt_g = 1;
            
            return;
        }
        tempgx+= MPU6050_GYRO_LAST.X;
        tempgy+= MPU6050_GYRO_LAST.Y;
        tempgz+= MPU6050_GYRO_LAST.Z;
        if(cnt_g==200)
        {
            GYRO_OFFSET.X=tempgx/cnt_g;
            GYRO_OFFSET.Y=tempgy/cnt_g;
            GYRO_OFFSET.Z=tempgz/cnt_g;
            cnt_g = 0;
            GYRO_OFFSET_OK = 1;
            
            return;
        }
        
        cnt_g++;
    }
    if(!ACC_OFFSET_OK)
    {
        static int32_t	tempax=0,tempay=0,tempaz=0;
        static uint8_t cnt_a=0;

        if(cnt_a==0)
        {
            ACC_OFFSET.X = 0;
            ACC_OFFSET.Y = 0;
            ACC_OFFSET.Z = 0;
            tempax = 0;
            tempay = 0;
            tempaz = 0;
            cnt_a = 1;
            
            return;
        }
        tempax+= MPU6050_ACC_LAST.X;
        tempay+= MPU6050_ACC_LAST.Y;
        tempaz+= MPU6050_ACC_LAST.Z;
        if(cnt_a==200)
        {
            ACC_OFFSET.X=tempax/cnt_a;
            ACC_OFFSET.Y=tempay/cnt_a;
            ACC_OFFSET.Z=tempaz/cnt_a;
            cnt_a = 0;
            ACC_OFFSET_OK = 1;
            
            return;
        }
        cnt_a++;		
    }
}


//更新最新的MPU数据
void MPU6050_READ(void)
{
    i2cRead(devAddr,MPU6050_RA_ACCEL_XOUT_H,14,mpu6050_buffer);
}


//修改写指定设备指定寄存器一个字节中的1个位
void IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
{
    u8 b;
    
    i2cRead(dev, reg, 1, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    i2cWrite(dev, reg, b);
}


//修改写指定设备 指定寄存器一个字节中的多个位
void IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{
    u8 mask;
    u8 b;
    
    i2cRead(dev, reg, 1, &b);
    mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
    data <<= (8 - length);
    data >>= (7 - bitStart);
    b &= mask;
    b |= data;
    i2cWrite(dev, reg, b);
}


//设置MPU6050的时钟源
/**********************************************************************
* 0       | Internal oscillator
* 1       | PLL with X Gyro reference
* 2       | PLL with Y Gyro reference
* 3       | PLL with Z Gyro reference
* 4       | PLL with external 32.768kHz reference
* 5       | PLL with external 19.2MHz reference
* 6       | Reserved
* 7       | Stops the clock and keeps the timing generator in reset
********************************************************************/
void MPU6050_setClockSource(uint8_t source)
{
    IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}


//设置 MPU6050 陀螺仪的最大量程
void MPU6050_setFullScaleGyroRange(uint8_t range)
{
    IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}


//设置 MPU6050 加速度计的最大量程
void MPU6050_setFullScaleAccelRange(uint8_t range) 
{
    IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

//设置 MPU6050 是否进入睡眠模式, enabled =1睡觉,enabled =0   工作
void MPU6050_setSleepEnabled(uint8_t enabled) 
{
    IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

//设置 MPU6050 是否为AUX I2C线的主机
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) 
{
    IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}



//设置MPU6050是否控制从IIC设备
void MPU6050_setI2CBypassEnabled(uint8_t enabled) 
{
    IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}


void MPU6050_setDLPF(uint8_t mode)
{
    IICwriteBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}



//初始化 MPU6050
void MPU6050_INIT(void)
{
    MPU6050_Delay(2000);   //延时很重要
    MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO); //设置时钟  0x6b   0x01
    MPU6050_Delay(2000);
    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_500);//陀螺仪最大量程 +-500度每秒
    MPU6050_Delay(2000);
    MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_4);	//加速度度最大量程 +-4G
    MPU6050_Delay(2000);
    MPU6050_setDLPF(MPU6050_DLPF_BW_42);
    MPU6050_Delay(2000);
    MPU6050_setSleepEnabled(0); //进入工作状态
    MPU6050_Delay(2000);
    MPU6050_setI2CMasterModeEnabled(0);	 //不让MPU6050 控制AUXI2C
    MPU6050_Delay(2000);
    MPU6050_setI2CBypassEnabled(1);  //主控制器的I2C与MPU6050的AUXI2C直通。控制器可以直接访问HMC5883L
    MPU6050_Delay(2000);
}

//初始化 MPU6050
//mpu6050初始化顺序：
//1.初始化IIC总线
//2.调用该函数MPU6050_Check()检测设备存在与否
//3.若设备存在，则初始化该设备，否则不初始化
char MPU6050_Check(void)
{
      uint8_t buf1[5]={0xaa,0xaa,0xaa,0xaa,0xaa};
      uint8_t buf2[5];
      char i;
      for(i=0;i<5;i++)
      IICwriteBits(devAddr, 0x00+i, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, buf1[i]);
      for(i=0;i<5;i++)
      i2cRead(devAddr,0x00+i,MPU6050_CFG_DLPF_CFG_LENGTH,buf2);//往0x00这个寄存器写入5个数，再从这个寄存器读出5个数
      
       /*比较*/ 
       for (i=0;i<5;i++) 
       { 
          if (buf1[i]!=0xaa) //如果读出的数为事先设定的数，则说明外部连接了设备，否则没有连接成功
          break; 
       } 
      
       if (i==5)   return 1 ;        //MCU 与MPU6050 成功连接 
       else        return 0 ;        //MCU与MPU6050不正常连接  
}

