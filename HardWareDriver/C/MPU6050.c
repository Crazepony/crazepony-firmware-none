/*    
      ____                      _____                  +---+
     / ___\                     / __ \                 | R |
    / /                        / /_/ /                 +---+
   / /   ________  ____  ___  / ____/___  ____  __   __
  / /  / ___/ __ `/_  / / _ \/ /   / __ \/ _  \/ /  / /
 / /__/ /  / /_/ / / /_/  __/ /   / /_/ / / / / /__/ /
 \___/_/   \__,_/ /___/\___/_/    \___ /_/ /_/____  /
                                                 / /
                                            ____/ /
                                           /_____/
mpu6050.c file
编写者：小马  (Camel)
作者E-mail：375836945@qq.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2014-01-28
功能：
1.飞机姿态传感器初始化
2.硬件上的数据中断int脚已连接，软件是轮训机制，大家可以在这方面做修改
------------------------------------
*/
#include "MPU6050.h"
#include "IIC.h"
#include "extern_variable.h"
#include "config.h"


/*

#define MPU6000_ACCEL_DEFAULT_RANGE_G			8
#define MPU6000_ACCEL_DEFAULT_RATE			1000
#define MPU6000_ACCEL_DEFAULT_DRIVER_FILTER_FREQ	30

#define MPU6000_GYRO_DEFAULT_RANGE_G			8
#define MPU6000_GYRO_DEFAULT_RATE			1000
#define MPU6000_GYRO_DEFAULT_DRIVER_FILTER_FREQ		30

#define MPU6000_DEFAULT_ONCHIP_FILTER_FREQ		42

#define MPU6000_ONE_G					9.80665f
*/

uint8_t buffer[14];
int16_t  MPU6050_FIFO[6][11];
int16_t Gx_offset=0,Gy_offset=0,Gz_offset=0;

// uint8_t pgm_read_byte(uint16_t add){
// 	return 0;
// }

/**************************实现函数********************************************
*函数原型:		unsigned char MPU6050_is_DRY(void)
*功　　能:	    检查 MPU6050的中断引脚，测试是否完成转换
返回 1  转换完成
0 数据寄存器还没有更新
*******************************************************************************/
unsigned char MPU6050_is_DRY(void){
    if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5)==Bit_SET){
	  return 1;
	 }
	 else return 0;
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setClockSource(uint8_tsource)
*功　　能:	    设置  MPU6050 的时钟源
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
*******************************************************************************/
void MPU6050_setClockSource(uint8_t source){
    IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);

}

/** Trigger a full device reset.
 * A small delay of ~50ms may be desirable after triggering a reset.
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_DEVICE_RESET_BIT
 */
void MPU6050_reset(void) {
    IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, 1);
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_setFullScaleGyroRange(uint8_t range) {
    IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setFullScaleAccelRange(uint8_trange)
*功　　能:	    设置  MPU6050 加速度计的最大量程
*******************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
    IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setSleepEnabled(uint8_tenabled)
*功　　能:	    设置  MPU6050 是否进入睡眠模式
				enabled =1   睡觉
			    enabled =0   工作
*******************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		uint8_tMPU6050_getDeviceID(void)
*功　　能:	    读取  MPU6050 WHO_AM_I 标识	 将返回 0x68
*******************************************************************************/
uint8_t MPU6050_getDeviceID(void) {

    IICreadBytes(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
    return buffer[0];
}

/**************************实现函数********************************************
*函数原型:		uint8_tMPU6050_testConnection(void)
*功　　能:	    检测MPU6050 是否已经连接
*******************************************************************************/
uint8_t MPU6050_testConnection(void) {
   if(MPU6050_getDeviceID() == 0x68)  //0b01101000;
   return 1;
   else return 0;
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setI2CMasterModeEnabled(uint8_tenabled)
*功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setI2CBypassEnabled(uint8_tenabled)
*功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_Check()
*功　　能:	  检测IIC总线上的MPU6050是否存在
*******************************************************************************/
void MPU6050_Check(void) 
{ 
  switch(MPU6050_testConnection())
  {
    case 0:printf("未检测到MPU6050...\r\n");
      break;
    case 1:printf("已检测到MPU6050...\r\n");
      break;
  }
} 
/**************************实现函数********************************************
*函数原型:		void MPU6050_initialize(void)
*功　　能:	    初始化 	MPU6050 以进入可用状态。
*******************************************************************************/
void MPU6050_initialize(void) {
	//int16_t temp[6];
//	unsigned char i;

/* 
		MPU6050_reset();
    delay_ms(5); // wait after reset 50ms
		MPU6050_setRate(0);  
		MPU6050_setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
		//printf(("设置DLPF带宽为42Hz...\r\n"));
		MPU6050_setDLPFMode(MPU6050_DLPF_BW_42);
		//printf(("设置角速度精度为 +/- 2000 deg/sec...\r\n"));
		MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
		MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_4);	//加速度度最大量程 
	*/

		IICwriteByte(devAddr, MPU6050_RA_PWR_MGMT_1, 0x80);      //PWR_MGMT_1    -- DEVICE_RESET 1
    delay_ms(50);
    IICwriteByte(devAddr, MPU6050_RA_SMPLRT_DIV, 0x00);      //SMPLRT_DIV    -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    IICwriteByte(devAddr, MPU6050_RA_PWR_MGMT_1, 0x03);      //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
    IICwriteByte(devAddr, MPU6050_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);  // INT_PIN_CFG   -- INT_LEVEL_HIGH, INT_OPEN_DIS, LATCH_INT_DIS, INT_RD_CLEAR_DIS, FSYNC_INT_LEVEL_HIGH, FSYNC_INT_DIS, I2C_BYPASS_EN, CLOCK_DIS
    IICwriteByte(devAddr, MPU6050_RA_CONFIG, MPU6050_DLPF_BW_42);  //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
//    IICwriteByte(devAddr, MPU6050_RA_GYRO_CONFIG, 0x18);      //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
		MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    // Accel scale 8g (4096 LSB/g)
    IICwriteByte(devAddr, MPU6050_RA_ACCEL_CONFIG, 2 << 3);


}
//读acc
void MPU6050AccRead(int16_t *accData)
{
    uint8_t buf[6];

    IICreadBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 6, buf);
    accData[0] = (int16_t)((buf[0] << 8) | buf[1]);
    accData[1] = (int16_t)((buf[2] << 8) | buf[3]);
    accData[2] = (int16_t)((buf[4] << 8) | buf[5]);


}
//读gyro
void MPU6050GyroRead(int16_t *gyroData)
{
    uint8_t buf[6];

    IICreadBytes(devAddr, MPU6050_RA_GYRO_XOUT_H, 6, buf);
    gyroData[0] = (int16_t)((buf[0] << 8) | buf[1]) ;
    gyroData[1] = (int16_t)((buf[2] << 8) | buf[3]) ;
    gyroData[2] = (int16_t)((buf[4] << 8) | buf[5]) ;
}


//用于校准 DMP的偏置值
void MPU6050_setAccOffset(int16_t offset[3])
{
		uint8_t buf[2],i=0;
		for(i=0;i<3;i++)
		{
			buf[0]=offset[i]>>8;
			buf[1]=offset[i];
			IICwriteBytes(devAddr, MPU6050_RA_XA_OFFS_H + i*2, 2,buf);
		}	 
}

void MPU6050_setGyroOffset(int16_t offset[3])
{
		uint8_t buf[2],i=0;
		for(i=0;i<3;i++)
		{
			buf[0]=offset[i]>>8;
			buf[1]=offset[i];
			IICwriteBytes(devAddr, MPU6050_RA_XG_OFFS_USRH + i*2, 2,buf);
		}	 
}



// BANK_SEL register
void MPU6050_setMemoryBank(uint8_t bank, uint8_t prefetchEnabled, uint8_t userBank) {
    bank &= 0x1F;
    if (userBank) bank |= 0x20;
    if (prefetchEnabled) bank |= 0x40;
    IICwriteByte(devAddr, MPU6050_RA_BANK_SEL, bank);
}

// MEM_START_ADDR register
void MPU6050_setMemoryStartAddress(uint8_t address) {
    IICwriteByte(devAddr, MPU6050_RA_MEM_START_ADDR, address);
}

// MEM_R_W register
uint8_t MPU6050_readMemoryByte(void) {
    IICreadBytes(devAddr, MPU6050_RA_MEM_R_W, 1 , buffer);
    return buffer[0];
}

// XG_OFFS_USR* registers
int16_t MPU6050_getXGyroOffset(void) {
    IICreadBytes(devAddr, MPU6050_RA_XG_OFFS_USRH, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
int16_t MPU6050_getYGyroOffset(void) {
    IICreadBytes(devAddr, MPU6050_RA_YG_OFFS_USRH, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
int16_t MPU6050_getZGyroOffset(void) {
    IICreadBytes(devAddr, MPU6050_RA_ZG_OFFS_USRH, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

uint8_t verifyBuffer[MPU6050_DMP_MEMORY_CHUNK_SIZE];
uint8_t progBuffer[MPU6050_DMP_MEMORY_CHUNK_SIZE];
uint8_t MPU6050_writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, uint8_t verify, uint8_t useProgMem) {
    uint8_t chunkSize;
    //uint8_t *verifyBuffer;
    uint8_t *tprogBuffer;
    uint16_t i;
    uint8_t j;
	MPU6050_setMemoryBank(bank,0,0);
    MPU6050_setMemoryStartAddress(address);
    for (i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;
        
        // write the chunk of data as specified
        tprogBuffer = (uint8_t*)data + i;
     

        IICwriteBytes(devAddr, MPU6050_RA_MEM_R_W, chunkSize, tprogBuffer);

        // verify data if needed
        if (verify) {
            MPU6050_setMemoryBank(bank,0,0);
            MPU6050_setMemoryStartAddress(address);
            IICreadBytes(devAddr, MPU6050_RA_MEM_R_W, chunkSize, verifyBuffer);

			for(j=0;j<chunkSize;j++){
			if(tprogBuffer[j] != verifyBuffer[j]) 
				return 0; // uh oh.
			}
        }

        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_tautomatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address == 0) bank++;
            MPU6050_setMemoryBank(bank,0,0);
            MPU6050_setMemoryStartAddress(address);
        }
    }
    return 1;
}

uint8_t MPU6050_writeDMPConfigurationSet(const uint8_t *data, uint16_t dataSize, uint8_t useProgMem) {
    uint8_t *progBuffer, success, special;
    uint16_t i;

    // config set data is a long string of blocks with the following structure:
    // [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
    uint8_t bank, offset, length;
    for (i = 0; i < dataSize;) {
            bank = data[i++];
            offset = data[i++];
            length = data[i++];

        // write data or perform special action
        if (length > 0) {
            // regular block of data to write
            progBuffer = (uint8_t*)data + i;
           
            success = MPU6050_writeMemoryBlock(progBuffer, length, bank, offset, 1, 0);
            i += length;
        } else {
            // special instruction
            // NOTE: this kind of behavior (what and when to do certain things)
            // is totally undocumented. This code is in here based on observed
            // behavior only, and exactly why (or even whether) it has to be here
            // is anybody's guess for now.
         
            special = data[i++];
            if (special == 0x01) {
                // enable DMP-related interrupts
                
                IICwriteByte(devAddr, MPU6050_RA_INT_ENABLE, 0x32);  // single operation

                success = 1;
            } else {
                // unknown special command
                success = 0;
            }
        }
        
        if (!success) {
            return 0; // uh oh
        }
    }
    return 1;
}

uint8_t MPU6050_writeProgMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, uint8_t verify) {
	return MPU6050_writeMemoryBlock(data, dataSize, bank, address, verify, 0);
}

uint8_t MPU6050_writeProgDMPConfigurationSet(const uint8_t *data, uint16_t dataSize) {
    return MPU6050_writeDMPConfigurationSet(data, dataSize, 0);
}

/** Set full interrupt enabled status.
 * Full register byte for all interrupts, for quick reading. Each bit should be
 * set 0 for disabled, 1 for enabled.
 * @param enabled New interrupt enabled status
 * @see getIntFreefallEnabled()
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_FF_BIT
 **/
void MPU6050_setIntEnabled(uint8_t enabled) {
    IICwriteByte(devAddr, MPU6050_RA_INT_ENABLE, enabled);
}

/** Set gyroscope sample rate divider.
 * @param rate New sample rate divider
 * @see getRate()
 * @see MPU6050_RA_SMPLRT_DIV
 */
void MPU6050_setRate(uint8_t rate) {
    IICwriteByte(devAddr, MPU6050_RA_SMPLRT_DIV, rate);
}

/** Set digital low-pass filter configuration.
 * @param mode New DLFP configuration setting
 * @see getDLPFBandwidth()
 * @see MPU6050_DLPF_BW_256
 * @see MPU6050_RA_CONFIG
 * @see MPU6050_CFG_DLPF_CFG_BIT
 * @see MPU6050_CFG_DLPF_CFG_LENGTH
 */
void MPU6050_setDLPFMode(uint8_t mode) {
    IICwriteBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

/** Set external FSYNC configuration.
 * @see getExternalFrameSync()
 * @see MPU6050_RA_CONFIG
 * @param sync New FSYNC configuration value
 */
void MPU6050_setExternalFrameSync(uint8_t sync) {
    IICwriteBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT, MPU6050_CFG_EXT_SYNC_SET_LENGTH, sync);
}

void MPU6050_setDMPConfig1(uint8_t config) {
    IICwriteByte(devAddr, MPU6050_RA_DMP_CFG_1, config);
}

void MPU6050_setDMPConfig2(uint8_t config) {
    IICwriteByte(devAddr, MPU6050_RA_DMP_CFG_2, config);
}

void MPU6050_setOTPBankValid(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, enabled);
}


void MPU6050_setXGyroOffset(int16_t offset) {
	buffer[0] = offset>>8;
	buffer[1] = offset&0x00ff;
    IICwriteBytes(devAddr, MPU6050_RA_XG_OFFS_USRH, 2, buffer);
}
void MPU6050_setYGyroOffset(int16_t offset) {
	buffer[0] = offset>>8;
	buffer[1] = offset&0x00ff;
    IICwriteBytes(devAddr, MPU6050_RA_YG_OFFS_USRH, 2, buffer);
}

void MPU6050_setZGyroOffset(int16_t offset) {
    buffer[0] = offset>>8;
	buffer[1] = offset&0x00ff;
    IICwriteBytes(devAddr, MPU6050_RA_ZG_OFFS_USRH, 2, buffer);
}

/** Reset the FIFO.
 * This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0. This
 * bit automatically clears to 0 after the reset has been triggered.
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_RESET_BIT
 */
void MPU6050_resetFIFO(void) {
    IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, 1);
}

/** Get current FIFO buffer size.
 * This value indicates the number of bytes stored in the FIFO buffer. This
 * number is in turn the number of bytes that can be read from the FIFO buffer
 * and it is directly proportional to the number of samples available given the
 * set of sensor data bound to be stored in the FIFO (register 35 and 36).
 * @return Current FIFO buffer size
 */
uint16_t MPU6050_getFIFOCount(void) {
    IICreadBytes(devAddr, MPU6050_RA_FIFO_COUNTH, 2, buffer);
    return (((uint16_t)buffer[0]) << 8) | buffer[1];
}

/** Set free-fall event acceleration threshold.
 * @param threshold New motion detection acceleration threshold value (LSB = 2mg)
 * @see getMotionDetectionThreshold()
 * @see MPU6050_RA_MOT_THR
 */
void MPU6050_setMotionDetectionThreshold(uint8_t threshold) {
    IICwriteByte(devAddr, MPU6050_RA_MOT_THR, threshold);
}

/** Set zero motion detection event acceleration threshold.
 * @param threshold New zero motion detection acceleration threshold value (LSB = 2mg)
 * @see getZeroMotionDetectionThreshold()
 * @see MPU6050_RA_ZRMOT_THR
 */
void MPU6050_setZeroMotionDetectionThreshold(uint8_t threshold) {
    IICwriteByte(devAddr, MPU6050_RA_ZRMOT_THR, threshold);
}

/** Set motion detection event duration threshold.
 * @param duration New motion detection duration threshold value (LSB = 1ms)
 * @see getMotionDetectionDuration()
 * @see MPU6050_RA_MOT_DUR
 */
void MPU6050_setMotionDetectionDuration(uint8_t duration) {
    IICwriteByte(devAddr, MPU6050_RA_MOT_DUR, duration);
}

/** Set zero motion detection event duration threshold.
 * @param duration New zero motion detection duration threshold value (LSB = 1ms)
 * @see getZeroMotionDetectionDuration()
 * @see MPU6050_RA_ZRMOT_DUR
 */
void MPU6050_setZeroMotionDetectionDuration(uint8_t duration) {
    IICwriteByte(devAddr, MPU6050_RA_ZRMOT_DUR, duration);
}

void MPU6050_readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address) {
    uint8_t chunkSize;
	uint16_t i;
	MPU6050_setMemoryBank(bank,0,0);
    MPU6050_setMemoryStartAddress(address);
    
    for (i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;

        // read the chunk of data as specified
        IICwriteBytes(devAddr, MPU6050_RA_MEM_R_W, chunkSize, data + i);
        
        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_tautomatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address == 0) bank++;
            MPU6050_setMemoryBank(bank,0,0);
            MPU6050_setMemoryStartAddress(address);
        }
    }
}

void MPU6050_getFIFOBytes(uint8_t *data, uint8_t length) {
    IICreadBytes(devAddr, MPU6050_RA_FIFO_R_W, length, data);
}

/** Get full set of interrupt status bits.
 * These bits clear to 0 after the register has been read. Very useful
 * for getting multiple INT statuses, since each single bit read clears
 * all of them because it has to read the whole byte.
 * @return Current interrupt status
 * @see MPU6050_RA_INT_STATUS
 */
uint8_t MPU6050_getIntStatus(void) {
    return I2C_ReadOneByte(devAddr, MPU6050_RA_INT_STATUS);
}

void MPU6050_setDMPEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, enabled);
}

uint8_t MPU6050_getOTPBankValid(void) {
	uint8_t  temp = I2C_ReadOneByte(devAddr, MPU6050_RA_XG_OFFS_TC);
    return temp&(1<<MPU6050_TC_OTP_BNK_VLD_BIT);
}

int8_t MPU6050_getXGyroOffsetTC(void) {
	uint8_t  temp = I2C_ReadOneByte(devAddr, MPU6050_RA_XG_OFFS_TC);
	temp &= 0x3F;
    return temp;
}
void MPU6050_setXGyroOffsetTC(int8_t offset) {
    IICwriteBits(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}


// YG_OFFS_TC register
int8_t MPU6050_getYGyroOffsetTC(void) {
	uint8_t  temp = I2C_ReadOneByte(devAddr, MPU6050_RA_YG_OFFS_TC);
	temp &= 0x3F;
    return temp;
}
void MPU6050_setYGyroOffsetTC(int8_t offset) {
    IICwriteBits(devAddr, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

// ZG_OFFS_TC register
int8_t MPU6050_getZGyroOffsetTC(void) {
	uint8_t  temp = I2C_ReadOneByte(devAddr, MPU6050_RA_ZG_OFFS_TC);
	temp &= 0x3F;
    return temp;
}
void MPU6050_setZGyroOffsetTC(int8_t offset) {
    IICwriteBits(devAddr, MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

/** Set the I2C address of the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param address New address for specified slave
 * @see getSlaveAddress()
 * @see MPU6050_RA_I2C_SLV0_ADDR
 */
void MPU6050_setSlaveAddress(uint8_t num, uint8_t address) {
    if (num > 3) return;
    IICwriteByte(devAddr, MPU6050_RA_I2C_SLV0_ADDR + num*3, address);
}

/** Reset the I2C Master.
 * This bit resets the I2C Master when set to 1 while I2C_MST_EN equals 0.
 * This bit automatically clears to 0 after the reset has been triggered.
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_I2C_MST_RESET_BIT
 */
void MPU6050_resetI2CMaster(void) {
    IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_RESET_BIT, 1);
}

/** Set FIFO enabled status.
 * @param enabled New FIFO enabled status
 * @see getFIFOEnabled()
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_EN_BIT
 */
void MPU6050_setFIFOEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, enabled);
}

void MPU6050_resetDMP(void) {
    IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_RESET_BIT, 1);
}




//------------------End of File----------------------------
