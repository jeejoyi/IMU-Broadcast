////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib-Arduino
//
//  Copyright (c) 2014, richards-tech
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all 
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "RTIMUMPU9150.h"
#include "RTIMUSettings.h"

//  this sets the learning rate for compass running average calculation

#define COMPASS_ALPHA 0.2f

RTIMUMPU9150::RTIMUMPU9150(RTIMUSettings *settings) : RTIMU(settings)
{

}

RTIMUMPU9150::~RTIMUMPU9150()
{
}

bool RTIMUMPU9150::setLpf(unsigned char lpf)
{
    switch (lpf) {
    case MPU9150_LPF_256:
    case MPU9150_LPF_188:
    case MPU9150_LPF_98:
    case MPU9150_LPF_42:
    case MPU9150_LPF_20:
    case MPU9150_LPF_10:
    case MPU9150_LPF_5:
        m_lpf = lpf;
        return true;

    default:
        return false;
    }
}


bool RTIMUMPU9150::setSampleRate(int rate)
{
    if ((rate < MPU9150_SAMPLERATE_MIN) || (rate > MPU9150_SAMPLERATE_MAX)) {
        return false;
    }
    m_sampleRate = rate;
    m_sampleInterval = (unsigned long)1000 / m_sampleRate;
	if (m_sampleInterval == 0)
		m_sampleInterval = 1;
    return true;
}

bool RTIMUMPU9150::setCompassRate(int rate)
{
    if ((rate < MPU9150_COMPASSRATE_MIN) || (rate > MPU9150_COMPASSRATE_MAX)) {
        return false;
    }
    m_compassRate = rate;
    return true;
}

bool RTIMUMPU9150::setGyroFsr(unsigned char fsr)
{
    switch (fsr) {
    case MPU9150_GYROFSR_250:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (131.0 * 180.0);
        return true;

    case MPU9150_GYROFSR_500:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (62.5 * 180.0);
        return true;

    case MPU9150_GYROFSR_1000:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (32.8 * 180.0);
        return true;

    case MPU9150_GYROFSR_2000:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (16.4 * 180.0);
        return true;

    default:
        return false;
    }
}

bool RTIMUMPU9150::setAccelFsr(unsigned char fsr)
{
    switch (fsr) {
    case MPU9150_ACCELFSR_2:
        m_accelFsr = fsr;
        m_accelScale = 1.0/16384.0;
        return true;

    case MPU9150_ACCELFSR_4:
        m_accelFsr = fsr;
        m_accelScale = 1.0/8192.0;
        return true;

    case MPU9150_ACCELFSR_8:
        m_accelFsr = fsr;
        m_accelScale = 1.0/4096.0;
        return true;

    case MPU9150_ACCELFSR_16:
        m_accelFsr = fsr;
        m_accelScale = 1.0/2048.0;
        return true;

    default:
        return false;
    }
}


int RTIMUMPU9150::IMUInit()
{
    unsigned char result;
    int loop;
    unsigned char asa[3];

    m_firstTime = true;

#ifdef MPU9150_CACHE_MODE
    m_cacheIn = m_cacheOut = m_cacheCount = 0;
#endif
    //  configure IMU

    m_slaveAddr = m_settings->m_I2CSlaveAddress;
    setSampleRate(m_settings->m_MPU9150GyroAccelSampleRate);
    setCompassRate(m_settings->m_MPU9150CompassSampleRate);
    setLpf(m_settings->m_MPU9150GyroAccelLpf);
    setGyroFsr(m_settings->m_MPU9150GyroFsr);
    setAccelFsr(m_settings->m_MPU9150AccelFsr);

    setCalibrationData();

    //  reset the MPU9150

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9150_PWR_MGMT_1, 0x80))
        return -1;

	delay(100);

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9150_PWR_MGMT_1, 0x00))
        return -4;

    if (!I2Cdev::readByte(m_slaveAddr, MPU9150_WHO_AM_I, &result))
        return -5;

    if (result != 0x68) {
         return -6;
    }

    //  now configure the various components

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9150_LPF_CONFIG, m_lpf))
        return -7;

    if (!setSampleRate())
        return -8;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9150_GYRO_CONFIG, m_gyroFsr))
        return -9;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9150_ACCEL_CONFIG, m_accelFsr))
         return -10;

    //  now configure compass

    if (!bypassOn())
		return -11;

    // get fuse ROM data

    if (!I2Cdev::writeByte(AK8975_ADDRESS, AK8975_CNTL, 0)) {
        bypassOff();
        return -12;
    }

    if (!I2Cdev::writeByte(AK8975_ADDRESS, AK8975_CNTL, 0x0f)) {
        bypassOff();
        return -13;
    }

    if (!I2Cdev::readBytes(AK8975_ADDRESS, AK8975_ASAX, 3, asa)) {
        bypassOff();
        return -14;
    }

    //  convert asa to usable scale factor

    m_compassAdjust[0] = ((float)asa[0] - 128.0) / 256.0 + 1.0f;
    m_compassAdjust[1] = ((float)asa[1] - 128.0) / 256.0 + 1.0f;
    m_compassAdjust[2] = ((float)asa[2] - 128.0) / 256.0 + 1.0f;

    if (!I2Cdev::writeByte(AK8975_ADDRESS, AK8975_CNTL, 0)) {
        bypassOff();
        return -15;
    }

    if (!bypassOff())
		return -16;

    //  now set up MPU9150 to talk to the compass chip

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9150_I2C_MST_CTRL, 0x40))
        return -17;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9150_I2C_SLV0_ADDR, 0x80 | AK8975_ADDRESS))
        return -18;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9150_I2C_SLV0_REG, AK8975_ST1))
        return -19;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9150_I2C_SLV0_CTRL, 0x88))
        return -20;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9150_I2C_SLV1_ADDR, AK8975_ADDRESS))
        return -21;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9150_I2C_SLV1_REG, AK8975_CNTL))
        return -22;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9150_I2C_SLV1_CTRL, 0x81))
        return -23;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9150_I2C_SLV1_DO, 0x1))
        return -24;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9150_I2C_MST_DELAY_CTRL, 0x3))
        return -25;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9150_YG_OFFS_TC, 0x80))
        return -26;

    if (!setCompassRate())
        return -27;

    //  enable the sensors

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9150_PWR_MGMT_1, 1))
        return -28;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9150_PWR_MGMT_2, 0))
         return -29;

    //  select the data to go into the FIFO and enable

    if (!resetFifo())
        return -30;

    m_gyroAlpha = 1.0f / m_sampleRate;
    m_gyroStartTime = millis();
    m_gyroLearning = true;

    return 1;
}

bool RTIMUMPU9150::resetFifo()
{
    if (!I2Cdev::writeByte(m_slaveAddr, MPU9150_INT_ENABLE, 0))
        return false;
    if (!I2Cdev::writeByte(m_slaveAddr, MPU9150_FIFO_EN, 0))
        return false;
    if (!I2Cdev::writeByte(m_slaveAddr, MPU9150_USER_CTRL, 0))
        return false;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9150_USER_CTRL, 0x04))
        return false;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9150_USER_CTRL, 0x60))
        return false;

    delay(50);

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9150_INT_ENABLE, 1))
        return false;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9150_FIFO_EN, 0x78))
        return false;

    return true;
}

bool RTIMUMPU9150::bypassOn()
{
    unsigned char userControl;

    if (!I2Cdev::readByte(m_slaveAddr, MPU9150_USER_CTRL, &userControl))
        return false;

    userControl &= ~0x20;
	userControl |= 2;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9150_USER_CTRL, userControl))
        return false;

    delay(50);

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9150_INT_PIN_CFG, 0x82))
        return false;

    delay(50);
    return true;
}


bool RTIMUMPU9150::bypassOff()
{
    unsigned char userControl;

    if (!I2Cdev::readByte(m_slaveAddr, MPU9150_USER_CTRL, &userControl))
        return false;

    userControl |= 0x20;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9150_USER_CTRL, userControl))
        return false;

    delay(50);

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9150_INT_PIN_CFG, 0x80))
         return false;

    delay(50);
    return true;
}

bool RTIMUMPU9150::setSampleRate()
{
    int clockRate = 1000;

    if (m_lpf == MPU9150_LPF_256)
        clockRate = 8000;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9150_SMPRT_DIV, (unsigned char)(clockRate / m_sampleRate - 1)))
        return false;

    return true;
}

bool RTIMUMPU9150::setCompassRate()
{
    int rate;

    rate = m_sampleRate / m_compassRate - 1;

    if (rate > 31)
        rate = 31;
    if (!I2Cdev::writeByte(m_slaveAddr, MPU9150_I2C_SLV4_CTRL, rate))
         return false;
    return true;
}

int RTIMUMPU9150::IMUGetPollInterval()
{
    return (400 / m_sampleRate);
}

bool RTIMUMPU9150::IMURead(long unsigned int time)
{
    unsigned char fifoCount[2];
    unsigned int count;
    unsigned char fifoData[12];
    unsigned char compassData[8];

    if (!I2Cdev::readBytes(m_slaveAddr, MPU9150_FIFO_COUNT_H, 2, fifoCount))
         return false;

    count = ((unsigned int)fifoCount[0] << 8) + fifoCount[1];

    if (count == 1024) {
        resetFifo();
        m_timestamp += m_sampleInterval * (1024 / MPU9150_FIFO_CHUNK_SIZE + 1); // try to fix timestamp
        return false;
    }

    if (count > MPU9150_FIFO_CHUNK_SIZE * 40) {
        // more than 40 samples behind - going too slowly so discard some samples but maintain timestamp correctly
        while (count >= MPU9150_FIFO_CHUNK_SIZE * 10) {
            if (!I2Cdev::readBytes(m_slaveAddr, MPU9150_FIFO_R_W, MPU9150_FIFO_CHUNK_SIZE, fifoData))
                return false;
            count -= MPU9150_FIFO_CHUNK_SIZE;
            m_timestamp += m_sampleInterval;
        }
    }

    if (count < MPU9150_FIFO_CHUNK_SIZE)
        return false;

    if (!I2Cdev::readBytes(m_slaveAddr, MPU9150_FIFO_R_W, MPU9150_FIFO_CHUNK_SIZE, fifoData))
        return false;

    if (!I2Cdev::readBytes(m_slaveAddr, MPU9150_EXT_SENS_DATA_00, 8, compassData))
        return false;

    RTMath::convertToVector(fifoData, m_accel, m_accelScale, true);
    RTMath::convertToVector(fifoData + 6, m_gyro, m_gyroScale, true);
    RTMath::convertToVector(compassData + 1, m_compass, 0.3f, false);

    if (m_gyroLearning) {
        // update gyro bias

        m_gyroBias.setX((1.0 - m_gyroAlpha) * m_gyroBias.x() + m_gyroAlpha * m_gyro.x());
        m_gyroBias.setY((1.0 - m_gyroAlpha) * m_gyroBias.y() + m_gyroAlpha * m_gyro.y());
        m_gyroBias.setZ((1.0 - m_gyroAlpha) * m_gyroBias.z() + m_gyroAlpha * m_gyro.z());

        if ((millis() - m_gyroStartTime) > 60000)
            m_gyroLearning = false;                     // only do this for 5 seconds
    }

    //  sort out gyro axes and correct for bias

    m_gyro.setX(m_gyro.x() - m_gyroBias.x());
    m_gyro.setY(-(m_gyro.y() - m_gyroBias.y()));
    m_gyro.setZ(-(m_gyro.z() - m_gyroBias.z()));

    //  sort out accel data;

    m_accel.setX(-m_accel.x());

    //  sort out compass axes

    float temp;

    temp = m_compass.x();
    m_compass.setX(m_compass.y());
    m_compass.setY(-temp);

    //  use the fuse data adjustments

    m_compass.setX(m_compass.x() * m_compassAdjust[0]);
    m_compass.setY(m_compass.y() * m_compassAdjust[1]);
    m_compass.setZ(m_compass.z() * m_compassAdjust[2]);

    //  calibrate if required

    if (!m_calibrationMode && m_calibrationValid) {
        m_compass.setX((m_compass.x() - m_compassCalOffset[0]) * m_compassCalScale[0]);
        m_compass.setY((m_compass.y() - m_compassCalOffset[1]) * m_compassCalScale[1]);
        m_compass.setZ((m_compass.z() - m_compassCalOffset[2]) * m_compassCalScale[2]);
    }

    //  update running average

    m_compassAverage.setX(m_compass.x() * COMPASS_ALPHA + m_compassAverage.x() * (1.0 - COMPASS_ALPHA));
    m_compassAverage.setY(m_compass.y() * COMPASS_ALPHA + m_compassAverage.y() * (1.0 - COMPASS_ALPHA));
    m_compassAverage.setZ(m_compass.z() * COMPASS_ALPHA + m_compassAverage.z() * (1.0 - COMPASS_ALPHA));
    m_compass = m_compassAverage;

    if (m_firstTime)
        m_timestamp = millis();
    else
        m_timestamp += m_sampleInterval;

    m_firstTime = false;
/*
   File myFile;
   if(SD.exists("output.txt"))
     SD.remove("output.txt");
   myFile = SD.open("output.txt", FILE_WRITE);
   String str = (String)(time) + ",";
   str +=(String)(m_gyro.data(0)) + ","; 
   str += (String)(m_gyro.data(1)) + ",";
   str += (String)(m_gyro.data(2)) + ",";
   str += (String)(m_accel.data(0)) + ",";
   str += (String)(m_accel.data(1)) + ",";
   str += (String)(m_accel.data(2)) + ",";
   str += (String)(m_compass.data(0)) + ",";
   str += (String)(m_compass.data(1)) + ",";
   str += (String)(m_compass.data(2)) + "\n";
   myFile.print(str); 
      
   myFile.close();
*/
    return true;
}
