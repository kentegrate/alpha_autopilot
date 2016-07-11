/*
MS5611 driver code is placed under the BSD license.
Copyright (c) 2014, Emlid Limited, www.emlid.com
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
      * Neither the name of the Emlid Limited nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL EMLID LIMITED BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*From Navio Resository adapted for bcm_2835 library and SPI by Ken Takaki
  https://github.com/emlid/Navio/blob/master/C%2B%2B/Navio/MS5611.cpp  */


#include <alpha_localization/ms5611.h>
#include <bcm2835.h>
#include <ros/ros.h>
#define MS5611_PIN BCM2835_SPI_CS0

MS5611::MS5611() {
  if (!bcm2835_init())
    {
      printf("bcm2835_init failed. Are you running as root??\n");
      return 1;
    }
  if (!bcm2835_spi_begin())
    {
      printf("bcm2835_spi_begin failedg. Are you running as root??\n");
      return 1;
    }
  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // The default
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_65536); // The default
  bcm2835_spi_setChipSelectPolarity(MS5611_PIN, LOW);      // the default
}
unsigned int MS5611::WriteReg(uint8_t WriteAddr, uint8_t WriteData){
{
  bcm2835_spi_chipSelect(MS5611_PIN);                      // The default
  bcm2835_spi_transfer(WriteAddr);
  return bcm2835_spi_transfer(WriteData);
}

//-----------------------------------------------------------------------------------------------

unsigned int  MS5611::ReadReg( uint8_t WriteAddr)
{
  return WriteReg(WriteAddr | READ_FLAG, 0x00);
}

//-----------------------------------------------------------------------------------------------

void MS5611::ReadRegs( uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes )
{
  bcm2835_spi_chipSelect(MS5611_PIN);                      // The default  
  bcm2835_spi_transfer(ReadAddr | READ_FLAG);
  for(int i = 0; i < Bytes; i++)
    ReadBuf[i] = bcm2835_spi_transfer(0x00);

  usleep(50);

}
/** Power on and prepare for general usage.
 * This method reads coefficients stored in PROM.
 */
void MS5611::initialize() {
  // Reading 6 calibration data values
  uint8_t buff[2];
  ReadRegs(MS5611_RA_C1,buff,2);
  C1 = buff[0]<<8 | buff[1];
  ReadRegs(MS5611_RA_C2,buff,2);
  C2 = buff[0]<<8 | buff[1];
  ReadRegs(MS5611_RA_C3,buff,2);
  C3 = buff[0]<<8 | buff[1];
  ReadRegs(MS5611_RA_C4,buff,2);
  C4 = buff[0]<<8 | buff[1];
  ReadRegs(MS5611_RA_C5,buff,2);
  C5 = buff[0]<<8 | buff[1];
  ReadRegs(MS5611_RA_C6,buff,2);
  C6 = buff[0]<<8 | buff[1];

  update();
}

/** Verify the SPI connection.
 * @return True if connection is valid, false otherwise
 */
bool MS5611::testConnection() {
  uint8_t data;
  int8_t status = ReadReg(MS5611_RA_C0);
  if (status > 0)
    return true;
  else
    return false;
}

/** Initiate the process of pressure measurement
 * @param OSR value
 * @see MS5611_RA_D1_OSR_4096
 */
void MS5611::refreshPressure(uint8_t OSR) {
  WriteReg(OSR,0);
  //  I2Cdev::writeBytes(devAddr, OSR, 0, 0);
}

/** Read pressure value
 */
void MS5611::readPressure() {
  //
  uint8_t buffer[3];
  ReadRegs(MS5611_RA_ADC, buffer, 3);
  D1 = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
}

/** Initiate the process of temperature measurement
 * @param OSR value
 * @see MS5611_RA_D2_OSR_4096
 */
 void MS5611::refreshTemperature(uint8_t OSR) {
  WriteReg(OSR,0);
  //  I2Cdev::writeBytes(devAddr, OSR, 0, 0);
}

/** Read temperature value
 */
void MS5611::readTemperature() {
  uint8_t buffer[3];
  ReadRegs(MS5611_RA_ADC, buffer, 3);
  D2 = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
}

/** Calculate temperature and pressure calculations and perform compensation
 *  More info about these calculations is available in the datasheet.
 */
void MS5611::calculatePressureAndTemperature() {
  float dT = D2 - C5 * pow(2, 8);
  TEMP = (2000 + ((dT * C6) / pow(2, 23)));
  float OFF = C2 * pow(2, 16) + (C4 * dT) / pow(2, 7);
  float SENS = C1 * pow(2, 15) + (C3 * dT) / pow(2, 8);

  float T2, OFF2, SENS2;

  if (TEMP >= 2000)
    {
      T2 = 0;
      OFF2 = 0;
      SENS2 = 0;
    }
  if (TEMP < 2000)
    {
      T2 = dT * dT / pow(2, 31);
      OFF2 = 5 * pow(TEMP - 2000, 2) / 2;
      SENS2 = OFF2 / 2;
    }
  if (TEMP < -1500)
    {
      OFF2 = OFF2 + 7 * pow(TEMP + 1500, 2);
      SENS2 = SENS2 + 11 * pow(TEMP + 1500, 2) / 2;
    }

  TEMP = TEMP - T2;
  OFF = OFF - OFF2;
  SENS = SENS - SENS2;

  // Final calculations
  PRES = ((D1 * SENS) / pow(2, 21) - OFF) / pow(2, 15) / 100;
  TEMP = TEMP / 100;
}

/** Perform pressure and temperature reading and calculation at once.
 *  Contains sleeps, better perform operations separately.
 */
void MS5611::update() {
  refreshPressure();
  usleep(10000); // Waiting for pressure data ready
  readPressure();

  refreshTemperature();
  usleep(10000); // Waiting for temperature data ready
  readTemperature();

  calculatePressureAndTemperature();
}

/** Get calculated temperature value
 @return Temperature in degrees of Celsius
*/
float MS5611::getTemperature() {
  return TEMP;
}

/** Get calculated pressure value
 @return Pressure in millibars
*/
float MS5611::getPressure() {
  return PRES;
}
int main(int argc, char* argv[]){
  ros::init(argc,argv,"test");
  MS5611 ms5611;
  ms5611.initialize();
  while(ros::ok()){
    ms5611.update();
    std::cout<<"pressure "<<ms5611.getPressure()<<std::endl;
  }
  return 0;
}
