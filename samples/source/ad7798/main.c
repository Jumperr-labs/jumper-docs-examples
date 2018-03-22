/***************************************************************************//**
 *   @file   AD7799.c
 *   @brief  Implementation of AD7799 Driver.
 *   @author Bancisor MIhai
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 577
*******************************************************************************/

/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include "AD7799.h"				// AD7799 definitions.
#include "Communication.h"		// Communication definitions.

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_delay.h"
#include "jumper.h"
#include "nrf_log_default_backends.h"

#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

const unsigned long test1_value = 6737;
const unsigned long test2_value = 6738;

const uint64_t kTicksPerMs_ = 64000;

/***************************************************************************//**
 * @brief Initializes the AD7799 and checks if the device is present.
 *
 * @param None.
 *
 * @return status - Result of the initialization procedure.
 *                  Example: 1 - if initialization was successful (ID is 0x0B).
 *                           0 - if initialization was unsuccessful.
*******************************************************************************/
unsigned char AD7799_Init(void)
{ 
	unsigned char status = 0x1;
	if((AD7799_GetRegisterValue(AD7799_REG_ID, 1) & 0x0F) != AD7799_ID) {
		status = 0x0;
	}
	
	return(status);
}

/***************************************************************************//**
 * @brief Sends 32 consecutive 1's on SPI in order to reset the part.
 *
 * @param None.
 *SPI_Write
 * @return  None.    
*******************************************************************************/
void AD7799_Reset(void)
{
	unsigned char dataToSend[4] = {0xff, 0xff, 0xff, 0xff};
	unsigned char null_buff[4];
	SPI_Write(dataToSend, 4, null_buff, 0);

}

/***************************************************************************//**
 * @brief Reads the value of the selected register
 *
 * @param regAddress - The address of the register to read.
 * @param size - The size of the register to read.
 *
 * @return data - The value of the selected register register.
*******************************************************************************/
unsigned long AD7799_GetRegisterValue(unsigned char regAddress, unsigned char size /* not include dummy bytes */) {

    uint8_t data = 0x0;
    uint16_t receivedData = 0x0;
	data = AD7799_COMM_READ |  AD7799_COMM_ADDR(regAddress);

    uint8_t rx_buff[3] = {0x0};
	SPI_Write(&data, 1, rx_buff, size + 1);

	if (size == 1) {
		receivedData += (rx_buff[1] << 0);
	}
	else if (size == 2) {
		receivedData += (rx_buff[1] << 8);
		receivedData += (rx_buff[2] << 0);
	}
    return receivedData;
}

/***************************************************************************//**
 * @brief Writes the value to the register
 *
 * @param -  regAddress - The address of the register to write to.
 * @param -  regValue - The value to write to the register.
 * @param -  size - The size of the register to write.
 *
 * @return  None.    
*******************************************************************************/
void AD7799_SetRegisterValue(unsigned char regAddress,
                             unsigned long regValue, 
                             unsigned char size)
{
    unsigned char data[3] = {0x00, 0x00, 0x00};
	data[0] = AD7799_COMM_WRITE |  AD7799_COMM_ADDR(regAddress);
    if(size == 1) {
        data[1] = (unsigned char)regValue;
    }
    if(size == 2) {
		data[2] = (unsigned char)((regValue & 0x0000FF) >> 0);
        data[1] = (unsigned char)((regValue & 0x00FF00) >> 8);
    }

    unsigned char null_buff[2];
	SPI_Write(data, (1 + size), null_buff, 0);
}

/***************************************************************************//**
 * @brief Reads /RDY bit of status reg.
 *
 * @param None.
 *
 * @return rdy	- 0 if RDY is 1.
 *              - 1 if RDY is 0.
*******************************************************************************/
unsigned char AD7799_Ready(void)
{
    unsigned char rdy = 0;
    rdy = (AD7799_GetRegisterValue( AD7799_REG_STAT,1) & 0x80);   
	
	return(!rdy);
}

/***************************************************************************//**
 * @brief Sets the operating mode of AD7799.
 *
 * @param mode - Mode of operation.
 *
 * @return  None.    
*******************************************************************************/
void AD7799_SetMode(unsigned long mode) {

    unsigned long command;
    command = AD7799_GetRegisterValue(AD7799_REG_MODE, 2);
    command &= ~AD7799_MODE_SEL(0xFF);
    command |= AD7799_MODE_SEL(mode);

    AD7799_SetRegisterValue(
            AD7799_REG_MODE,
            command,
            2
    );
}
/***************************************************************************//**
 * @brief Selects the channel of AD7799.
 *
 * @param  channel - ADC channel selection.
 *
 * @return  None.    
*******************************************************************************/
void AD7799_SetChannel(unsigned long channel)
{
    unsigned long command;
    command = AD7799_GetRegisterValue(AD7799_REG_CONF,2);
    command &= ~AD7799_CONF_CHAN(0xFF);
    command |= AD7799_CONF_CHAN(channel);
    AD7799_SetRegisterValue(
            AD7799_REG_CONF,
            command,
            2
    );
}

/***************************************************************************//**
 * @brief  Sets the gain of the In-Amp.
 *
 * @param  gain - Gain.
 *
 * @return  None.    
*******************************************************************************/
void AD7799_SetGain(unsigned long gain)
{
    unsigned long command;
    command = AD7799_GetRegisterValue(AD7799_REG_CONF,2);
    command &= ~AD7799_CONF_GAIN(0xFF);
    command |= AD7799_CONF_GAIN(gain);
    AD7799_SetRegisterValue(
            AD7799_REG_CONF,
            command,
            2
    );
}

/***************************************************************************//**
 * @brief Enables or disables the reference detect function.
 *
 * @param state - State of the reference detect function.
 *               Example: 0	- Reference detect disabled.
 *                        1	- Reference detect enabled.
 *
 * @return None.    
*******************************************************************************/
void AD7799_SetReference(unsigned char state)
{
    unsigned long command = 0;
    command = AD7799_GetRegisterValue(AD7799_REG_CONF,2);
    command &= ~AD7799_CONF_REFDET(1);
    command |= AD7799_CONF_REFDET(state);
    AD7799_SetRegisterValue(AD7799_REG_CONF,
							command,
							2);
}

/***************************************************************************//**
 * @brief Set the ad7798 to continuous read mode
 * 
 * @return None.    
*******************************************************************************/
void SetContinuousReadMode() {
	unsigned char data[3] = {0x00, 0x00, 0x00};
	data[0] = AD7799_COMM_WEN;
	data[0] |= AD7799_COMM_READ;
	data[0] |= AD7799_COMM_ADDR(AD7799_REG_DATA);
	data[0] |= AD7799_COMM_CREAD;

	data[0] = 0x5C;
	unsigned char null_buff[3];
	SPI_Write(data, 1, null_buff, 0);	
}

/***************************************************************************//**
 * @brief return next data when ad7798 at continuous read mode.
 * 
 * @return data register(measurement data).    
*******************************************************************************/
unsigned long CREADNextRead() {
    uint8_t tx_buf = AD7799_REG_COMM | AD7799_COMM_WEN | AD7799_COMM_READ;
    uint8_t rx_buf[3] = {0x0};

	SPI_Write(&tx_buf, 1, rx_buf, 3);

    uint16_t received_data = 0;
    received_data += (rx_buf[1] << 8);
    received_data += (rx_buf[2] << 0);

	return received_data;
}

/***************************************************************************//**
 * @brief exit the ad7798 from continuous read mode
 * 
 * @return None.    
*******************************************************************************/
void ExitContinuousReadMode() {
	unsigned char data[3] = {0x00, 0x00, 0x00};

	data[0] = 0x58;
	unsigned char null_buff[3];
	SPI_Write(data, 1, null_buff, 0);	
}

void TestReadSingleConvertion(uint8_t test_number, uint16_t expected_value) {
	//single conversion mode
    AD7799_SetMode(AD7799_MODE_SINGLE);
	char print_once = 0;
	while (!AD7799_Ready()) {
		if (0 == print_once) {
			NRF_LOG_INFO("Not ready yet.. \r\n");
			NRF_LOG_FLUSH();
			print_once = 1;
		}
	}
	unsigned long data;
	data = AD7799_GetRegisterValue(AD7799_REG_DATA, 2);
	NRF_LOG_INFO("Test #%d. Data register: %d\r\n", test_number, data);
	NRF_LOG_FLUSH();
	if (data == expected_value) {
		NRF_LOG_INFO("Data test #%d OK\r\n", test_number);
		NRF_LOG_FLUSH();
	}
	else {
		NRF_LOG_INFO("Data test #%d mismatch\r\n", test_number);
		NRF_LOG_FLUSH();
	}
}

void TestContinuousConversion(uint16_t first_value) {

	NRF_LOG_INFO("Continuous conversion\r\n");
	NRF_LOG_FLUSH();

	//Go to Continuous conversion mode
	AD7799_SetMode(AD7799_MODE_CONT);

	uint8_t test_succeed = 1;
	for (int i=0; i<3; i++) {
		while (!AD7799_Ready()); //busy waiting
		//take data
		unsigned long data = AD7799_GetRegisterValue(AD7799_REG_DATA, 2);
		NRF_LOG_INFO("Data register: %d\r\n", data);
		NRF_LOG_FLUSH();
		if (data != first_value++) {
			test_succeed = 0;
		}
	}

    if (test_succeed) {
        NRF_LOG_INFO("Continuous conversion OK\r\n");
    } else {
        NRF_LOG_INFO("Continuous conversion failed\r\n");
    }

	NRF_LOG_INFO("End of continuous conversion\r\n");
	NRF_LOG_FLUSH();
}

float GetFrequency() {
	return 16.7;
}

uint64_t GetMs() {
	return 50;
}

void TestContinuousRead() {

	AD7799_SetMode(AD7799_MODE_CONT);
	NRF_LOG_INFO("Set MODE_CONT\r\n");
	NRF_LOG_FLUSH();
	SetContinuousReadMode();
	NRF_LOG_INFO("Set COMM_CREAD\r\n");
	NRF_LOG_FLUSH();

	const uint64_t ms_delay = GetMs();
	nrf_delay_ms(ms_delay);
	const int cread_test_values[] = {6742, 6743, 6744};
    bool test_succeed = true;
	for (int i = 0; i < 3; i++) {
		nrf_delay_ms(ms_delay);
		unsigned long data = CREADNextRead();
		NRF_LOG_INFO("Continuous read. Data: %d\r\n", data);
		NRF_LOG_FLUSH();
		if (data != cread_test_values[i]) {
			test_succeed = false;
            break;
		}
	}
	ExitContinuousReadMode();

    if (test_succeed) {
        NRF_LOG_INFO("Continuous read OK\r\n");
    } else {
        NRF_LOG_INFO("Continuous read failed\r\n");
    }
    NRF_LOG_FLUSH();
}

int main(void) {
	APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

	SPI_Init(); //TODO: change params list
	
    unsigned char result = AD7799_Init();
	NRF_LOG_INFO("Test #%d. Init should be 1: %d\r\n", 0, result);
	NRF_LOG_FLUSH();

	NRF_LOG_INFO("AD7798\r\n");
	NRF_LOG_FLUSH();

	TestReadSingleConvertion(1, test1_value);
	
	jumper_sudo_exit_with_exit_code(0);

	while(true) {}

    return 0;
}
