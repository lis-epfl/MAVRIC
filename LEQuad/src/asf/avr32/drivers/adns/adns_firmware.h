/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file adns_firmware.h
 *
 * \author MAV'RIC Team
 *   
 * \brief Driver for the ADNS optic flow sensor
 *
 ******************************************************************************/

#ifndef ADNS_FIRMWARE_H_
#define ADNS_FIRMWARE_H_


#include <asf.h>
#include "delay.h"
#include "sysclk.h"


//----- SPI Definitions
#define SPI_0_PORT              (&AVR32_SPI0)
#define SPI_SLAVECHIP_NUMBER    (0)
#define ADNS_PIN				AVR32_SPI0_SCK_0_0_PIN


// Apparently mode 3 has no definition. What we need for ADNS is mode 3
#define SPI_MODE_3				3

//----- ADNS register address
#define ADNS_PRODUCT_ID			0x00
#define ADNS_REVISION_ID		0x01
#define ADNS_MOTION				0x02
#define ADNS_DELTAXL			0x03
#define ADNS_DELTAXH			0x04
#define ADNS_DELTAYL			0x05
#define ADNS_DELTAYH			0x06
#define	ADNS_SQUAL_REG			0x07
#define ADNS_INVERSE_PRODUCT_ID	0x3f
#define ADNS_CONF1				0x0f
#define ADNS_CONF2				0x10
#define ADNS_FRAME_CAPTURE		0x12
#define ADNS_LASER_CTRL0		0x20
#define ADNS_CONF5				0x2f
#define ADNS_POWER_UP			0x3a
#define ADNS_MOTION_BURST		0x50
#define ADNS_PIXEL_BURST 		0x64
#define REG_Configuration_V     0x2f
#define REG_Configuration_IV    0x39
#define REG_SROM_Enable         0x13
#define REG_SROM_ID             0x2a
#define REG_SROM_Load_Burst     0x62

//----- ADNS Commands
#define ADNS_POWER_UP_CMD		0x5A

static uint32_t cpu_clkhz;


typedef struct
{
	int16_t flowx;
	int16_t flowy;
	int16_t squal;
}motion_burst_t;

static spi_options_t adns_spi_options={
	// The SPI channel to set up : Memory is connected to CS0
	SPI_SLAVECHIP_NUMBER,
	// Preferred baudrate for the SPI.
	1000000,
	// Number of bits in each character (8 to 16).
	8,
	// Delay before first clock pulse after selecting slave (in PBA clock periods).
	0,
	// Delay between each transfer/character (in PBA clock periods).
	0,
	// Sets this chip to stay active after last transfer to it. :tag
	0,
	// Which SPI mode to use when transmitting.
	SPI_MODE_3,
	// Disables the mode fault detection.
	// With this bit cleared, the SPI master mode will disable itself if another
	// master tries to address it.
	1
};


static inline void usdelay(unsigned long delay){
	cpu_delay_us(delay, cpu_clkhz);
}


//TODO : Documentation
void spi_init_module(void); //TODO : return init result
void adns_ss_assert(void);			// make these inline?
void adns_ss_deassert(void);
void adns_write(uint8_t regaddr, uint8_t txdata); 
uint8_t adns_read(uint8_t regaddr);
motion_burst_t adns_burstread(void);
void adns_reset(void);
void adns_readmotion(void);
void adns_init(void);


#endif /* ADNS_FIRMWARE_H_ */