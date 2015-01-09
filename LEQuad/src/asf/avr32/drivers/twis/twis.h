/*This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief TWIS driver for AVR32 UC3.
 *
 * This file defines a useful set of functions for TWIS on AVR32 devices.
 *
 * - Compiler:           IAR EWAVR32 and GNU GCC for AVR32
 * - Supported devices:  All AVR32 devices with a TWIS module can be used.
 * - AppNote:
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support and FAQ: http://support.atmel.no/
 *
 *****************************************************************************/
/** Copyright (C) 2010 Atmel Corporation. All rights reserved.
 *
 * \page License
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
 * 3. The name of Atmel may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 * Atmel AVR product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

#ifndef _TWIS_H_
#define _TWIS_H_

#include <avr32/io.h>
#include <stdint.h>
#include "compiler.h"
#include "status_codes.h"
#include "conf_twis.h"

/** 
 * \name TWI Driver Compatibility
 * Codes for UC3 devices using TWI modules can easily be ported
 * to UC3 devices with TWIM module
 * @{
 */
//#define avr32_twi_t avr32_twis_t
//#define twi_options_t twis_options_t
//#define twi_package_t twis_package_t
#define twi_slave_fct_t twis_slave_fct_t
#define twi_slave_init twis_slave_init
//! @}

/*!
 * \brief Input parameters when initializing the twi module mode
 */
typedef struct
{
	//! The PBA clock frequency.
	uint32_t pba_hz;
	//! The baudrate of the TWI bus.
	uint32_t speed;
	//! The desired address.
	uint8_t chip;
	//! smbus mode
	bool smbus;
	//! tenbit mode
	bool tenbit;
}
twis_options_t;

/*!
 * \brief Information concerning the data transmission
 */
typedef struct
{
	//! TWI chip address to communicate with.
	uint8_t chip;
	//! TWI address/commands to issue to the other chip (node).
	uint32_t addr;
	//! Length of the TWI data address segment (1-3 bytes).
	uint8_t addr_length;
	//! Pointer to the transmit buffer
	void *buffer;
	//! The number of bytes in the buffer
	uint32_t length;
}
twis_package_t;

/*!
 * \brief Pointer on TWI slave user specific application routines
 */
typedef struct
{
	//! Routine to receive data from TWI master
	void (*rx) (uint8_t);
	//! Routine to transmit data to TWI master
	uint8_t (*tx) (void);
	//! Routine to signal a TWI STOP
	void (*stop) (void);
}
twis_slave_fct_t;

/**
 * \brief Initialize the TWI Slave Module
 *
 * \param twis         Base address of the TWI (i.e. &AVR32_TWI).
 * \param *opt        Options for initializing the TWI module
 *                    (see \ref twis_options_t)
 * \param *slave_fct  Pointer on application functions
 */
status_code_t twis_slave_init (volatile avr32_twis_t *twis,
		const twis_options_t *opt, const twis_slave_fct_t *slave_fct);

#endif // _TWIS_H_
