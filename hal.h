/*
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your
 * option) any later version.
 * This program is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public
 * License along with this program; if not, write to the Free
 * Software Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307, USA.
 *
 */

/*!
 * @file 	hal.h
 * @brief 	Hardware Abstraction Layer
 * @author 	Timo Sandmann
 * @date 	12.05.2009
 */

#ifndef HAL_H_
#define HAL_H_

#ifdef __AVR_ATtiny25__
#include "attiny25_hal.h"

#else
#error "MCU not supported"
#endif // MCU-Type

#include <avr/builtins.h>

/*! Deaktiviert Interrupts global */
#define CLI() __builtin_avr_cli()

/*! Aktiviert Interrupts global */
#define SEI() __builtin_avr_sei()

#endif /* HAL_H_ */
