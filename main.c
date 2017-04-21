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
 * @file 	main.c
 * @brief 	IR-Baken Steuercode
 * @author 	Timo Sandmann
 * @date 	07.05.2009
 */

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "hal.h"

#define IR_DUTYCYCLE			32		/*!< Puls-Pause-Verhaeltnis fuer Modulation: [0; IR_MODULATION_PWM_TOP-1] */
#define SUPPLY_DUTYCYCLE		40		/*!< Puls-Pause-Verhaeltnis fuer Spannungserzeugung [0; 255] */
#define IR_MODULATION_PWM_TOP	211		/*!< Maximum des PWM-Timers fuer IR-Modulation (erzeugt 36 kHz).
										 * Achtung, Wert muss an den internen Oszillator angepasst werden!
										 * Theoretisch F_CPU / 36000UL - 1, denn f_pwm1 = f_timer1 / (IR_MODULATION_PWM_TOP + 1) -> IR_MODULATION_PWM_TOP = 221 => f_pwm1 = 36 kHz */
#define BITS_TO_SEND			5		/*!< Anzahl der zu sendenden Bits inkl. Startbit */

#define DATA					0x1a00	/*!< Datum, das als RC5-Code gesendet wird */
#define DEBUG_LED						/*!< Schaltet die LED an PB3 mit set_debug_led() */

/*!@todo: DATA aus EEPROM lesen */
static uint16_t data_to_send = DATA | _BV(13);	/*!< Zwischenspeicher fuer Datum */

#ifndef DEBUG_LED
#define set_debug_led(_X)	/*!< Debug-Code deaktiviert */
#endif

/*!
 * Initialisiert die Hardware
 */
static void init(void) {
	CLI();

	wdt_disable(); // Watchdog aus

	init_ints();
	init_pins();

	unused_features_off();

	timer_modulationOn(); // USI aus, ADC-Clock aus

	init_ir_modulation(IR_MODULATION_PWM_TOP);

	init_supply_pwm(SUPPLY_DUTYCYCLE);

	timer_int_supplyOff_modulationOn();

	set_sleep_mode_idle(); // IO-Clock und Timer bleiben aktiv
	sleep_enable(); // Sleep Enable Bit bleibt dauerhaft gesetzt

	SEI();
}

int main(void) __attribute__((OS_main));

/*!
 * Hauptprogramm der IR-Baken Steuerung.
 * Initialisiert lediglich die Hardware und vesetzt die CPU in den Schlaf.
 * Die eigentliche Arbeit wird von den ISRs erledigt.
 * @return 0
 */
int main(void) {
	init();

	while (42) {
		sleep_cpu(); // Dauerschlaf ;-)
	}
}

/*!
 * ISR fuer IR-Modulation. Aufruffrequenz 36 kHz
 * ISR_NAKED, da die Schleife in main() _keine_ Register veraendert
 */
ISR(IR_MODULATION_COMP, ISR_NAKED) {
	static uint8_t int_count = 1;	/*!< zaehlt die Aufrufe der ISR */
	static uint8_t bit_count = 0;	/*!< zaehlt die bereits (vollstaendig) gesendeten Bits des Datums */

	register uint8_t int_count_reg = int_count; // Anzahl der (beim Eintritt in die ISR) bereits gesendeten Samples
	register uint16_t data_reg = data_to_send;

	if (int_count_reg == 19 /*32*/) {
		/* erstes Halbbit fertig */
		if ((data_reg & _BV(13)) == _BV(13)) {
			/* zweites Halbbit einer 1 senden */
			IR_DUTYCYCLE_REG = IR_DUTYCYCLE;
		} else {
			/* zweites Halbbit einer 0 senden */
			IR_DUTYCYCLE_REG = 0;
		}
	} else if (int_count_reg == 38 /*64*/) {
		register uint8_t bit_count_reg = bit_count;
		/* zweites Halbbit fertig */
		data_reg <<= 1; // ein Bit weiter schalten
		bit_count_reg++;
		int_count_reg = 0; // counter zuruecksetzen
		if (bit_count_reg == BITS_TO_SEND) {
			/* Datum komplett gesendet */
			bit_count_reg = 0;
			data_reg = DATA | _BV(13); // Datum neu laden

			/* 6 ms Pause */
			IR_DUTYCYCLE_REG = 0; // LED sofort aus
			supplyOff(); // Versorgungsspannung fuer LEDs aus
			timer_modulationOff(); // Modulation aus
			timer_int_supplyOn_modulationOff(); // diesen Int aus, Beenden der Pause per Timer

			set_debug_led(1);
		}
		if ((data_reg & _BV(13)) == _BV(13)) {
			/* erstes Halbbit einer 1 senden */
			IR_DUTYCYCLE_REG = 0;
		} else {
			/* erstes Halbbit einer 0 senden */
			IR_DUTYCYCLE_REG = IR_DUTYCYCLE;
		}
		bit_count = bit_count_reg;
		data_to_send = data_reg;
	}
	int_count_reg++;
	int_count = int_count_reg;
	reti();
}

/*!
 * ISR fuer Ueberlauf des Versorgungsspannungs-Timers. Aufruffrequenz 488 Hz.
 * Beendet die Pause von 6 ms fuer IR-Modulation wieder.
 * ISR_NAKED, da die Schleife in main() _keine_ Register veraendert.
 */
ISR(SUPPLY_TIMER_OVF, ISR_NAKED) {
	static uint8_t counter = 0;	/*!< zaehlt die Aufrufe der ISR */
	if (counter == 2) {
		/* Pause von 6 ms zu Ende */
		supplyOn(SUPPLY_DUTYCYCLE); // Versorungsspannung fuer LEDs wieder an
		timer_modulationOn(); // IR Modulation fortsetzen
		timer_int_supplyOff_modulationOn(); // diesen Int aus, ISR zur Datenuebertragung wieder an
		counter = 0;

		set_debug_led(0);
	} else {
		counter++;
	}
	reti();
}
