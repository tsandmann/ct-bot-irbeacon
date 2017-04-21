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
 * @file 	attiny25_hal.h
 * @brief 	Hardware Abstraction Layer fuer ATtiny25-MCU.
 * 			Hier befindet sich der Code, den man anpassen muss,
 * 			wenn eine andere MCU verwendet werden soll.
 * 			Dafuer kopiert man diese Datei, passt die Kopie an
 * 			die neue Hardware an und bindet jene in hal.h ein.
 * @author 	Timo Sandmann
 * @date 	12.05.2009
 */


/*!
 * Fuse Bits:
 *  avrdude -c dragon_isp -P usb -p attiny25 -B 1024 -U lfuse:w:0xe2:m -U hfuse:w:0xd7:m -U efuse:w:0xff:m
 *
 * ISP:
 *  1: gruen (!reset)
 *  4: blau (gnd)
 *  5: gelb (mosi)
 *  6: braun (miso)
 *  7: orange (sck)
 *  8: rot (vcc)
 *
 * Pins:
 *  1: PB5 / !reset
 *  2: PB3
 *  3: PB4 / OC1B
 *  4: GND
 *  5: PB0 / mosi / sda
 *  6: PB1 / mios / OC0B
 *  7: PB2 / sck / scl
 *  8: VCC
 */

#ifndef ATTINY25_HAL_H_
#define ATTINY25_HAL_H_

#define IR_DUTYCYCLE_REG		OCR1B	/*!< Register fuer DC der 36 kHz-PWM */
#define SUPPLY_DUTYCYCLE_REG	OCR0B	/*!< Register fuer DC zur Spannungserzeugung */
#define IR_MODULATION_COMP		TIMER1_COMPB_vect	/*!< Interrupt-Vektor des IR-Modulations-Timers (36 kHz) */
#define SUPPLY_TIMER_OVF		TIMER0_OVF_vect		/*!< Interrupt-Vektor des Timers fuer Versorgungsspannung ( 488 Hz) */

/*!
 * Initialisiert alle verwendeten Pins
 */
static inline void init_pins(void) {
	DDRB = _BV(DDB1) | _BV(DDB3) | _BV(DDB4); // PWM-Pins und Debug-LED als Output, Rest als Eingang
	PORTB = (uint8_t)(~_BV(PB1) & ~_BV(PB3) & ~_BV(PB4)); // Pullups an
}

/*!
 * Initialisiert globale Interrupt-Maske
 */
static inline void init_ints(void) {
	GIMSK = (uint8_t)(GIMSK & ~_BV(INT0) & ~_BV(PCIE)); // externe Ints aus
}

/*!
 * Schaltet alle ungenutzten Funktionen ab (Strom sparen)
 */
static inline void unused_features_off(void) {
	ACSR = _BV(ACD); // AC aus
	ADCSRA = 0; // ADC aus
	DIDR0 = 0x3f; // Digital Input Disable
}

/*!
 * Initialisiert die IR-Modulation
 * @param timer_top	Maximum fuer PWM-Timer. Damit wird die PWM-Frequenz von 36 kHz festgelegt.
 */
static inline void init_ir_modulation(uint8_t timer_top) {
	PLLCSR = (uint8_t)(PLLCSR & ~_BV(PCKE)); // System-Clock fuer Timer1
	TCNT1 = 0; // Timer1 vorladen

	TCCR1 = _BV(CS10) | _BV(CTC1); // Prescaler = 1 -> f_timer1 = 8 MHz; Clear Timer/Counter on Compare Match
	GTCCR = _BV(PWM1B) | _BV(COM1B1); // Pulse Width Modulator B Enable; OC1B cleared on compare match. Set when TCNT1 = $00

	OCR1B = 0; // PWM erstmal aus
	OCR1C = timer_top; // 36 kHz
}

static inline void supplyOn(uint8_t dc) __attribute__((always_inline));
/*!
 * Initialisiert die Erzeugung der LED-Versorgungsspannung
 * @param power	DutyCycle fuer PWM
 */
static inline void init_supply_pwm(uint8_t power) {
	TCCR0B = _BV(CS01) | _BV(CS00); // Prescaler 64 -> f_timer0 = 8 MHz / 64 = 125 kHz; f_pwm0 = 8 MHz / (64 * 256) = 488 Hz
	TCNT0 = 0; // Timer0 vorladen
	supplyOn(power);
}

static inline void timer_int_supplyOn_modulationOff(void) __attribute__((always_inline));
/*!
 * Aktiviert den Overflow-Interrupt von Timer0,
 * deaktiviert den Compare-Interrupt von Timer1
 */
static inline void timer_int_supplyOn_modulationOff(void) {
	TCNT0 = 0; // Timer0 reset
	TIMSK = _BV(TOIE0); // Timer/Counter0 Overflow Interrupt Enable, Timer/Counter1 Compare MatchB Interrupt Disable
}

static inline void timer_int_supplyOff_modulationOn(void) __attribute__((always_inline));
/*!
 * Deaktiviert den Overflow-Interrupt von Timer0,
 * aktiviert den Compare-Intterupt von Timer1
 */
static inline void timer_int_supplyOff_modulationOn(void) {
	TIMSK = _BV(OCIE1B); // Timer/Counter0 Overflow Interrupt Disable, Timer/Counter1 Compare MatchB Interrupt Enabled
}

static inline void timer_modulationOn(void) __attribute__((always_inline));
/*!
 * Startet Timer1
 */
static inline void timer_modulationOn(void) {
	PRR = _BV(PRUSI) | _BV(PRADC); // USI aus, ADC-Clock aus
}

static inline void timer_modulationOff(void) __attribute__((always_inline));
/*!
 * Stoppt Timer1
 */
static inline void timer_modulationOff(void) {
	PRR = _BV(PRUSI) | _BV(PRADC) | _BV(PRTIM1); // USI aus, ADC-Clock aus, Timer1 aus
}

/*!
 * Schaltet die Versorgungsspannung fuer die LEDs an
 * @param dc	DutyCycle fuer PWM
 */
static inline void supplyOn(uint8_t dc) {
	SUPPLY_DUTYCYCLE_REG = dc;
	TCCR0A = _BV(COM0B1) | // Clear OC0B on Compare Match, set OC0B at BOTTOM (non-inverting mode)
			 _BV(WGM00) | _BV(WGM01); // Fast PWM
}

static inline void supplyOff(void) __attribute__((always_inline));
/*!
 * Schaltet die Versorgungsspannung fuer die LEDs aus
 */
static inline void supplyOff(void) {
	TCCR0A = _BV(WGM00) | _BV(WGM01); // Fast PWM, disconnect OC0B
}

static inline void set_sleep_mode_idle(void) __attribute__((always_inline));
/*!
 * Setzt den Sleep-Modus auf IDLE
 */
static inline void set_sleep_mode_idle(void) {
	_SLEEP_CONTROL_REG = (uint8_t)((_SLEEP_CONTROL_REG & ~(_BV(SM0) | _BV(SM1))) | SLEEP_MODE_IDLE);
}

static inline void set_debug_led(uint8_t value) __attribute__((always_inline));
/*!
 * Schaltet die Debug-LED an oder aus
 * @param value 1: LED an; 0: LED aus
 */
static inline void set_debug_led(uint8_t value) {
	// wenn value eine Konstante ist, wird aus folgender Zeile nur eine Instruktion (sbi/cbi):
	PORTB = (uint8_t)((PORTB & ~_BV(PB3)) | (_BV(PB3) & ((value & 1) << PB3)));
}

#endif /* ATTINY25_HAL_H_ */
