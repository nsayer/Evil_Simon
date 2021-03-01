/*

    Evil Simon
    Copyright (C) 2019 Nicholas W. Sayer

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
    
  */

#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/atomic.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "pff.h"
#include "random.h"

// define for hardware > 1.3.4.
#define HAS_BATTERY_TEST

#ifdef HAS_BATTERY_TEST
// About 2.21 volts or so
#define AC_BATT_THRESHOLD (41)
#endif

// EEPROM layout
// 0-15: random seed
#define EE_RAND_SEED ((void*)0)

// 16-21: high score per level
#define EE_HIGH_SCORE ((void*)16)

// For the LED raster system, these are the values to set in the display registers
// This is also the "home" button order.
#define BLACK (0)
#define RED (1)
#define GREEN (2)
#define BLUE (3)
#define YELLOW (4)

// We store a move as a single byte with 3 bit fields representing the
// position where we will put the light, the sound we'll play and the
// color (which is what the player actually has to match).
// xxPPSSCC - P = position, S = sound, C = color
#define MOVE_POS(x) ((x >> 4) & 3)
#define MOVE_SOUND(x) ((x >> 2) & 3)
#define MOVE_COLOR(x) ((x >> 0) & 3)

// how many times does ticks() increment per second?
// Keep this synced with the configuration of Timer C4!
#define F_TICK (8000UL)

// Debounce time is ~80 ms
#define DEBOUNCE_TICKS (F_TICK / 12)
// While in attract mode, wait 30 seconds for a game and then go to sleep
#define SLEEP_TIMEOUT (F_TICK * 30)
// While it's the player's turn, wait 10 seconds before giving up on him
#define TURN_TIMEOUT (F_TICK * 10)

// This is the division factor for converting the 8 kHz tick interrupt into the LED raster rate.
// There are 32 cycles in the raster system, so the maximum rate is 250 Hz.
#define RASTER_RATE (1)

// How long can a game be before we give up?
#define MAX_LEVEL 100

// Thanks to Gareth Evans at http://todbot.com/blog/2008/06/19/how-to-do-big-strings-in-arduino/
// Note that you must be careful not to use this macro more than once per "statement", lest you
// risk overwriting the buffer before it is used. So no using it inside methods that return
// strings that are then used in snprintf statements that themselves use this macro.
char p_buffer[96];
#define P(str) (strcpy_P(p_buffer, PSTR(str)), p_buffer)

// The LED "frame buffer" - the raster system reads from here.
// The bottom nibble is the color, and the top nibble is a brightness value
// that goes from 7 (dimmest) to 0 (brightest).
volatile unsigned char disp_buf[4];

// The double audio buffers. The length of each MUST be even!
volatile unsigned char audio_buf[2][512];

volatile unsigned long ticks_cnt;
volatile unsigned char audio_playing;
unsigned char more_audio;

unsigned char last_button_state;
unsigned long debounce_start;

// How many possible "win" and "lose" audio tracks are there?
unsigned long start_max, lose_max, win_max;

// random number seed
pcg32_random_t rand_ctx;

// PFF's structure
FATFS fatfs;

// blank the display
static inline __attribute__ ((always_inline)) void blank_display() {
	memset((void*)&disp_buf, 0, sizeof(disp_buf));
}

// ticks counter and display raster ISR
ISR(TCC4_OVF_vect) {
	TCC4.INTFLAGS = TC4_OVFIF_bm; // ack
	ticks_cnt++;

	static unsigned char disp_num;

	if (ticks_cnt % RASTER_RATE) return;

	if (((ticks_cnt / RASTER_RATE) % 8) == 0) {
		PORTD.OUTSET = 0xf0; // turn all the digits off.
		PORTD.OUTCLR = 0x07; // turn all the color pins off.
		if (++disp_num >= 4) disp_num = 0; // next LED
	}

	unsigned char brightness = (disp_buf[disp_num] >> 4) & 0xf;

	if (((ticks_cnt / RASTER_RATE) % 8) == brightness) {
		unsigned char color = disp_buf[disp_num] & 0xf;
		switch(color) {
			case RED:
				PORTD.OUTSET = 0b0001; // red pin on
				break;
			case GREEN:
				PORTD.OUTSET = 0b0010; // green pin on
				break;
			case BLUE:
				PORTD.OUTSET = 0b0100; // blue pin on
				break;
			case YELLOW:
				PORTD.OUTSET = 0b0011; // red + green pin on
				break;
		}
		PORTD.OUTCLR = 0x10 << disp_num; // turn on the selected display
	}
}

unsigned long inline __attribute__ ((always_inline)) ticks() {
	unsigned long out;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		out = ticks_cnt;
	}
	return out;
}

// We normally do want to update the seed, but not when the battery is about to fail
static void __ATTR_NORETURN__ power_off() {
	cli(); // no more rastering
	PORTD.OUTSET = 0xf0; // LEDs off
	PORTD.OUTCLR = 0x7; // colors off too
	PORTC.OUTCLR = _BV(0); // power down and...
	while(1) wdt_reset(); // wait patiently for death
	__builtin_unreachable();
}

static void __ATTR_NORETURN__ fail(unsigned char type) {
	cli(); // No more rastering.
	// announce the failure - write 4 bit binary value in white LEDs.
	PORTD.OUTSET = 7;
	PORTD.OUTCLR = 8;
	PORTD.OUTSET = 0xf0;
	PORTD.OUTCLR = type << 4;

	while(1) wdt_reset(); // wait forever
	__builtin_unreachable();
}

// Read a block of audio and perform mu-law decompression
static size_t read_audio(void* buf, size_t len) {
	unsigned int cnt;
	if (pf_read(buf, len / 2, &cnt) != FR_OK) {
		// indicate an error
		fail(4);
	}
	for(int i = cnt - 1; i >= 0; i--) {
		unsigned int sample = ((unsigned char *)buf)[i];
		// mu-law decompression and signed to unsigned conversion.
		if (sample == 0xff) sample = 0x8000;
		else if (sample == 0x7f) sample = 0x7fff;
		else {
			unsigned char interval = (sample & 0xf);
			switch(sample >> 4) {
			case 0: sample = 127 + (interval << 10); break;
			case 1: sample = 16511 + (interval << 9); break;
			case 2: sample = 24703 + (interval << 8); break;
			case 3: sample = 28799 + (interval << 7); break;
			case 4: sample = 30847 + (interval << 6); break;
			case 5: sample = 31871 + (interval << 5); break;
			case 6: sample = 32383 + (interval << 4); break;
			case 7: sample = 32639 + (interval << 3); break;
			case 8: sample = 49025 + ((15 - interval) << 10); break;
			case 9: sample = 40833 + ((15 - interval) << 9); break;
			case 0xa: sample = 36737 + ((15 - interval) << 8); break;
			case 0xb: sample = 34689 + ((15 - interval) << 7); break;
			case 0xc: sample = 33665 + ((15 - interval) << 6); break;
			case 0xd: sample = 33153 + ((15 - interval) << 5); break;
			case 0xe: sample = 32897 + ((15 - interval) << 4); break;
			case 0xf: sample = 32769 + ((15 - interval) << 3); break;
			}
		}
		((unsigned char*)buf)[i * 2] = (unsigned char)(sample >> 0);
		((unsigned char*)buf)[i * 2 + 1] = (unsigned char)(sample >> 8);
	}
	return cnt * 2;
}

static unsigned char audio_poll(void) {
	wdt_reset(); // pet the dog
	// Is there anything to do?
	if (!audio_playing) return 0; // duh.
	if (!(EDMA.INTFLAGS & (EDMA_CH0TRNFIF_bm | EDMA_CH2TRNFIF_bm))) return 1;

	// A DMA transfer completed, so we must at least ACK it.
	// which channel?
	unsigned char chan = (EDMA.INTFLAGS & EDMA_CH0TRNFIF_bm)?0:1;
	EDMA.INTFLAGS |= chan?EDMA_CH2TRNFIF_bm:EDMA_CH0TRNFIF_bm; // ack

	if (!more_audio) {
		// If there's no more file reading to do, we may still
		// need to wait for the other buffer before marking
		// playback all the way done.
		if ((EDMA.STATUS & (EDMA_CH2BUSY_bm | EDMA_CH0BUSY_bm)) == 0) {
			DACA.CTRLA &= ~(DAC_CH0EN_bm);
#ifdef HAS_BATTERY_CHECK
			// The battery is allowed to go low during audio
			// but it eventually has to come back.
			check_battery();
#endif
			audio_playing = 0;
		}
		return audio_playing;
	}
	unsigned int cnt = read_audio((void *)(audio_buf[chan]), sizeof(audio_buf[chan]));
	if (cnt == 0) {
		// There was no data in the final read, so don't bother with it.
		more_audio = 0;
		return 1;
	}
	(chan?&(EDMA.CH2):&(EDMA.CH0))->TRFCNT = cnt;
	(chan?&(EDMA.CH2):&(EDMA.CH0))->CTRLA |= EDMA_CH_REPEAT_bm; // continue with this buffer
	if (cnt != sizeof(audio_buf[chan])) {
		// This is the last audio chunk, so we are done.
		// DMA will stop after this buffer plays because the other one won't have had
		// REPEAT turned on.
		more_audio = 0;
	}
	return 1;
}

// To play a file, call play_file with the filename, then
// while(audio_poll()) .... ;
static void play_file(char *filename) {
	// Force an abort by turning off enable and repeat.
	EDMA.CH0.CTRLA &= ~(EDMA_CH_ENABLE_bm | EDMA_CH_REPEAT_bm);
	EDMA.CH2.CTRLA &= ~(EDMA_CH_ENABLE_bm | EDMA_CH_REPEAT_bm);
	// select the file. Do this now so the abort has time to take, if necessary.
	if (pf_open(filename) != FR_OK) {
		fail(2);
	}
	while(EDMA.STATUS & (EDMA_CH0BUSY_bm | EDMA_CH2BUSY_bm)) ; // wait for abort - should be nothing
	// If we aborted, then the addresses will be wrong and must be reset.
	EDMA.CH0.ADDR = (unsigned int)&(audio_buf[0]);
	EDMA.CH2.ADDR = (unsigned int)&(audio_buf[1]);
	EDMA.CH0.DESTADDR = (unsigned int)&(DACA.CH0DATA);
	EDMA.CH2.DESTADDR = (unsigned int)&(DACA.CH0DATA);

	// first, make sure the transfer complete flags are clear.
	EDMA.INTFLAGS |= EDMA_CH0TRNFIF_bm | EDMA_CH2TRNFIF_bm;

	unsigned int cnt = read_audio((void *)(audio_buf[0]), sizeof(audio_buf[0]));
	if (!cnt) return; // empty file - we're done

	// We're certainly going to play *something* at this point
	audio_playing = 1;
	DACA.CTRLA |= DAC_CH0EN_bm;
	EDMA.CH0.TRFCNT = cnt;
	EDMA.CH0.CTRLA |= EDMA_CH_ENABLE_bm; // start
	if (cnt != sizeof(audio_buf[0])) {
		// last read - just return.
		return;
	}

	// Set up the next block immediately
	cnt = read_audio((void *)(audio_buf[1]), sizeof(audio_buf[1]));
	if (!cnt) return; // no actual data - so the first block was the end.
	EDMA.CH2.TRFCNT = cnt;
	EDMA.CH2.CTRLA |= EDMA_CH_REPEAT_bm; // take over from the first channel
	if (cnt == sizeof(audio_buf[1])) {
		// If this isn't the last block, then tell the top layer to
		// continue filling buffers as they become empty.
		more_audio = 1;
	}
}

// Returns a bitmask of buttons - they can be chorded.
unsigned char get_buttons() {
	unsigned long now = ticks();
	if (now == 0) now++; // it must never be zero.
	unsigned char button_state = (((PORTA.IN & 0xf0) ^ 0xf0) >> 4); // buttons are reverse and in the top nibble
	if (button_state != last_button_state) {
		// It changed. It must stay stable for a debounce period before we report.
		last_button_state = button_state;
		debounce_start = now;
		return 0;
	}
	if (debounce_start == 0) return 0; // we're not waiting to report.
	if (now - debounce_start > DEBOUNCE_TICKS) {
		debounce_start = 0; //debounce ends
		return button_state;
	}
	return 0;
}

// Turn two color values into a sound filename. Either a single digit for single
// colors or a chord filename, where the lower number is always first.
static void sound_filename(char *buf, unsigned char color1, unsigned char color2) {
	if (color2 != 0xff) {
		// For chords, the filename is always low-number-first
		if (color1 < color2) {
			buf[0] = '1' + color1;
			buf[1] = '1' + color2;
		} else {
			buf[0] = '1' + color2;
			buf[1] = '1' + color1;
		}
		buf[2] = 0;
	} else {
		buf[0] = '1' + color1;
		buf[1] = 0;
	}
}

#ifdef HAS_BATTERY_TEST
static void __ATTR_NORETURN__ battery_fail() {
	// Blink one red light for a second, then power down
	blank_display();
	for(unsigned long now = ticks(); ticks() - now < F_TICK;) {
		wdt_reset();
		if (((ticks() - now) / (F_TICK / 8)) % 2) {
			disp_buf[0] = RED;
		} else {
			blank_display();
		}
	}
	power_off();
	__builtin_unreachable();
}

ISR(ACA_AC0_vect) {
	if (audio_playing) return; // audio causes surges we need to ignore.
	battery_fail();
}

static inline __attribute__ ((always_inline)) void check_battery() {
	if ((ACA.STATUS & AC_AC0STATE_bm) == 0) battery_fail();
}

#endif

void __ATTR_NORETURN__ main(void) {
	// The very first thing - set the power-hold pin high.
	PORTC.OUTSET = _BV(0);
	PORTC.DIRSET = _BV(0);

	// Next, take control of the LEDs.
	PORTD.OUTSET = 0xf0; // Turn all the anodes off
	PORTD.OUTCLR = 0x7; // Turn all the cathodes off
	PORTD.DIRCLR = _BV(3); // D3 is unused
	PORTD.DIRSET = ~_BV(3); // All the rest are outputs

	// Switch to the 32 MHz internal osc.
	OSC.CTRL |= OSC_RC32MEN_bm;
	while(!(OSC.STATUS & OSC_RC32MRDY_bm)) ; // wait for it.

	_PROTECTED_WRITE(CLK.CTRL, CLK_SCLKSEL_RC32M_gc); // switch to it
	OSC.CTRL &= ~(OSC_RC2MEN_bm); // we're done with the 2 MHz osc.

	// We want a 1 second watchdog
	//wdt_enable(WDTO_1S); // This is broken on XMegas.
	// This replacement code doesn't disable interrupts (but they're not on now anyway)
	_PROTECTED_WRITE(WDT.CTRL, WDT_PER_1KCLK_gc | WDT_ENABLE_bm | WDT_CEN_bm);
	while(WDT.STATUS & WDT_SYNCBUSY_bm) ; // wait for it to take
	// We don't want a windowed watchdog.
	_PROTECTED_WRITE(WDT.WINCTRL, WDT_WCEN_bm);
	while(WDT.STATUS & WDT_SYNCBUSY_bm) ; // wait for it to take

	// Turn off the parts of the chip we don't use.
	PR.PRGEN = PR_XCL_bm | PR_RTC_bm;
#ifdef HAS_BATTERY_TEST
	PR.PRPA = PR_ADC_bm;
#else
	PR.PRPA = PR_ADC_bm | PR_AC_bm;
#endif
	PR.PRPC = PR_TWI_bm | PR_HIRES_bm | PR_USART0_bm | PR_TC5_bm;
	PR.PRPD = PR_USART0_bm | PR_TC5_bm;

	PORTA.DIRCLR = 0xff; // All of port A is inputs
#ifdef HAS_BATTERY_TEST
	PORTA.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc; //... except that pin 1 is the battery voltage input
#endif
	PORTA.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc; //... and pin 2 is DAC output
	PORTA.PIN4CTRL = PORT_OPC_PULLUP_gc; // all of the buttons get pull-ups
	PORTA.PIN5CTRL = PORT_OPC_PULLUP_gc;
	PORTA.PIN6CTRL = PORT_OPC_PULLUP_gc;
	PORTA.PIN7CTRL = PORT_OPC_PULLUP_gc;

	PORTC.OUTSET = _BV(4); // !CS defaults to high
	PORTC.DIRSET = _BV(4) | _BV(5) | _BV(7); // !CS and most of SPI is output
	PORTC.PIN6CTRL = PORT_OPC_PULLUP_gc; // pull up on MISO because it's Hi-Z while !CS is high.

	// TCC4 is an 8 kHz clock for triggering DMA to the DAC for audio
	// playback. It's also a tick counter for events and stuff.
	TCC4.CTRLA = TC45_CLKSEL_DIV8_gc; // 2 MHz timer clocking.
	TCC4.CTRLB = 0;
	TCC4.CTRLC = 0;
	TCC4.CTRLD = 0;
	TCC4.CTRLE = 0;
	TCC4.INTCTRLA = TC45_OVFINTLVL_HI_gc; // interrupt on overflow
	TCC4.INTCTRLB = 0;
	TCC4.PER = 499; // 8 kHz

	// Event 0 is used to trigger DAC conversion during audio output.
	EVSYS.CH0MUX = EVSYS_CHMUX_TCC4_OVF_gc;
	EVSYS.CH0CTRL = 0;

	DACA.CTRLA = DAC_ENABLE_bm;
	DACA.CTRLB = DAC_CHSEL_SINGLE_gc | DAC_CH0TRIG_bm; // Trigger a conversion on event 0 - from the 8 kHz timer
	DACA.CTRLC = DAC_REFSEL_AVCC_gc | DAC_LEFTADJ_bm; // lop off the low 4 bits of each sample
	DACA.EVCTRL = DAC_EVSEL_0_gc; // trigger event 0

	// Load factory calibration into the DAC
	NVM.CMD = NVM_CMD_READ_CALIB_ROW_gc;
	DACA.CH0GAINCAL = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, DACA0GAINCAL));
	DACA.CH0OFFSETCAL = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, DACA0OFFCAL));
	DACA.CH1GAINCAL = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, DACA1GAINCAL));
	DACA.CH1OFFSETCAL = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, DACA1OFFCAL));
	NVM.CMD = NVM_CMD_NO_OPERATION_gc;

	EDMA.CTRL = EDMA_RESET_bm;
	while(EDMA.CTRL & EDMA_RESET_bm); // wait for it

	// DMA is two double-buffered, standard channels.
	EDMA.CTRL = EDMA_ENABLE_bm | EDMA_CHMODE_STD02_gc | EDMA_DBUFMODE_BUF0123_gc | EDMA_PRIMODE_RR0123_gc;

	EDMA.CH0.CTRLA = EDMA_CH_SINGLE_bm | EDMA_CH_BURSTLEN_bm; // single-shot, two byte burst
	EDMA.CH0.CTRLB = 0; // no interrupts
	EDMA.CH0.ADDRCTRL = EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_INC_gc;
	EDMA.CH0.DESTADDRCTRL = EDMA_CH_RELOAD_BURST_gc | EDMA_CH_DIR_INC_gc;
	EDMA.CH0.TRIGSRC = EDMA_CH_TRIGSRC_DACA_CH0_gc;
	EDMA.CH0.DESTADDR = (unsigned int)&(DACA.CH0DATA);
	EDMA.CH0.ADDR = (unsigned int)&(audio_buf[0]);

	// Channel 2 is configured exactly the same way as channel 0, but with the 2nd memory buffer.
	EDMA.CH2.CTRLA = EDMA_CH_SINGLE_bm | EDMA_CH_BURSTLEN_bm; // single-shot, two byte burst
	EDMA.CH2.CTRLB = 0;
	EDMA.CH2.ADDRCTRL = EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_INC_gc;
	EDMA.CH2.DESTADDRCTRL = EDMA_CH_RELOAD_BURST_gc| EDMA_CH_DIR_INC_gc;
	EDMA.CH2.TRIGSRC = EDMA_CH_TRIGSRC_DACA_CH0_gc;
	EDMA.CH2.DESTADDR = (unsigned int)&(DACA.CH0DATA);
	EDMA.CH2.ADDR = (unsigned int)&(audio_buf[1]);

#ifdef HAS_BATTERY_TEST
	ACA.AC0CTRL = AC_HYSMODE_SMALL_gc; // no interrupts, hysteresis.
	ACA.AC0MUXCTRL = AC_MUXPOS_PIN1_gc | AC_MUXNEG_SCALER_gc; // compare pin 1 to voltage scaler
	ACA.AC1CTRL = 0; // disable AC1.
	ACA.AC1MUXCTRL = 0;
	ACA.CTRLA = 0; // no special config
	ACA.CTRLB = AC_BATT_THRESHOLD << AC_SCALEFAC_gp;
	ACA.WINCTRL = 0; // no window
	ACA.CURRCTRL = 0; // no CC source
	ACA.AC0CTRL |= AC_ENABLE_bm; // turn it on
#endif

	// clear the display buffer
	blank_display();

	// Since we just got turned on, a button is probably being held right now.
	// Don't count that press.
	last_button_state = (((PORTA.IN & 0xf0) ^ 0xf0) >> 4);
	debounce_start = 0;

	// Seed the PRNG from our last power-off state.
	eeprom_read_block(&rand_ctx, EE_RAND_SEED, sizeof(rand_ctx));
	// And now set up for the *next* power-up.
	{
		pcg32_random_t preseed;
		pcg32_preseed(&rand_ctx, &preseed);
		eeprom_update_block((void*)&preseed, EE_RAND_SEED, sizeof(rand_ctx));
	}

	// Release the hounds!
	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	sei();

#ifdef HAS_BATTERY_TEST
	// At startup when the battery's low, we should give it
	// a little bit of grace... but not forever.
	for(unsigned long wait = ticks(); ticks() - wait < 4;)
		if ((ACA.STATUS & AC_AC0STATE_bm) != 0) break;
	ACA.STATUS |= AC_AC0IF_bm;
	ACA.AC0CTRL |= AC_INTMODE_FALLING_gc | AC_INTLVL_LO_gc;
	// if it's still low, then it's a failure
	check_battery();
#endif

	if (pf_mount(&fatfs) != FR_OK) {
		fail(1);
	}

	start_max = lose_max = win_max = 0;
	// figure out how many LOSE_ and WIN_ files there are.
	// We assume they're numbered 1-n.
	{
		DIR dir;
		FILINFO file;
		if (pf_opendir(&dir, P("/")) != FR_OK)
			fail(3);
		while(1) {
			if (pf_readdir(&dir, &file) != FR_OK)
				fail(5);
			if (file.fname[0] == 0) break; // we're done
			if (!strncasecmp_P(file.fname, PSTR("WIN_"), 4)) {
				unsigned int n = atoi(&(file.fname[4]));
				if (n > win_max) win_max = n;
				continue;
			}
			if (!strncasecmp_P(file.fname, PSTR("LOSE_"), 5)) {
				unsigned int n = atoi(&(file.fname[5]));
				if (n > lose_max) lose_max = n;
				continue;
			}
			if (!strncasecmp_P(file.fname, PSTR("START_"), 6)) {
				unsigned int n = atoi(&(file.fname[6]));
				if (n > start_max) start_max = n;
				continue;
			}
		}
		
	}

	{
		unsigned char count = 0, doit = 1;
		for(int i = 0; i < 4; i++) {
			if (last_button_state & (1 << i)) count++;
		}
		if (count > 1) {
			// vulcan neck pinch - clear high scores
			for(unsigned long now = ticks(); ticks() - now < F_TICK * 10; ) {
				wdt_reset();
				if ((ticks() - now) / (F_TICK / 5) % 2) {
					blank_display();
				} else {
					for(int i = 0; i < 4; i++) disp_buf[i] = i + 1;
				}
				if (last_button_state != (((PORTA.IN & 0xf0) ^ 0xf0) >> 4)) {
					doit = 0;
					break;
				}
			}
			if (doit) {
				// clear high scores
				for(int i = 0; i < 5; i++)
					eeprom_write_byte(EE_HIGH_SCORE + i, 0xff);
			}
			blank_display();
		}
	}

	while(1) {

		// Attract mode - cycle the brightness up and down displaying the home colors
		unsigned char game_select;
		{
			unsigned long wait_start = ticks();
			unsigned char breath_cycle = 0;
			while(1) {
				wdt_reset();
				unsigned long now = ticks();
				if (now - wait_start > SLEEP_TIMEOUT) {
					// clear the display
					blank_display();
					// Zzzzzzz
					power_off();
					__builtin_unreachable();
				}
				if (!((now - wait_start) % (F_TICK / 10))) {
					if (++breath_cycle >= 0x10) {
						breath_cycle = 0;
					}
					// We want the brightness to go from 0 to 7 and back to 0
					unsigned char brightness = (breath_cycle > 7)?16 - breath_cycle:breath_cycle;
					for(int i = 0; i < 4; i++) {
						disp_buf[i] = (i + 1) | (brightness << 4);
					}
					while(ticks() == now) ; // wait for change.
				}
				if ((game_select = get_buttons())) break;
			}
		}

		blank_display();

		// Translate the button push into a difficulty level.
		// Level 0 is ordinary Simon.
		// Level 1 shifts the sounds around
		// Level 2 shifts the positions around
		// Level 3 shifts the home colors around every turn.
		// Level 4 shifts the home colors around even within a turn.

		switch(game_select) {
			case 1 << (GREEN - 1):
				game_select = 0;
				disp_buf[1] = GREEN;
				break;
			case 1 << (BLUE - 1):
				game_select = 1;
				disp_buf[2] = BLUE;
				break;
			case 1 << (YELLOW - 1):
				game_select = 2;
				disp_buf[3] = YELLOW;
				break;
			case 1 << (RED - 1):
				game_select = 3;
				disp_buf[0] = RED;
				break;
			default: // This must have been a chord.
				game_select = 4;
				for(int i = 0; i < 4; i++)
					disp_buf[i] = i + 1; // light up everything
				break;
		}
		{
			char fname[9];
			unsigned int n = pcg32_random_bound_r(&rand_ctx, start_max) + 1;
			snprintf_P(fname, sizeof(fname), PSTR("START_%d"), n);
			play_file(fname);
		}
		while(audio_poll()) ; // finish the noise

		// blank and wait a tick
		blank_display();
		for(unsigned long start = ticks(); ticks() - start < F_TICK / 2; ) ;
		// We just sat here for a bit... pet the dog.
		wdt_reset();
		
		// Start the game
		unsigned char level = 0, step;
		unsigned char pattern[256][2];
		unsigned char home_row[4];

		// game loop
		while(1) {
			char fname[3];
			if (level > MAX_LEVEL) {
				// I give up
				level = 0;
				goto game_over;
			}
			pattern[level][0] = pcg32_random_bound_r(&rand_ctx, 0x40); // from 0b000000 to 0b111111
			if (game_select < 1) { // At levels less than 1, the sounds become consistent.
				// Copy the color to the sound
				pattern[level][0] = (pattern[level][0] & 0b110011) | (MOVE_COLOR(pattern[level][0]) << 2);
			}
			if (game_select < 2) { // At levels less than 2, the positions become consistent.
				// Copy the color to the position
				pattern[level][0] = (pattern[level][0] & 0b001111) | (MOVE_COLOR(pattern[level][0]) << 4);
			}
			if (level < 5 || pcg32_random_bound_r(&rand_ctx, 10) > (3 + (level - 5) / 5)) // increasing probability as we go
				pattern[level][1] = 0xff; // the top two bits being 1 make this distinct from a move triad.
			else {
				do {
					// make a chord move
					pattern[level][1] = pcg32_random_bound_r(&rand_ctx, 0x40); // from 0b000000 to 0b111111
					if (game_select < 1) { // At levels less than 1, the sounds become consistent.
						// Copy the color to the sound
						pattern[level][1] = (pattern[level][1] & 0b110011) | (MOVE_COLOR(pattern[level][1]) << 2);
					}
					if (game_select < 2) { // At levels less than 2, the positions become consistent.
						// Copy the color to the position
						pattern[level][1] = (pattern[level][1] & 0b001111) | (MOVE_COLOR(pattern[level][1]) << 4);
					}
					// But it has to be completely distinct from the "primary" move.
				} while(MOVE_POS(pattern[level][0]) == MOVE_POS(pattern[level][1]) ||
					MOVE_COLOR(pattern[level][0]) == MOVE_COLOR(pattern[level][1]) ||
					MOVE_SOUND(pattern[level][0]) == MOVE_SOUND(pattern[level][1]));
			}
			level++;
			// My turn
			for(int i = 0; i < level; i++) {
				disp_buf[MOVE_POS(pattern[i][0])] = MOVE_COLOR(pattern[i][0]) + 1;
				if (pattern[i][1] != 0xff)
					disp_buf[MOVE_POS(pattern[i][1])] = MOVE_COLOR(pattern[i][1]) + 1;
				sound_filename(fname, MOVE_SOUND(pattern[i][0]), pattern[i][1] == 0xff?0xff:MOVE_SOUND(pattern[i][1]));
				play_file(fname);
				// How long we show depends on how far we've gone.
				// Start at 1.5 seconds, reduce by a quarter second every 2nd level, minimum 1/4 sec.
				long play_time = ((F_TICK * 3) / 2) - ((level /2) * (F_TICK / 4));
				if (play_time < (long)(F_TICK / 4)) play_time = F_TICK / 4;
				for(unsigned long start = ticks(); ticks() - start < play_time; ) audio_poll();
				blank_display();
				// pause 100 msec
				for(unsigned long delay_start = ticks(); ticks() - delay_start < (F_TICK / 10); ) wdt_reset();
			}

			// Their turn
			for(int i = 0; i < 4; i++) home_row[i] = i;

			for(step = 0; step < level; step++) {
				// for game level 3, perturb the user's button colors once.
				// for game level 4, perturb the user's button colors constantly.
				if (game_select >= 4 || (game_select >= 3 && step == 0)) {
					// do a 4 position knuth shuffle
					for(int i = 0; i < 3; i++) {
						int j = pcg32_random_bound_r(&rand_ctx, (4 - i)) + i; // random number i through 3 inclusive
						unsigned char swap = home_row[i];
						home_row[i] = home_row[j];
						home_row[j] = swap;
					}
				}
				unsigned long wait_start = ticks();
				unsigned char buttons;
				while((!(buttons = get_buttons())) && (ticks() - wait_start < TURN_TIMEOUT)) {
					audio_poll();
					if (ticks() - wait_start > (F_TICK / 4)) {
						// The feedback is done, so put the home pattern back
						for(int i = 0; i < 4; i++) disp_buf[i] = home_row[i] + 1;
					}
					wdt_reset();
				}
				unsigned char move1 = 0xff, move2 = 0xff, button1 = 0xff, button2 = 0xff;;
				for(int i = 0; i < 4; i++) {
					if (buttons & (1 << i)) {
						if (button1 == 0xff)
							button1 = i;
						else
							button2 = i;
					}
				}
				if (button1 != 0xff) move1 = home_row[button1]; // translate it into its displayed color
				if (button2 != 0xff) move2 = home_row[button2];
				if (pattern[step][1] == 0xff) {
					// Single expected
					if (move2 != 0xff) goto game_over; // WRONG! No chords allowed!
					if (MOVE_COLOR(pattern[step][0]) != move1) goto game_over; // WRONG!
				} else {
					// chord expected
					// For testing chords, the order doesn't matter, so test both ways
					if (MOVE_COLOR(pattern[step][0]) != move1 && MOVE_COLOR(pattern[step][0]) != move2) goto game_over; // WRONG!
					if (MOVE_COLOR(pattern[step][1]) != move1 && MOVE_COLOR(pattern[step][1]) != move2) goto game_over; // WRONG!
				}
				blank_display();
				// Light up what they pushed
				disp_buf[button1] = home_row[button1] + 1;
				if (pattern[step][1] != 0xff) {
					disp_buf[button2] = home_row[button2] + 1;
				}
				// play the noises we made.
				sound_filename(fname, MOVE_SOUND(pattern[step][0]), pattern[step][1] == 0xff?0xff:MOVE_SOUND(pattern[step][1]));
				play_file(fname);
			}
			while(audio_poll()) ; // Finish up the last chime
			blank_display();
			// wait a half second
			for(unsigned long wait = ticks(); ticks() - wait < F_TICK / 2; ) wdt_reset();
		}

game_over:
		// game over
		if (level != 0) {
			// YOU LOSE!! HAHAHAHAHAHAHAHA!!
			unsigned char color1 = MOVE_COLOR(pattern[step][0]) + 1; // The first color they should have hit
			unsigned char color2 = 0;
			if (pattern[step][1] != 0xff) color2 = MOVE_COLOR(pattern[step][1]) + 1; // the second color
			unsigned char pos1 = 0, pos2 = 0;
			for(int i = 0; i < 4; i++) {
				if (home_row[i] == color1 - 1) pos1 = i;
				if (home_row[i] == color2 - 1) pos2 = i; // this won't matter if color2 == 0.
			}
			blank_display();
			unsigned long lose_start = ticks();
			{
				char fname[9];
				unsigned int n = pcg32_random_bound_r(&rand_ctx, lose_max) + 1;
				snprintf_P(fname, sizeof(fname), PSTR("LOSE_%d"), n);
				play_file(fname);
			}
			// Show them what they were supposed to push
			while(audio_poll()) {
				if (((ticks() - lose_start) / (F_TICK / 4)) % 2) {
					disp_buf[pos1] = color1;
					if (color2 != 0) disp_buf[pos2] = color2;
				} else {
					disp_buf[pos1] = BLACK;
					if (color2 != 0) disp_buf[pos2] = BLACK;
				}
			}
			blank_display();
			level--; // what did they actually achieve?
			unsigned char hi_score = eeprom_read_byte(EE_HIGH_SCORE + game_select);
			if (hi_score == 0xff || hi_score < level) {
				eeprom_write_byte(EE_HIGH_SCORE + game_select, level);
				play_file(P("HI_SCORE"));
			} else {
				play_file(P("SCORE"));
			}

			while(audio_poll()); // wait it out

			char fname[12];
			if (level < 20) {
				snprintf_P(fname, sizeof(fname), PSTR("NUMBERS/%d"), level);
				play_file(fname);
				while(audio_poll());
			} else {
				snprintf_P(fname, sizeof(fname), PSTR("NUMBERS/%d"), (level / 10) * 10); // twenty...
				play_file(fname);
				while(audio_poll());
				if (level % 10) { // not if an even twenty
					snprintf_P(fname, sizeof(fname), PSTR("NUMBERS/%d"), level % 10); // three
					play_file(fname);
					while(audio_poll());
				}
			}
		} else {
			// you win.
			blank_display();
			eeprom_write_byte(EE_HIGH_SCORE + game_select, MAX_LEVEL); // none shall pass.
			unsigned long win_start = ticks();
			{
				char fname[9];
				unsigned int n = pcg32_random_bound_r(&rand_ctx, win_max) + 1;
				snprintf_P(fname, sizeof(fname), PSTR("WIN_%d"), n);
				play_file(fname);
			}
			while(audio_poll()) {
				for(int i = 0; i < 4; i++) {
					if (((ticks() - win_start) / (F_TICK / 2)) % 2)
						disp_buf[i] = i + 1;
					else
						disp_buf[i] = 0;
				}
			}
			blank_display();
		}

	}
	__builtin_unreachable();
}

