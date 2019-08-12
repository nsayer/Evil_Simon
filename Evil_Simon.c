
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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "pff.h"
#include "Evil_Simon.h"

// EEPROM layout
// 0-3: random seed
#define EE_RAND_SEED ((void*)0)

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

// F_TICK is defined in the .h file.
// Debounce time is 50 ms
#define DEBOUNCE_TICKS (F_TICK / 20)
// While in attract mode, wait 30 seconds for a game and then go to sleep
#define SLEEP_TIMEOUT (30 * F_TICK)
// While it's the player's turn, wait 10 seconds before giving up on him
#define TURN_TIMEOUT (10 * F_TICK)

// Thanks to Gareth Evans at http://todbot.com/blog/2008/06/19/how-to-do-big-strings-in-arduino/
// Note that you must be careful not to use this macro more than once per "statement", lest you
// risk overwriting the buffer before it is used. So no using it inside methods that return
// strings that are then used in snprintf statements that themselves use this macro.
char p_buffer[96];
#define P(str) (strcpy_P(p_buffer, PSTR(str)), p_buffer)

volatile unsigned char disp_buf[4];

// The double audio buffers. The lenght of each MUST be even!
volatile unsigned char audio_buf[2][512];

volatile unsigned long ticks_cnt;
unsigned char more_audio, audio_playing;

unsigned char last_button_state;
unsigned long debounce_start;

// PFF's structure
FATFS fatfs;

// ticks counter and display raster ISR
ISR(TCC4_OVF_vect) {
        TCC4.INTFLAGS = TC4_OVFIF_bm; // ack
	if (++ticks_cnt == 0) ticks_cnt++; // it must never be zero.

	static unsigned char disp_num;
	PORTD.OUTSET = 0xf0; // turn all the digits off.
	PORTD.OUTCLR = 0x07; // turn all the color pins off.
	if (ticks_cnt % 4) return; // Only go at 25% duty cycle
	if (++disp_num >= 4) disp_num = 0;
	switch(disp_buf[disp_num]) {
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

unsigned long ticks() {
        unsigned long out;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                out = ticks_cnt;
        }
        return out;
}

static void __ATTR_NORETURN__ power_off(void) {
	eeprom_update_dword(EE_RAND_SEED, random());
	PORTC.DIRCLR = _BV(0); // make the power pin suddenly go Hi-Z.
	while(1) wdt_reset(); // wait patiently for death
	__builtin_unreachable();
}

static void __ATTR_NORETURN__ fail(unsigned char type) {
	// announce the failure

	// XXX TODO

	while(1) wdt_reset(); // wait forever
	__builtin_unreachable();
}

// Read a block of audio and perform mu-law decompression
static size_t read_audio(void* buf, size_t len) {
	unsigned int cnt;
	if (pf_read(buf, len / 2, &cnt) != FR_OK) {
		// indicate an error
		fail(3);
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
		if ((EDMA.STATUS & (EDMA_CH2BUSY_bm | EDMA_CH0BUSY_bm)) == 0) audio_playing = 0;
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
	// If we aborted, then the start address will be wrong and must be reset.
	EDMA.CH0.ADDR = (unsigned int)&(audio_buf[0]);
	EDMA.CH2.ADDR = (unsigned int)&(audio_buf[1]);

        // first, make sure the transfer complete flags are clear.
        EDMA.INTFLAGS |= EDMA_CH0TRNFIF_bm | EDMA_CH2TRNFIF_bm;

        unsigned int cnt = read_audio((void *)(audio_buf[0]), sizeof(audio_buf[0]));
        if (!cnt) return; // empty file - we're done

	// We're certainly going to play *something* at this point
	audio_playing = 1;
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

void __ATTR_NORETURN__ main(void) {

	// The very first thing - set the power-hold pin low.
	PORTC.OUTCLR = _BV(0);
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
        PR.PRPA = PR_ADC_bm | PR_AC_bm;
        PR.PRPC = PR_TWI_bm | PR_HIRES_bm | PR_USART0_bm | PR_TC5_bm;
        PR.PRPD = PR_USART0_bm | PR_TC5_bm;

        PORTA.DIRCLR = 0xff; // All of port A is inputs
        PORTA.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc; //... except that pin 2 is DAC output
        PORTA.PIN4CTRL = PORT_OPC_PULLUP_gc; // all of the buttons get pull-ups
        PORTA.PIN5CTRL = PORT_OPC_PULLUP_gc;
        PORTA.PIN6CTRL = PORT_OPC_PULLUP_gc;
        PORTA.PIN7CTRL = PORT_OPC_PULLUP_gc;

        PORTC.OUTSET = _BV(4); // !SD_CS defaults to high
        PORTC.DIRSET = _BV(4) | _BV(5) | _BV(7); // !SD_CS and most of SPI is output
        PORTC.PIN6CTRL = PORT_OPC_PULLUP_gc; // pull MISO up

        // TCC4 is an 8 kHz clock for triggering DMA to the DAC for audio
        // playback.
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

        DACA.CTRLA = DAC_CH0EN_bm | DAC_ENABLE_bm;
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

	// clear the display buffer
	memset(&disp_buf, 0, sizeof(disp_buf));

	last_button_state = 0;

	srandom(eeprom_read_dword(EE_RAND_SEED));

	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	sei();

        if (pf_mount(&fatfs) != FR_OK) {
		fail(1);
        }

	unsigned char level, step;
	unsigned char pattern[256];
	while(1) {

		// Attract mode
		unsigned char game_select;
		unsigned long wait_start = ticks();
		while(1) {
			wdt_reset();
			unsigned long now = ticks();
			if (now - wait_start > SLEEP_TIMEOUT) {
				// clear the display
				memset(&disp_buf, 0, sizeof(disp_buf));
				// Zzzzzzz
				power_off();
				__builtin_unreachable();
			}
			unsigned char light = ((now - wait_start) / F_TICK) % 4;
			for(int i = 0; i < 4; i++) {
				disp_buf[i] = (i == light)? (i + 1) : BLACK;
			}
			if ((game_select = get_buttons())) break;
		}

		memset(&disp_buf, 0, sizeof(disp_buf));

		// Start the game
		level = 0;

		// game loop
		while(1) {
			char fname[2];
			fname[1] = 0;
			if (level >= 250) {
				// I give up
				level = 0;
				goto game_over;
			}
			pattern[level++] = random() % 0x40;
			// My turn
			for(int i = 0; i < level; i++) {
				disp_buf[MOVE_POS(pattern[i])] = MOVE_COLOR(pattern[i]) + 1;
				fname[0] = '1' + MOVE_SOUND(pattern[i]);
				play_file(fname);
				while(audio_poll()) wdt_reset();
				disp_buf[MOVE_POS(pattern[i])] = 0;
				// pause 100 msec
				for(unsigned long delay_start = ticks(); ticks() - delay_start > (F_TICK / 10); ) wdt_reset();
			}
			// Their turn
			for(step = 0; step < level; step++) {
				unsigned long wait_start = ticks();
				unsigned char buttons;
				while((!(buttons = get_buttons())) && (ticks() - wait_start < TURN_TIMEOUT)) {
					audio_poll();
					if (ticks() - wait_start > (F_TICK / 4)) {
						// The feedback is done, so put the home pattern back
						for(int i = 0; i < 4; i++) disp_buf[i] = i + 1;
					}
					wdt_reset();
				}
				unsigned char move;
				switch(buttons) {
					case 0b1:
						move = 0;
						break;
					case 0b10:
						move = 1;
						break;
					case 0b100:
						move = 2;
						break;
					case 0b1000:
						move = 3;
						break;
					default:
						goto game_over; // TIME OUT!!!
				}
				if (MOVE_COLOR(pattern[step]) != move) goto game_over; // WRONG!!!
				memset(&disp_buf, 0, sizeof(disp_buf));
				// Light up what they pushed
				disp_buf[MOVE_COLOR(pattern[step])] = MOVE_COLOR(pattern[step]) + 1;
				fname[0] = '1' + MOVE_COLOR(pattern[step]);
				play_file(fname);
			}
			while(audio_poll()) wdt_reset();
			memset(&disp_buf, 0, sizeof(disp_buf));
			for(unsigned long wait = ticks(); ticks() - wait < F_TICK; ) wdt_reset();
		}

game_over:
		// game over
		if (level != 0) {
			// YOU LOSE!! HAHAHAHAHAHAHAHA!!
			memset(&disp_buf, 0, sizeof(disp_buf));
			unsigned long lose_start = ticks();
			play_file(P("LOSE"));
			// Show them what they were supposed to push
			while(audio_poll()) {
				if (((ticks() - lose_start) / (F_TICK / 4)) % 2)
					disp_buf[MOVE_COLOR(pattern[step])] = MOVE_COLOR(pattern[step]) + 1;
				else
					disp_buf[MOVE_COLOR(pattern[step])] = 0;
			}
		} else {
			// XXX they win.
		}

	}
	__builtin_unreachable();
}

