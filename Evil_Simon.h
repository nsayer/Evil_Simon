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

extern unsigned long ticks();

// how many times does ticks() increment per second?
// Keep this synced with the configuration of Timer C4!
#define F_TICK (8000UL)

// Port C pin 4 is the SD card !CS pin.
#define ASSERT_CS (PORTC.OUTCLR = _BV(4))
#define DEASSERT_CS (PORTC.OUTSET = _BV(4))

