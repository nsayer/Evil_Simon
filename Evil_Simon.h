extern unsigned long ticks();

// how many times does ticks() increment per second?
// Keep this synced with the configuration of Timer C4!
#define F_TICK (8000UL)

// Port C pin 4 is the SD card !CS pin.
#define ASSERT_CS (PORTC.OUTCLR = _BV(4))
#define DEASSERT_CS (PORTC.OUTSET = _BV(4))

