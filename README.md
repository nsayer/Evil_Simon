
# Симон (aka Evil Simon)

This is the source code for https://hackaday.io/project/166766-Симон

Note that this codebase includes Petit FAT FS. Petit FAT FS can be found at http://elm-chan.org/fsw/ff/00index_p.html

The code I've written, exclusive of random.c and Petit FAT FS (but *inclusive* of the portion of diskio.c that is the actual code
that reads from flash), is licensed under GPL V2. That does not apply to Petit, of course, which remains covered by what
is essentially an MIT license. Nor does it apply to random.c, which is a modified excerpt from Berkeley libc.

## Setup

Upload a suitable disk image with audio samples to the SPI flash chip first. If the XMega chip is programmed, then you
must hold it in RESET (or erase it) to keep the SPI bus quiescent for programming.

Build the code with the included Makefile, which assumes you have an avr-gcc with XMega support in your path. Upload
it to the controller on the board with avrdude and a PDI capable programmer.

Note that there are two options for the audio storage: either SPI flash or an SD card. You should pick the appropriate option
in the Makefile depending on your hardware.

The flash ROM is loaded with a FAT12, FAT16 or FAT32 filesystem in an MBR partition table. The filesystem has the following
files:

* 1, 2, 3, 4: the sound files. When not scrambled, they are in order Red, Green, Blue, Yellow.
* 12, 23, 34, 14, 13, 24: Sound files for chords - for each, it's the two given files from before mixed together.
* WIN_n where n starts at 1 and counts arbitrarily high: the sound files played when the player manages to win, which is to say
they get to a certain number of steps in the pattern (right now, that's 250, so it seems unlikely to ever happen). One is picked
at random to play.
* LOSE_n where n starts at 1 and counts arbitrarily high: the sound files played when the player loses. Like the WIN_ files, one
of these is randomly selected to be played.

The sound files are all raw, single channel, 8 bit mu-law encoded 8 kHz audio files. You can make them with 'sox', and the output
file parameters are "-t raw -r 8000 -c 1 -b 8 -e mu-law".

To prepare the disk image, first make a file the size of the chip filled with 0xff bytes: "dd if=/dev/zero bs=8m count=1 | tr '\0' '\377' > image".
Next, make a FAT filesystem on the image: "mkfs.vfat image". Next, mount the image: "mount -o loop image /mnt". Copy the files into the image and unmount it. "image" is now the file you need to burn into the flash ROM.

## Usage

To play the game, remember the pattern of the colored lights Симон shows, and then repeat the pattern by pressing the
correct colored buttons. Every time you get the pattern right, the pattern will be made one step longer, and the game will
speed up the playback a little bit.

Depending on the difficulty level of the game, the sounds and the positions may not remain consistent. However, in every case,
it is the *color* that you must match.

To begin the game, first press a button to wake Симон up (if the lights are out). It will show the 4 colors racing around in a circle. Press one or more
buttons to select a level of difficulty:

* Green is easy - it's normal Simon. No tricky stuff. The colors never move and the sounds are consistent.
* Blue is harder - the sounds are no longer tied to the colors during the pattern statement.
* Yellow is hardest - the positions of the colors are no longer constant during the pattern statement.
* Red is nightmare - In addition to the sounds and positions being scrambled during the pattern statement, the button colors
will be randomized at the start of the player's turn.
* The "impossible" level is activated by pressing any two buttons to start. Here the button colors are
randomized at every step during the player's turn.

