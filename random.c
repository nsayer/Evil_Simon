// From http://www.pcg-random.org/download.html
// *Really* minimal PCG32 code / (c) 2014 M.E. O'Neill / pcg-random.org
// Licensed under Apache License 2.0 (NO WARRANTY, etc. see website)

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "random.h"

// We want this to be prime so we have a good shot at eventually
// landing on all the values. And this is substantially more than
// we expect the number of random numbers picked in a single session.
#define PRESEED_COUNT 5003

uint32_t pcg32_random_r(pcg32_random_t* rng)
{
    uint64_t oldstate = rng->state;
    // Advance internal state
    rng->state = oldstate * 6364136223846793005ULL + (rng->inc|1);
    // Calculate output function (XSH RR), uses old state for max ILP
    uint32_t xorshifted = ((oldstate >> 18u) ^ oldstate) >> 27u;
    uint32_t rot = oldstate >> 59u;
    return (xorshifted >> rot) | (xorshifted << ((-rot) & 31));
}

uint32_t pcg32_random_bound_r(pcg32_random_t* rng, uint32_t bound)
{
    uint32_t threshold = -bound % bound;
    for (;;) {
        uint32_t r = pcg32_random_r(rng);
        if (r >= threshold)
            return r % bound;
    }
}
void pcg32_preseed(pcg32_random_t* from, pcg32_random_t* to) {
	// first, copy it all
	memcpy((void*)to, (void*) from, sizeof(pcg32_random_t));
	// Now roll the RNG forward to a new block
	for(int i = 0; i < PRESEED_COUNT; i++)
    		to->state = to->state * 6364136223846793005ULL + (to->inc|1);
}

