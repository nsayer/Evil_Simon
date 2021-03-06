

// From http://www.pcg-random.org/download.html
// *Really* minimal PCG32 code / (c) 2014 M.E. O'Neill / pcg-random.org
// Licensed under Apache License 2.0 (NO WARRANTY, etc. see website)

typedef struct { uint64_t state;  uint64_t inc; } pcg32_random_t;

uint32_t pcg32_random_r(pcg32_random_t* rng);
uint32_t pcg32_random_bound_r(pcg32_random_t* rng, uint32_t bound);
// Do this at powerup to prepare for the next powerup
void pcg32_preseed(pcg32_random_t* from, pcg32_random_t* to);
