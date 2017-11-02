/* Licensed under the MIT license
 * Copyright 2014 Michael Torrie
 * torriem@gmail.com
 */
#include <stdint.h>
#include <Arduino.h>


#ifndef __PPMREADER_H__
#define __PPMREADER_H__

#define TICKS_PER_CALC 10

class PPMReader
{
protected:
	uint32_t last_interrupt;

	uint32_t last_times[TICKS_PER_CALC];
	uint8_t last_time;

public:
	uint8_t   record;
	uint16_t  ave_ppm;
	uint32_t  totalpulses;

	uint16_t zero_time; //ms since last interrupt to zero ave ppm (stopped)
	uint16_t ppm_cutoff; //ppm at which we may as well just go to zero.
	                     //probably not needed check() has other better ways

	PPMReader();
	uint16_t get_ppm(void);
	void reset(void);
	void on_trigger(void); //must be called from a static isr wrapper func
};
#endif

