/* Licensed under the MIT license
 * Copyright 2014 Michael Torrie
 * torriem@gmail.com
 */

#include "ppmreader.h"
#include <limits.h>


//TODO should this be configurable by the caller?
//This sets up a moving average so that what was seen
//before is less important.
static const double coeff = 0.10;//0.10

void PPMReader::on_trigger(void)
{
	uint32_t now;
	uint32_t delta;
	uint16_t ppm;

	if (record)
		totalpulses ++;
	/* after the first 10 ticks, this will calculate an average
	 * speed over the last 10 ticks each tick.  It's a bit slower
	 * but it's going to be a lot smoother.  Hopefully eliminates
	 * spikes
	 */
	now = micros();

	/* Not sure if this is really necessary but wait for the ring
	 * buffer of last times to be completely full (at least 
	 * TICKS_PER_CALC number of ticks) before starting calculations.
	 */
	if (0 == last_times[last_time]) {
		last_times[last_time] = now;
		last_time = (last_time + 1 ) % TICKS_PER_CALC;
		return;
	}

	ppm = (uint16_t)( TICKS_PER_CALC *
	        /* convert microseconds to minutes */
	        1000000 * 60 /
		/* divided by microseconds passed */
	       (now - last_times[last_time]) );
	last_time = (last_time + 1 ) % TICKS_PER_CALC;

	/* smooth ppm using a moving average. */
	ave_ppm = ppm * coeff + ave_ppm * (1 - coeff);

	last_interrupt = millis();

}

uint16_t PPMReader::get_ppm(void) {
	uint32_t now;
	uint16_t max_ppm;

	//TODO possibly turn off interrupts here
	//I'm tempted to leave them on though, as if the ISR runs
	//durring this routine, it's not a big deal, but it could
	//introduce some noise.
	
	if (ave_ppm < ppm_cutoff) {
		ave_ppm = 0;
		return ave_ppm;
	}

	/* if too much time has elapsed since our last interrupt,
	 * let's zero out the average pulses/minute
	 */

	now = millis() - last_interrupt;

	if (now > zero_time) {
		/* if a certain amount of time has passed without an
		 * interrupt, assume we're stopped, since we're going
		 * so slow as to just as well be stopped.
		 */
		ave_ppm = 0;

	} else {
		/* calculate the maximum ppm, given time since last
		 * interrupt
		 */
		max_ppm = 60000 /*ms in a minute*/ 
		          / now; //leaves us with per minute

		/* if an interrupt by now would result in a ppm that
		 * is already lower than the current running average,
		 * at least we know we're slowing down, so go ahead and
		 * bring the average down
		 */
		if (ave_ppm > max_ppm) {
			ave_ppm = max_ppm * coeff + 
				ave_ppm * 
				(1 - coeff);
		}
	}
	return ave_ppm;
}

PPMReader::PPMReader () 
{
	totalpulses = 0;

	//default values that can be overridden
	ppm_cutoff = 50; //50
	zero_time = 1000; //1000
	record = false;
	reset();
}

void PPMReader::reset(void) {
	//TODO maybe disable interrupts for this
	ave_ppm = 0;
	totalpulses = 0;
	memset(last_times,0,sizeof(last_times));
	last_time=0;
	last_interrupt = millis();
}


