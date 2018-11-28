/*
 * Copyright (c) 2010, Joshua Lackey
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     *  Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *
 *     *  Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifdef XTRX_DEV
#include "xtrx_source.h"
#else
#include "usrp_source.h"
#endif

#include <string.h>

#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>

#include "fcch_detector.h"
#include "util.h"

#ifdef _WIN32
inline double round(double x) { return floor(x + 0.5); }
#endif

static const unsigned int AVG_COUNT = 100;
static const unsigned int AVG_THRESHOLD = (AVG_COUNT / 10);
static const float OFFSET_MAX = 40e3;

extern int g_verbosity;

static const unsigned int CB_LEN = (16 * 16384);

int offset_detect(SoapySDRDevice *sdr) {

#define GSM_RATE (1625000.0 / 6.0)

	unsigned int new_overruns = 0, overruns = 0;
	int notfound = 0;
	unsigned int s_len, consumed, count;
	float offset = 0.0;
	float avg_offset = 0.0;
	float min = 0.0, max = 0.0;
	float stddev = 0.0;
	float sps;
	float offsets[AVG_COUNT];
	double total_ppm;
	fcch_detector *detector;

	double sample_rate = SoapySDRDevice_getSampleRate(sdr, SOAPY_SDR_RX, 0);

	detector = new fcch_detector(sample_rate);

	/*
	 * We deliberately grab 12 frames and 1 burst.  We are guaranteed to
	 * find at least one FCCH burst in this much data.
	 */
	sps = sample_rate / GSM_RATE;
	s_len = (unsigned int)ceil((12 * 8 * 156.25 + 156.25) * sps);

	SoapySDRStream *rxStream;
	if (SoapySDRDevice_setupStream(sdr, &rxStream, SOAPY_SDR_RX, SOAPY_SDR_CF32, NULL, 0, NULL) != 0) {
		fprintf(stderr, "Error in setupStream: %s\n", SoapySDRDevice_lastError());
	}
	SoapySDRDevice_activateStream(sdr, rxStream, 0, 0, 0); //start streaming

	complex *buff = new complex[s_len * 4];

	// ensure at least s_len contiguous samples are read
	void *buffs[] = {buff}; //array of buffers
	int flags; //flags set by receive operation
	long long timeNs; //timestamp for receive buffer
	int ret = SoapySDRDevice_readStream(sdr, rxStream, buffs, s_len * 4, &flags, &timeNs, 100000);
	printf("ret=%d, flags=%d, timeNs=%lld\n", ret, flags, timeNs);

	if(detector->scan(buff, s_len * 4, &offset, &consumed)) {
		printf("Offset: %.2f\tGSM Rate:%i\ts_len:\n", offset, GSM_RATE, s_len);

		// FCH is a sine wave at GSM_RATE / 4
		offset = offset - GSM_RATE / 4;

		// sanity check offset
		if(fabs(offset) < OFFSET_MAX) {

			offsets[count] = offset;
			count += 1;

			fprintf(stderr, "\toffset %3u: %.2f\n", count, offset);
			if(g_verbosity > 0) {
				fprintf(stderr, "\toffset %3u: %.2f\n", count, offset);
			}
		}
	} else {
		++notfound;
	}

	delete buff;

	//shutdown the stream
	SoapySDRDevice_deactivateStream(sdr, rxStream, 0, 0);
	SoapySDRDevice_closeStream(sdr, rxStream);
	delete detector;

	// construct stats
	sort(offsets, AVG_COUNT);
	avg_offset = avg(offsets + AVG_THRESHOLD, AVG_COUNT - 2 * AVG_THRESHOLD, &stddev);
	min = offsets[AVG_THRESHOLD];
	max = offsets[AVG_COUNT - AVG_THRESHOLD - 1];

	printf("average\t\t[min, max]\t(range, stddev)\n");
	display_freq(avg_offset);
	printf("\t\t[%d, %d]\t(%d, %f)\n", (int)round(min), (int)round(max), (int)round(max - min), stddev);
	printf("overruns: %u\n", overruns);
	printf("not found: %u\n", notfound);

	double current_correction = SoapySDRDevice_getFrequencyCorrection(sdr, SOAPY_SDR_RX, 0);
	double current_frequency = SoapySDRDevice_getFrequency(sdr, SOAPY_SDR_RX, 0);

	total_ppm = current_correction - (avg_offset / current_frequency) * 1000000;

	printf("average absolute error: %.3f ppm\n", total_ppm);

	return 0;
}
