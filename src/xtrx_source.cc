/*
 * Copyright (c) 2010, Joshua Lackey
 * Copyright (c) 2017, Sergey Kostanbaev <sergey.kostanbaev@fairwaves.co>
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


#include <stdio.h>
#include <stdlib.h>
#ifndef _WIN32
#include <unistd.h>
#endif
#include <string.h>
#include <pthread.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <complex>

#include <assert.h>

#include "xtrx_source.h"

extern int g_verbosity;


#ifdef _WIN32
inline double round(double x) { return floor(x + 0.5); }
#endif

xtrx_source::xtrx_source(float sample_rate, long int fpga_master_clock_freq, int loglevel) {

	m_fpga_master_clock_freq = fpga_master_clock_freq;
	m_desired_sample_rate = sample_rate;
	m_sample_rate = 0.0;
	m_decimation = 0;
	m_cb = new circular_buffer(CB_LEN, sizeof(complex), 0);

	pthread_mutex_init(&m_u_mutex, 0);

	m_loglevel = loglevel;
}


xtrx_source::xtrx_source(unsigned int decimation, long int fpga_master_clock_freq, int loglevel) {

	m_fpga_master_clock_freq = fpga_master_clock_freq;
	m_sample_rate = 0.0;
	m_cb = new circular_buffer(CB_LEN, sizeof(complex), 0);

	pthread_mutex_init(&m_u_mutex, 0);

	m_decimation = decimation & ~1;
	if(m_decimation < 4)
		m_decimation = 4;
	if(m_decimation > 256)
		m_decimation = 256;

	m_loglevel = loglevel;
}


xtrx_source::~xtrx_source() {

	stop();
	delete m_cb;
	xtrx_close(dev);
	pthread_mutex_destroy(&m_u_mutex);
}


void xtrx_source::stop() {

	pthread_mutex_lock(&m_u_mutex);

	xtrx_stop(dev, XTRX_RX);

	pthread_mutex_unlock(&m_u_mutex);
}


void xtrx_source::start() {

	pthread_mutex_lock(&m_u_mutex);

	xtrx_stop(dev, XTRX_RX);

	xtrx_run_params_t params;
	params.dir = XTRX_RX;
	params.nflags = 0;
	params.rx.chs = XTRX_CH_AB;
	params.rx.flags = XTRX_RSP_SISO_MODE | XTRX_RSP_SCALE;
	params.rx.hfmt = XTRX_IQ_FLOAT32;
	params.rx.wfmt = XTRX_WF_16;
	params.rx.paketsize = 0;
	params.rx_stream_start = 20000;
	params.rx.scale = 32767;

	int r = xtrx_run_ex(dev, &params);
	if (r < 0)
		fprintf(stderr, "WARNING: Failed to run streaming.\n");

	pthread_mutex_unlock(&m_u_mutex);
}


float xtrx_source::sample_rate() {

	return m_sample_rate;

}


int xtrx_source::tune(double freq) {
	double actual;
	int r = 0;

	pthread_mutex_lock(&m_u_mutex);
	if (freq != m_center_freq) {
		r = xtrx_tune(dev, XTRX_TUNE_RX_FDD, freq, &actual);

		if (r < 0)
			fprintf(stderr, "Tuning to %f Hz failed!\n", freq);
		else
			m_center_freq = freq;
	}

	pthread_mutex_unlock(&m_u_mutex);

	return 1; //(r < 0) ? 0 : 1;
}

int xtrx_source::set_freq_correction(int ppm) {
	m_freq_corr = ppm;
	//return rtlsdr_set_freq_correction(dev, ppm);
	//abort();
	fprintf(stderr, "TODO: Need to apply correction of %d ppm\n", ppm);

	return 1;
}

bool xtrx_source::set_antenna(int antenna) {

	return 0;
}

bool xtrx_source::set_gain(float gain) {
	int r;
	double actual;

	/* Enable manual gain */
	fprintf(stderr, "Setting gain: %.1f dB\n", gain);
	//r = rtlsdr_set_tuner_gain(dev, g);
	r = xtrx_set_gain(dev, XTRX_CH_AB, XTRX_RX_LNA_GAIN, gain, &actual);

	return (r < 0) ? 0 : 1;
}


/*
 * open() should be called before multiple threads access xtrx_source.
 */
int xtrx_source::open(unsigned int subdev) {
	int r;
	uint32_t dev_index = subdev;
	double samp_rate = 13e6 / 48.0;
	double actual, abw;
	double master = 0;

	r = xtrx_open("/dev/xtrx0", m_loglevel, &dev);
	if (r < 0) {
		fprintf(stderr, "Failed to open xtrx device %d\n", dev_index);
		exit(1);
	}

	if (m_fpga_master_clock_freq != 0) {
		xtrx_set_ref_clk(dev, m_fpga_master_clock_freq, XTRX_CLKSRC_INT);
	}

	const bool extra_decim = false;
	if (m_decimation > 0) {
		master = 4 * samp_rate * m_decimation * (extra_decim ? 2 : 1);
	}

	/* Set the sample rate */
	r = xtrx_set_samplerate(dev, master, samp_rate, 0.0,
							(extra_decim ? XTRX_SAMPLERATE_FORCE_RX_DECIM : 0),
							NULL, &actual, NULL);
	if (r < 0)
		fprintf(stderr, "WARNING: Failed to set sample rate.\n");
	if (samp_rate != actual) {
		fprintf(stderr, "NOTE: Requested %.3f got %.3f\n", samp_rate, actual);
	}

	m_sample_rate = actual / (extra_decim ? 2 : 1);

	r = xtrx_tune_rx_bandwidth(dev, XTRX_CH_AB, 2e6, &abw);
	if (r < 0)
		fprintf(stderr, "WARNING: Failed to set bandwidth.\n");

	/* works best for GSM */
	xtrx_set_antenna(dev, XTRX_RX_W);

	return 0;
}


int xtrx_source::fill(unsigned int num_samples, unsigned int *overrun_i) {
	complex *c;
	unsigned avail, j;
	unsigned overruns = 0;
#if 1
	static float tmp_data[8192*2];
	float *buf = &tmp_data[0];

	xtrx_recv_ex_info_t ri;
	ri.samples = 8192;
	ri.buffer_count = 1;
	ri.buffers = (void* const* )&buf;
	ri.flags = 0;

	for (; ; num_samples -= 8192) {
		unsigned csm = (num_samples > 8192) ? 8192 : num_samples;

		pthread_mutex_lock(&m_u_mutex);
		if (xtrx_recv_sync_ex(dev, &ri) < 0) {
			pthread_mutex_unlock(&m_u_mutex);

			fprintf(stderr, "error: xtrx_recv_sync_ex\n");
			return -1;
		}
		pthread_mutex_unlock(&m_u_mutex);


		c = (complex *)m_cb->poke(&avail);
		assert(avail >= csm);
		memcpy(c, tmp_data, csm * sizeof(float) * 2);
		m_cb->wrote(csm);

		if (ri.out_samples != ri.samples)
			overruns++;

		if (num_samples <= csm)
			break;
	}
#else

	c = (complex *)m_cb->poke(&avail);
	assert(avail >= num_samples);

	xtrx_recv_ex_info_t ri;
	ri.samples = num_samples;
	ri.buffer_count = 1;
	ri.buffers = (void* const* )&c;
	ri.flags = 0;

	pthread_mutex_lock(&m_u_mutex);
	if (xtrx_recv_sync_ex(dev, &ri) < 0) {
		pthread_mutex_unlock(&m_u_mutex);
		fprintf(stderr, "error: xtrx_recv_sync_ex\n");
		return -1;
	}
	pthread_mutex_unlock(&m_u_mutex);

	m_cb->wrote(num_samples);

	if (ri.out_samples != ri.samples)
		overruns++;

#endif
	if(overrun_i)
		*overrun_i = overruns;
	return 0;
}


int xtrx_source::read(complex *buf, unsigned int num_samples,
   unsigned int *samples_read) {

	unsigned int n;

	if(fill(num_samples, 0))
		return -1;

	n = m_cb->read(buf, num_samples);

	if(samples_read)
		*samples_read = n;

	return 0;
}


/*
 * Don't hold a lock on this and use the usrp at the same time.
 */
circular_buffer *xtrx_source::get_buffer() {

	return m_cb;
}

#define FLUSH_SIZE		8192
int xtrx_source::flush(unsigned int flush_count) {

	unsigned i;

	m_cb->flush();
	for (i = 0; i < flush_count; i++)
		fill(FLUSH_SIZE, 0);
	m_cb->flush();

	return 0;
}
