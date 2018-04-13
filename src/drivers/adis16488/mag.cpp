/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mag.cpp
 *
 * Driver for the ak8963 magnetometer within the Invensense adis16488
 *
 * @author Robert Dickenson
 *
 */

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>

#include <systemlib/perf_counter.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/spi.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

#include "mag.h"
#include "adis16488.h"


/* in 16-bit sampling mode the mag resolution is 0.1 milli Gauss per bit */

#define ADIS16488_MAG_RANGE_GA        0.1e-3f;

/* we are using the continuous fixed sampling rate of 100Hz */

#define ADIS16488_MAG_SAMPLE_RATE 1000


ADIS16488_mag::ADIS16488_mag(ADIS16488 *parent, const char *path) :
    CDev("ADIS16488_mag", path),
	_parent(parent),
	_mag_topic(nullptr),
	_mag_orb_class_instance(-1),
    _mag_class_instance(-1),
	_mag_reading_data(false),
	_mag_reports(nullptr),
	_mag_scale{},
	_mag_range_scale(),
    _mag_reads(perf_alloc(PC_COUNT, "adis16488_mag_reads")),
    _mag_errors(perf_alloc(PC_COUNT, "adis16488_mag_errors")),
	_mag_asa_x(1.0),
	_mag_asa_y(1.0),
	_mag_asa_z(1.0),
	_last_mag_data{}
{
	// default mag scale factors
	_mag_scale.x_offset = 0;
	_mag_scale.x_scale  = 1.0f;
	_mag_scale.y_offset = 0;
	_mag_scale.y_scale  = 1.0f;
	_mag_scale.z_offset = 0;
	_mag_scale.z_scale  = 1.0f;

    _mag_range_scale = ADIS16488_MAG_RANGE_GA;
}

ADIS16488_mag::~ADIS16488_mag()
{
	if (_mag_class_instance != -1) {
		unregister_class_devname(MAG_BASE_DEVICE_PATH, _mag_class_instance);
	}

	if (_mag_reports != nullptr) {
		delete _mag_reports;
	}

	perf_free(_mag_reads);
	perf_free(_mag_errors);
}

int
ADIS16488_mag::init()
{
	int ret;

	ret = CDev::init();

	/* if setup failed, bail now */
	if (ret != OK) {
        DEVICE_DEBUG("ADIS16488 mag init failed");
		return ret;
	}

	_mag_reports = new ringbuffer::RingBuffer(2, sizeof(mag_report));

	if (_mag_reports == nullptr) {
		goto out;
	}

    _mag_class_instance = register_class_devname(MAG_BASE_DEVICE_PATH);

	/* advertise sensor topic, measure manually to initialize valid report */
	struct mag_report mrp;
	_mag_reports->get(&mrp);

	_mag_topic = orb_advertise_multi(ORB_ID(sensor_mag), &mrp,
					 &_mag_orb_class_instance, ORB_PRIO_LOW);

	if (_mag_topic == nullptr) {
        warnx("ADVERT MAG FAIL");
	}

out:
	return ret;
}

bool ADIS16488_mag::check_duplicate(uint8_t *mag_data)
{
	if (memcmp(mag_data, &_last_mag_data, sizeof(_last_mag_data)) == 0) {
		// it isn't new data - wait for next timer
		return true;

	} else {
		memcpy(&_last_mag_data, mag_data, sizeof(_last_mag_data));
		return false;
	}
}

void
ADIS16488_mag::measure(struct adis16488_mag_regs data)
{
    if (check_duplicate((uint8_t *)&data.x)/* && !(data.st1 & 0x02)*/) {
        return;
    }

	mag_report	mrb;
	mrb.timestamp = hrt_absolute_time();

	/*
	 * Align axes - note the accel & gryo are also re-aligned so this
	 *              doesn't look obvious with the datasheet
	 */
    mrb.x_raw =  data.x;// data.x;
    mrb.y_raw = -data.y;//-data.y;
    mrb.z_raw = -data.z;;//-data.z;

    float xraw_f =  data.x;// data.x;
    float yraw_f = -data.y;//-data.y;
    float zraw_f = -data.z;;//-data.z;

	/* apply user specified rotation */
	rotate_3f(_parent->_rotation, xraw_f, yraw_f, zraw_f);

	mrb.x = ((xraw_f * _mag_range_scale * _mag_asa_x) - _mag_scale.x_offset) * _mag_scale.x_scale;
	mrb.y = ((yraw_f * _mag_range_scale * _mag_asa_y) - _mag_scale.y_offset) * _mag_scale.y_scale;
	mrb.z = ((zraw_f * _mag_range_scale * _mag_asa_z) - _mag_scale.z_offset) * _mag_scale.z_scale;
    mrb.range_ga = (float)2.5f;
	mrb.scaling = _mag_range_scale;
	mrb.temperature = _parent->_last_temperature;

	mrb.error_count = perf_event_count(_mag_errors);

    if (!(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(sensor_mag), _mag_topic, &mrb);
	}

    _mag_reports->force(&mrb);

    /* notify anyone waiting for data */
    poll_notify(POLLIN);
}

ssize_t
ADIS16488_mag::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(mag_report);

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	if (_parent->_call_interval == 0) {
		_mag_reports->flush();
		/* TODO: this won't work as getting valid magnetometer
		 *       data requires more than one measure cycle
		 */
		_parent->measure();
	}

	/* if no data, error (we could block here) */
	if (_mag_reports->empty()) {
		return -EAGAIN;
	}

	perf_count(_mag_reads);

	/* copy reports out of our buffer to the caller */
    mag_report *mrp = reinterpret_cast<mag_report *>(buffer);
	int transferred = 0;

	while (count--) {
		if (!_mag_reports->get(mrp)) {
			break;
		}

		transferred++;
		mrp++;
	}

	/* return the number of bytes transferred */
    return (transferred * sizeof(mag_report));
}

int
ADIS16488_mag::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCRESET:
		/*
		 * TODO: we could implement a reset of the AK8963 registers
		 */
		//return reset();
		return _parent->ioctl(filp, cmd, arg);

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				/*
				 * TODO: investigate being able to stop
				 *       the continuous sampling
				 */
				//stop();
				return OK;

			/* external signalling not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
                return ioctl(filp, SENSORIOCSPOLLRATE, 1000);

			case SENSOR_POLLRATE_DEFAULT:
                return ioctl(filp, SENSORIOCSPOLLRATE, ADIS16488_MAG_SAMPLE_RATE);

			/* adjust to a legal polling interval in Hz */
			default: {
                    if (ADIS16488_MAG_SAMPLE_RATE != arg) {
						return -EINVAL;
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
        return ADIS16488_MAG_SAMPLE_RATE;

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = irqsave();

			if (!_mag_reports->resize(arg)) {
				irqrestore(flags);
				return -ENOMEM;
			}

			irqrestore(flags);

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _mag_reports->size();

	case MAGIOCGSAMPLERATE:
        return ADIS16488_MAG_SAMPLE_RATE;

	case MAGIOCSSAMPLERATE:

		/*
		 * We don't currently support any means of changing
		 * the sampling rate of the mag
		 */
        if (ADIS16488_MAG_SAMPLE_RATE != arg) {
			return -EINVAL;
		}

		return OK;

	case MAGIOCSSCALE:
		/* copy scale in */
		memcpy(&_mag_scale, (struct mag_scale *) arg, sizeof(_mag_scale));
		return OK;

	case MAGIOCGSCALE:
		/* copy scale out */
		memcpy((struct mag_scale *) arg, &_mag_scale, sizeof(_mag_scale));
		return OK;

	case MAGIOCSRANGE:
		return -EINVAL;

	case MAGIOCGRANGE:
		return 48; // fixed full scale measurement range of +/- 4800 uT == 48 Gauss

	case MAGIOCSELFTEST:
        return OK;

#ifdef MAGIOCSHWLOWPASS

	case MAGIOCSHWLOWPASS:
		return -EINVAL;
#endif

#ifdef MAGIOCGHWLOWPASS

	case MAGIOCGHWLOWPASS:
		return -EINVAL;
#endif

	default:
		return (int)CDev::ioctl(filp, cmd, arg);
	}
}

void
ADIS16488_mag::read_block(uint8_t reg, uint8_t *val, uint8_t count)
{
	uint8_t addr = reg | 0x80;
	uint8_t tx[32] = { addr, };
	uint8_t rx[32];

	_parent->transfer(tx, rx, count + 1);
	memcpy(val, rx + 1, count);
}

