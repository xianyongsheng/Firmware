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
#include <drivers/drv_baro.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

#include "baro.h"
#include "adis16488.h"


#define ADIS16488_BARO_SAMPLE_RATE 1000


ADIS16488_baro::ADIS16488_baro(ADIS16488 *parent, const char *path) :
    CDev("ADIS16488_baro", path),
    _parent(parent),
    _baro_topic(nullptr),
    _baro_reports(nullptr),
    _baro_orb_class_instance(-1),
    _baro_class_instance(-1),
    _baro_reads(perf_alloc(PC_COUNT, "adis16488_baro_reads")),
    _comms_errors(perf_alloc(PC_COUNT, "adis16488_comms_errors")),
    _msl_pressure(101325),
    _last_baro_data(0)
{


}

ADIS16488_baro::~ADIS16488_baro()
{
    if (_baro_class_instance != -1) {
        unregister_class_devname(BARO_BASE_DEVICE_PATH, _baro_class_instance);
    }

    if (_baro_reports != nullptr) {
        delete _baro_reports;
    }

    // free perf counters
    perf_free(_baro_reads);
    perf_free(_comms_errors);
}

int
ADIS16488_baro::init()
{
    int ret;

    ret = CDev::init();

    /* if setup failed, bail now */
    if (ret != OK) {
        DEVICE_DEBUG("ADIS16488 baro init failed");
        return ret;
    }

    /* allocate basic report buffers */
    _baro_reports = new ringbuffer::RingBuffer(2, sizeof(baro_report));

    if (_baro_reports == nullptr) {
        goto out;
    }

    _baro_class_instance = register_class_devname(BARO_BASE_DEVICE_PATH);

    struct baro_report brp;

    _baro_reports->get(&brp);

    _baro_topic = orb_advertise_multi(ORB_ID(sensor_baro), &brp,
                     &_baro_orb_class_instance, ORB_PRIO_LOW);


    if (_baro_topic == nullptr) {
        warnx("ADVERT BARO FAIL");
    }

out:
    return ret;
}
bool ADIS16488_baro::check_duplicate(int16_t baro_data)
{
    if (baro_data == _last_baro_data) {
        // it isn't new data - wait for next timer
        return true;
    } else {
       _last_baro_data = baro_data;
        return false;
    }
}
void
ADIS16488_baro::measure(float press)
{
    if (check_duplicate(press)) {
//        return;
    }

    float P = press *4.0f;

    baro_report	brb;
    brb.timestamp = hrt_absolute_time();
    brb.error_count = perf_event_count(_comms_errors);

    /* generate a new report */
    brb.temperature = _parent->_last_temperature / 1.0f;
    brb.pressure = P / 100.0f;		/* convert to millibar */

    /* altitude calculations based on http://www.kansasflyer.org/index.asp?nav=Avi&sec=Alti&tab=Theory&pg=1 */

    /*
     * PERFORMANCE HINT:
     *
     * The single precision calculation is 50 microseconds faster than the double
     * precision variant. It is however not obvious if double precision is required.
     * Pending more inspection and tests, we'll leave the double precision variant active.
     *
     * Measurements:
     * 	double precision: ms5611_read: 992 events, 258641us elapsed, min 202us max 305us
     *	single precision: ms5611_read: 963 events, 208066us elapsed, min 202us max 241us
     */

    /* tropospheric properties (0-11km) for standard atmosphere */
    const double T1 = 15.0 + 273.15;	/* temperature at base height in Kelvin */
    const double a  = -6.5 / 1000;	/* temperature gradient in degrees per metre */
    const double g  = 9.80665;	/* gravity constant in m/s/s */
    const double R  = 287.05;	/* ideal gas constant in J/kg/K */

    /* current pressure at MSL in kPa */
    double p1 = _msl_pressure / 1000.0;

    /* measured pressure in kPa */
    double p = P / 1000.0;

    /*
     * Solve:
     *
     *     /        -(aR / g)     \
     *    | (p / p1)          . T1 | - T1
     *     \                      /
     * h = -------------------------------  + h1
     *                   a
     */
    brb.altitude = (((pow((p / p1), (-(a * R) / g))) * T1) - T1) / a;

    /* publish it */
    if (!(_pub_blocked)) {
        /* publish it */
        orb_publish(ORB_ID(sensor_baro), _baro_topic, &brb);
    }

    _baro_reports->force(&brb);

    /* notify anyone waiting for data */
    poll_notify(POLLIN);

}

ssize_t
ADIS16488_baro::read(struct file *filp, char *buffer, size_t buflen)
{
    unsigned count = buflen / sizeof(baro_report);

    /* buffer must be large enough */
    if (count < 1) {
        return -ENOSPC;
    }

    /* if automatic measurement is not enabled, get a fresh measurement into the buffer */
    if (_parent->_call_interval == 0) {
        _baro_reports->flush();
        /* TODO: this won't work as getting valid BARO
         *       data requires more than one measure cycle
         */
        _parent->measure();
       // measure(baro_press);
    }

    /* if no data, error (we could block here) */
    if (_baro_reports->empty()) {
        //warnx("read baro return EAGAIN %d",-EAGAIN);
        return -EAGAIN;
    }

    perf_count(_baro_reads);

    /* copy reports out of our buffer to the caller */
    baro_report *brp = reinterpret_cast<baro_report *>(buffer);
    int transferred = 0;

    while (count--) {
        if (!_baro_reports->get(brp)) {
            break;
        }

        transferred++;
        brp++;
    }

    /* return the number of bytes transferred */
    return (transferred * sizeof(baro_report));
}

int
ADIS16488_baro::ioctl(struct file *filp, int cmd, unsigned long arg)
{
    switch (cmd) {

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
                        return ioctl(filp, SENSORIOCSPOLLRATE, ADIS16488_BARO_SAMPLE_RATE);

                    /* adjust to a legal polling interval in Hz */
                    default: {
                            if (ADIS16488_BARO_SAMPLE_RATE != arg) {
                                return -EINVAL;
                            }

                            return OK;
                        }
                    }
                }

    case SENSORIOCGPOLLRATE:
        return ADIS16488_BARO_SAMPLE_RATE;

    case SENSORIOCSQUEUEDEPTH: {
            /* lower bound is mandatory, upper bound is a sanity check */
            if ((arg < 1) || (arg > 100)) {
                return -EINVAL;
            }

            irqstate_t flags = irqsave();

            if (!_baro_reports->resize(arg)) {
                irqrestore(flags);
                return -ENOMEM;
            }

            irqrestore(flags);
            return OK;
        }

    case SENSORIOCGQUEUEDEPTH:
        return _baro_reports->size();

    case SENSORIOCRESET:
        /*
         * Since we are initialized, we do not need to do anything, since the
         * PROM is correctly read and the part does not need to be configured.
         */
        return _parent->ioctl(filp, cmd, arg);

    case BAROIOCSMSLPRESSURE:

        /* range-check for sanity */
        if ((arg < 80000) || (arg > 120000)) {
            return -EINVAL;
        }

//		_msl_pressure = arg;
        return OK;

    case BAROIOCGMSLPRESSURE:
        return 0;//_msl_pressure;

    default:
        break;
    }

    /* give it to the bus-specific superclass */
    // return bus_ioctl(filp, cmd, arg);
    return CDev::ioctl(filp, cmd, arg);
}



