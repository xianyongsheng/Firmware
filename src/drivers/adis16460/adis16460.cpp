/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file ADIS16460.cpp
 *
 * Driver for the Invensense ADIS16460 connected via SPI.
 *
 * @author Andrew Tridgell
 *
 */

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <getopt.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/conversions.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

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

#include "adis16460.h"

// User Register Memory Map from Table 6
#define FLASH_CNT   0x00  //Flash memory write count
#define DIAG_STAT   0x02  //Diagnostic and operational status
#define X_GYRO_LOW  0x04  //X-axis gyroscope output, lower word
#define X_GYRO_OUT  0x06  //X-axis gyroscope output, upper word
#define Y_GYRO_LOW  0x08  //Y-axis gyroscope output, lower word
#define Y_GYRO_OUT  0x0A  //Y-axis gyroscope output, upper word
#define Z_GYRO_LOW  0x0C  //Z-axis gyroscope output, lower word
#define Z_GYRO_OUT  0x0E  //Z-axis gyroscope output, upper word
#define X_ACCL_LOW  0x10  //X-axis accelerometer output, lower word
#define X_ACCL_OUT  0x12  //X-axis accelerometer output, upper word
#define Y_ACCL_LOW  0x14  //Y-axis accelerometer output, lower word
#define Y_ACCL_OUT  0x16  //Y-axis accelerometer output, upper word
#define Z_ACCL_LOW  0x18  //Z-axis accelerometer output, lower word
#define Z_ACCL_OUT  0x1A  //Z-axis accelerometer output, upper word
#define SMPL_CNTR   0x1C  //Sample Counter, MSC_CTRL[3:2]=11
#define TEMP_OUT    0x1E  //Temperature output (internal, not calibrated)
#define X_DELT_ANG  0x24  //X-axis delta angle output
#define Y_DELT_ANG  0x26  //Y-axis delta angle output
#define Z_DELT_ANG  0x28  //Z-axis delta angle output
#define X_DELT_VEL  0x2A  //X-axis delta velocity output
#define Y_DELT_VEL  0x2C  //Y-axis delta velocity output
#define Z_DELT_VEL  0x2E  //Z-axis delta velocity output
#define MSC_CTRL    0x32  //Miscellaneous control
#define SYNC_SCAL   0x34  //Sync input scale control
#define DEC_RATE    0x36  //Decimation rate control
#define FLTR_CTRL   0x38  //Filter control, auto-null record time
#define GLOB_CMD    0x3E  //Global commands
#define XGYRO_OFF   0x40  //X-axis gyroscope bias offset error
#define YGYRO_OFF   0x42  //Y-axis gyroscope bias offset error
#define ZGYRO_OFF   0x44  //Z-axis gyroscope bias offset factor
#define XACCL_OFF   0x46  //X-axis acceleration bias offset factor
#define YACCL_OFF   0x48  //Y-axis acceleration bias offset factor
#define ZACCL_OFF   0x4A  //Z-axis acceleration bias offset factor
#define LOT_ID1     0x52  //Lot identification number
#define LOT_ID2     0x54  //Lot identification number
#define PROD_ID     0x56  //Product identifier
#define SERIAL_NUM  0x58  //Lot-specific serial number
#define CAL_SGNTR   0x60  //Calibration memory signature value
#define CAL_CRC     0x62  //Calibration memory CRC values
#define CODE_SGNTR  0x64  //Code memory signature value
#define CODE_CRC    0x66  //Code memory CRC values

#define ADIS16460_Product	0x404C/* Product ID Description for ADIS16460 */

#define DIR_READ			0x00
#define DIR_WRITE			0x80

#define ADIS16460_ACCEL_DEFAULT_RATE				550
#define ADIS16460_GYRO_DEFAULT_RATE					650

#define ADIS16460_ACCEL_DEFAULT_DRIVER_FILTER_FREQ	30
#define ADIS16460_GYRO_DEFAULT_DRIVER_FILTER_FREQ	30

#define ADIS16460_TIMER_REDUCTION                   -300

#define ADIS16460_ACCEL_MAX_OUTPUT_RATE              1000
#define ADIS16460_GYRO_MAX_OUTPUT_RATE               ADIS16460_ACCEL_MAX_OUTPUT_RATE

#define ADIS16460_ONE_G								9.80665f

#define FW_FILTER									false

#define ADIS16460_BUS_SPEED							1.5*1000*1000
#define T_STALL										9

#define GYROINITIALSENSITIVITY						0.005f/65536.0f
#define ACCELINITIALSENSITIVITY						(0.25f/1000.0f)/65536.0f
#define ACCELDYNAMICRANGE							5.0f

#define DRV_ACC_DEVTYPE_ADIS16460  0x18
#define DRV_GYR_DEVTYPE_ADIS16460  0X26

#define ADI_DEVICE_PATH_ACCEL		"/dev/adis16460_accel"
#define ADI_DEVICE_PATH_GYRO		"/dev/adis16460_gyro"
#define ADI_DEVICE_PATH_ACCEL_EXT	"/dev/adis16460_accel_ext"
#define ADI_DEVICE_PATH_GYRO_EXT	"/dev/adis16460_gyro_ext"

/** driver 'main' command */
extern "C" { __EXPORT int adis16460_main(int argc, char *argv[]); }

ADIS16460::ADIS16460(int bus, const char *path_accel, const char *path_gyro, spi_dev_e device,
         enum Rotation rotation) :
    SPI("ADIS16460", path_accel, bus, device, SPIDEV_MODE3, ADIS16460_BUS_SPEED),
    _gyro(new ADIS16460_gyro(this, path_gyro)),
    _product(0),
    _call{},
    _call_interval(0),
    _gyro_reports(nullptr),
    _gyro_scale{},
    _gyro_range_scale(0.0f),
    _gyro_range_rad_s(0.0f),
    _accel_reports(nullptr),
    _accel_scale{},
    _accel_range_scale(0.0f),
    _accel_range_m_s2(0.0f),
    _accel_topic(nullptr),
    _accel_orb_class_instance(-1),
    _accel_class_instance(-1),
    _dlpf_freq(41),
    _sample_rate(1000),
    _accel_reads(perf_alloc(PC_COUNT, "adis16460_acc_read")),
    _gyro_reads(perf_alloc(PC_COUNT, "adis16460_gyro_read")),
    _sample_perf(perf_alloc(PC_ELAPSED, "adis16460_read")),
    _bad_transfers(perf_alloc(PC_COUNT, "adis16460_bad_trans")),
    _gyro_filter_x(ADIS16460_GYRO_DEFAULT_RATE, ADIS16460_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
    _gyro_filter_y(ADIS16460_GYRO_DEFAULT_RATE, ADIS16460_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
    _gyro_filter_z(ADIS16460_GYRO_DEFAULT_RATE, ADIS16460_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
    _accel_filter_x(ADIS16460_ACCEL_DEFAULT_RATE, ADIS16460_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
    _accel_filter_y(ADIS16460_ACCEL_DEFAULT_RATE, ADIS16460_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
    _accel_filter_z(ADIS16460_ACCEL_DEFAULT_RATE, ADIS16460_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
    _accel_int(1000000 / ADIS16460_ACCEL_MAX_OUTPUT_RATE),
    _gyro_int(1000000 / ADIS16460_GYRO_MAX_OUTPUT_RATE, true),
    _rotation(rotation)
{
    // disable debug() calls
    _debug_enabled = false;

    _device_id.devid_s.devtype = DRV_ACC_DEVTYPE_ADIS16460;

    _gyro->_device_id.devid = _device_id.devid;
    _gyro->_device_id.devid_s.devtype = DRV_GYR_DEVTYPE_ADIS16460;

    // default accel scale factors
    _accel_scale.x_offset = 0;
    _accel_scale.x_scale  = 1.0f;
    _accel_scale.y_offset = 0;
    _accel_scale.y_scale  = 1.0f;
    _accel_scale.z_offset = 0;
    _accel_scale.z_scale  = 1.0f;

    // default gyro scale factors
    _gyro_scale.x_offset = 0;
    _gyro_scale.x_scale  = 1.0f;
    _gyro_scale.y_offset = 0;
    _gyro_scale.y_scale  = 1.0f;
    _gyro_scale.z_offset = 0;
    _gyro_scale.z_scale  = 1.0f;

    memset(&_call, 0, sizeof(_call));
}

ADIS16460::~ADIS16460()
{
    /* make sure we are truly inactive */
    stop();

    /* delete the gyro subdriver */
    delete _gyro;

    /* free any existing reports */
    if (_accel_reports != nullptr) {
        delete _accel_reports;
    }

    if (_gyro_reports != nullptr) {
        delete _gyro_reports;
    }

    if (_accel_class_instance != -1) {
        unregister_class_devname(ACCEL_BASE_DEVICE_PATH, _accel_class_instance);
    }

    /* delete the perf counter */
    perf_free(_sample_perf);
    perf_free(_accel_reads);
    perf_free(_gyro_reads);
    perf_free(_bad_transfers);
}

int
ADIS16460::init()
{
    int ret;

    /* do SPI init (and probe) first */
    ret = SPI::init();

    /* if probe/setup failed, bail now */
    if (ret != OK) {
        DEVICE_DEBUG("SPI setup failed");
        return ret;
    }

    /* allocate basic report buffers */
    _accel_reports = new ringbuffer::RingBuffer(2, sizeof(accel_report));

    if (_accel_reports == nullptr) {
        goto out;
    }

    _gyro_reports = new ringbuffer::RingBuffer(2, sizeof(gyro_report));

    if (_gyro_reports == nullptr) {
        goto out;
    }

    if (reset() != OK) {
        goto out;
    }

    /* Initialize offsets and scales */
    _accel_scale.x_offset = 0;
    _accel_scale.x_scale  = 1.0f;
    _accel_scale.y_offset = 0;
    _accel_scale.y_scale  = 1.0f;
    _accel_scale.z_offset = 0;
    _accel_scale.z_scale  = 1.0f;

    _gyro_scale.x_offset = 0;
    _gyro_scale.x_scale  = 1.0f;
    _gyro_scale.y_offset = 0;
    _gyro_scale.y_scale  = 1.0f;
    _gyro_scale.z_offset = 0;
    _gyro_scale.z_scale  = 1.0f;

    /* do CDev init for the gyro device node, keep it optional */
    ret = _gyro->init();

    /* if probe/setup failed, bail now */
    if (ret != OK) {
        DEVICE_DEBUG("gyro init failed");
        return ret;
    }

    _accel_class_instance = register_class_devname(ACCEL_BASE_DEVICE_PATH);

    measure();

    /* advertise sensor topic, measure manually to initialize valid report */
    struct accel_report arp;
    _accel_reports->get(&arp);

    /* measurement will have generated a report, publish */
    _accel_topic = orb_advertise_multi(ORB_ID(sensor_accel), &arp,
                       &_accel_orb_class_instance, (is_external()) ? ORB_PRIO_MAX - 1 : ORB_PRIO_HIGH - 1);

    if (_accel_topic == nullptr) {
        DEVICE_DEBUG("ADVERT FAIL");
    }

    /* advertise sensor topic, measure manually to initialize valid report */
    struct gyro_report grp;
    _gyro_reports->get(&grp);

    _gyro->_gyro_topic = orb_advertise_multi(ORB_ID(sensor_gyro), &grp,
                 &_gyro->_gyro_orb_class_instance, (is_external()) ? ORB_PRIO_MAX - 1 : ORB_PRIO_HIGH - 1);

    if (_gyro->_gyro_topic == nullptr) {
        DEVICE_DEBUG("ADVERT FAIL");
    }

out:
    return ret;
}

/*
  set sample rate (approximate) - 1kHz to 5Hz, for both accel and gyro
*/
void
ADIS16460::_set_sample_rate(unsigned desired_sample_rate_hz)
{
    if (desired_sample_rate_hz == 0 ||
        desired_sample_rate_hz == GYRO_SAMPLERATE_DEFAULT ||
        desired_sample_rate_hz == ACCEL_SAMPLERATE_DEFAULT) {
        desired_sample_rate_hz = ADIS16460_GYRO_DEFAULT_RATE;
    }

    uint8_t div = 1000 / desired_sample_rate_hz;

    if (div > 200) { div = 200; }

    if (div < 1) { div = 1; }

    _sample_rate = 1000 / div;
}

int
ADIS16460::probe()
{
    uint16_t serial_number;

    /* retry 5 time to get the ADIS16460 PRODUCT ID number */
    for (int i = 0; i < 5; i++) {
        /* recognize product ID */
        _product = read_reg16(PROD_ID);
//        warnx("product %d",_product);
    }
    if (_product == ADIS16460_Product) {
        /* recognize product serial number */
        serial_number = (read_reg16(SERIAL_NUM) & 0xfff);
        DEVICE_DEBUG("ADIS16460 is detected ID: 0x%02x, Serial: 0x%02x", _product, serial_number);
        return OK;
    }

    DEVICE_DEBUG("unexpected ID 0x%02x", _product);
    return -EIO;
}
/* set the DLPF FIR filter tap. This affects both accelerometer and gyroscope. */
void
ADIS16460::_set_dlpf_filter(uint16_t frequency_hz)
{
    /*
       choose next highest filter frequency available
     */
    if (frequency_hz == 0) {
        _dlpf_freq = 0;

    } else if (frequency_hz <= 5) {
        _dlpf_freq = 5;

    } else if (frequency_hz <= 10) {
        _dlpf_freq = 10;

    } else if (frequency_hz <= 20) {
        _dlpf_freq = 20;

    } else if (frequency_hz <= 41) {
        _dlpf_freq = 41;

    } else if (frequency_hz <= 92) {
        _dlpf_freq = 92;

    } else if (frequency_hz <= 184) {
        _dlpf_freq = 184;

    } else if (frequency_hz <= 250) {
        _dlpf_freq = 250;

    } else {
        _dlpf_freq = 0;
    }
}
ssize_t
ADIS16460::read(struct file *filp, char *buffer, size_t buflen)
{
    unsigned count = buflen / sizeof(accel_report);

    /* buffer must be large enough */
    if (count < 1) {
        DEVICE_DEBUG("ENOSPC  %d ",-ENOSPC);
        return -ENOSPC;
    }

    /* if automatic measurement is not enabled, get a fresh measurement into the buffer */
    if (_call_interval == 0) {
        _accel_reports->flush();
        measure();
    }

    /* if no data, error (we could block here) */
    if (_accel_reports->empty()) {
        DEVICE_DEBUG("EAGAIN  %d ",-EAGAIN);
        return -EAGAIN;
    }

    perf_count(_accel_reads);

    /* copy reports out of our buffer to the caller */
    accel_report *arp = reinterpret_cast<accel_report *>(buffer);
    int transferred = 0;

    while (count--) {
        if (!_accel_reports->get(arp)) {
            break;
        }

        transferred++;
        arp++;
    }
    /* return the number of bytes transferred */
    return (transferred * sizeof(accel_report));
}

int
ADIS16460::self_test()
{
    if (perf_event_count(_sample_perf) == 0) {
        measure();
    }

    /* return 0 on success, 1 else */
    return (perf_event_count(_sample_perf) > 0) ? 0 : 1;
}

int
ADIS16460::accel_self_test()
{
    if (self_test()) {
        return 1;
    }

    return 0;
}

int
ADIS16460::gyro_self_test()
{
    if (self_test()) {
        return 1;
    }

    return 0;
}

ssize_t
ADIS16460::gyro_read(struct file *filp, char *buffer, size_t buflen)
{
    unsigned count = buflen / sizeof(gyro_report);

    /* buffer must be large enough */
    if (count < 1) {
        return -ENOSPC;
    }

    /* if automatic measurement is not enabled, get a fresh measurement into the buffer */
    if (_call_interval == 0) {
        _gyro_reports->flush();
        measure();
    }

    /* if no data, error (we could block here) */
    if (_gyro_reports->empty()) {
        return -EAGAIN;
    }

    perf_count(_gyro_reads);

    /* copy reports out of our buffer to the caller */
    gyro_report *grp = reinterpret_cast<gyro_report *>(buffer);
    int transferred = 0;

    while (count--) {
        if (!_gyro_reports->get(grp)) {
            break;
        }

        transferred++;
        grp++;
    }

    /* return the number of bytes transferred */
    return (transferred * sizeof(gyro_report));
}

int
ADIS16460::ioctl(struct file *filp, int cmd, unsigned long arg)
{
    switch (cmd) {

    case SENSORIOCRESET:
        return reset();

    case SENSORIOCSPOLLRATE: {
            switch (arg) {

            /* switching to manual polling */
            case SENSOR_POLLRATE_MANUAL:
                stop();
                _call_interval = 0;
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
                return ioctl(filp, SENSORIOCSPOLLRATE, ADIS16460_ACCEL_DEFAULT_RATE);

            /* adjust to a legal polling interval in Hz */
            default: {
                    /* do we need to start internal polling? */
                    bool want_start = (_call_interval == 0);

                    /* convert hz to hrt interval via microseconds */
                    unsigned ticks = 1000000 / arg;

                    /* check against maximum sane rate */
                    if (ticks < 1000) {
                        return -EINVAL;
                    }

                    // adjust filters
                    float cutoff_freq_hz = _accel_filter_x.get_cutoff_freq();
                    float sample_rate = 1.0e6f / ticks;
                    _set_dlpf_filter(cutoff_freq_hz);
                    _accel_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
                    _accel_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
                    _accel_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz);


                    float cutoff_freq_hz_gyro = _gyro_filter_x.get_cutoff_freq();
                    _set_dlpf_filter(cutoff_freq_hz_gyro);
                    _gyro_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
                    _gyro_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
                    _gyro_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);

                    /* update interval for next measurement */
                    /* XXX this is a bit shady, but no other way to adjust... */
                    _call_interval = ticks;

                    /*
                      set call interval faster than the sample time. We
                      then detect when we have duplicate samples and reject
                      them. This prevents aliasing due to a beat between the
                      stm32 clock and the ADIS16460 clock
                     */
                    _call.period = _call_interval - ADIS16460_TIMER_REDUCTION;

                    /* if we need to start the poll state machine, do it */
                    if (want_start) {
                        start();
                    }

                    return OK;
                }
            }
        }

    case SENSORIOCGPOLLRATE:
        if (_call_interval == 0) {
            return SENSOR_POLLRATE_MANUAL;
        }

        return 1000000 / _call_interval;

    case SENSORIOCSQUEUEDEPTH: {
            /* lower bound is mandatory, upper bound is a sanity check */
            if ((arg < 1) || (arg > 100)) {
                return -EINVAL;
            }

            irqstate_t flags = irqsave();

            if (!_accel_reports->resize(arg)) {
                irqrestore(flags);
                return -ENOMEM;
            }

            irqrestore(flags);

            return OK;
        }

    case SENSORIOCGQUEUEDEPTH:
        return _accel_reports->size();

    case ACCELIOCGSAMPLERATE:
        return _sample_rate;

    case ACCELIOCSSAMPLERATE:
        _set_sample_rate(arg);
        return OK;

    case ACCELIOCGLOWPASS:
        return _accel_filter_x.get_cutoff_freq();

    case ACCELIOCSLOWPASS:
        // set software filtering
        _accel_filter_x.set_cutoff_frequency(1.0e6f / _call_interval, arg);
        _accel_filter_y.set_cutoff_frequency(1.0e6f / _call_interval, arg);
        _accel_filter_z.set_cutoff_frequency(1.0e6f / _call_interval, arg);
        return OK;

    case ACCELIOCSSCALE: {
            /* copy scale, but only if off by a few percent */
            struct accel_calibration_s *s = (struct accel_calibration_s *) arg;
            float sum = s->x_scale + s->y_scale + s->z_scale;

            if (sum > 2.0f && sum < 4.0f) {
                memcpy(&_accel_scale, s, sizeof(_accel_scale));
                return OK;

            } else {
                return -EINVAL;
            }
        }

    case ACCELIOCGSCALE:
        /* copy scale out */
        memcpy((struct accel_calibration_s *) arg, &_accel_scale, sizeof(_accel_scale));
        return OK;

    case ACCELIOCSRANGE:
        return OK;

    case ACCELIOCGRANGE:
        return (unsigned long)((_accel_range_m_s2) / ADIS16460_ONE_G + 0.5f);

    case ACCELIOCSELFTEST:
        return accel_self_test();

#ifdef ACCELIOCSHWLOWPASS

    case ACCELIOCSHWLOWPASS:
        _set_dlpf_filter(arg);
        return OK;
#endif

#ifdef ACCELIOCGHWLOWPASS

    case ACCELIOCGHWLOWPASS:
        return _dlpf_freq;
#endif

    default:
        /* give it to the superclass */
        return SPI::ioctl(filp, cmd, arg);
    }
}

int
ADIS16460::gyro_ioctl(struct file *filp, int cmd, unsigned long arg)
{
    switch (cmd) {

    /* these are shared with the accel side */
    case SENSORIOCSPOLLRATE:
    case SENSORIOCGPOLLRATE:
    case SENSORIOCRESET:
        return ioctl(filp, cmd, arg);

    case SENSORIOCSQUEUEDEPTH: {
            /* lower bound is mandatory, upper bound is a sanity check */
            if ((arg < 1) || (arg > 100)) {
                return -EINVAL;
            }

            irqstate_t flags = irqsave();

            if (!_gyro_reports->resize(arg)) {
                irqrestore(flags);
                return -ENOMEM;
            }

            irqrestore(flags);

            return OK;
        }

    case SENSORIOCGQUEUEDEPTH:
        return _gyro_reports->size();

    case GYROIOCGSAMPLERATE:
        return _sample_rate;

    case GYROIOCSSAMPLERATE:
        _set_sample_rate(arg);
        return OK;

    case GYROIOCGLOWPASS:
        return _gyro_filter_x.get_cutoff_freq();

    case GYROIOCSLOWPASS:
        // set software filtering
        _gyro_filter_x.set_cutoff_frequency(1.0e6f / _call_interval, arg);
        _gyro_filter_y.set_cutoff_frequency(1.0e6f / _call_interval, arg);
        _gyro_filter_z.set_cutoff_frequency(1.0e6f / _call_interval, arg);
        return OK;

    case GYROIOCSSCALE:
        /* copy scale in */
        memcpy(&_gyro_scale, (struct gyro_calibration_s *) arg, sizeof(_gyro_scale));
        return OK;

    case GYROIOCGSCALE:
        /* copy scale out */
        memcpy((struct gyro_calibration_s *) arg, &_gyro_scale, sizeof(_gyro_scale));
        return OK;

    case GYROIOCSRANGE:
        /* XXX not implemented */
        // XXX change these two values on set:
        // _gyro_range_scale = xx
        // _gyro_range_rad_s = xx
        return -EINVAL;

    case GYROIOCGRANGE:
        return (unsigned long)(_gyro_range_rad_s * 180.0f / M_PI_F + 0.5f);

    case GYROIOCSELFTEST:
        return gyro_self_test();

#ifdef GYROIOCSHWLOWPASS

    case GYROIOCSHWLOWPASS:
        _set_dlpf_filter(arg);
        return OK;
#endif

#ifdef GYROIOCGHWLOWPASS

    case GYROIOCGHWLOWPASS:
        return _dlpf_freq;
#endif

    default:
        /* give it to the superclass */
        return SPI::ioctl(filp, cmd, arg);
    }
}

uint16_t
ADIS16460::read_reg16(unsigned reg)
{
    uint8_t cmd[2] = { (uint8_t)(reg), 00};

    //up_udelay(T_STALL);
    transfer(cmd, nullptr, sizeof(cmd));
    up_udelay(T_STALL);
    transfer(nullptr, cmd, sizeof(cmd));

    return ((cmd[0] << 8)|cmd[1]);
}

void
ADIS16460::write_reg16(unsigned reg, uint16_t value)
{
    uint8_t	cmd[2];

    cmd[0] = (reg | DIR_WRITE);
    cmd[1] = (uint8_t)(value);
    transfer(cmd, nullptr, sizeof(cmd));
    up_udelay(T_STALL);
    cmd[0] = ((reg + 0x1) | DIR_WRITE);
    cmd[1] = (uint8_t) (value >> 8);
    transfer(cmd, nullptr, sizeof(cmd));
}

void
ADIS16460::modify_reg16(unsigned reg, uint16_t clearbits, uint16_t setbits)
{
    uint16_t	val;

    val = read_reg16(reg);
    val &= ~clearbits;
    val |= setbits;
    write_reg16(reg, val);
}


void
ADIS16460::start()
{
    stop();

    /* discard any stale data in the buffers */
    _accel_reports->flush();
    _gyro_reports->flush();

    /* start polling at the specified rate */
    hrt_call_every(&_call,
               1000,
               _call_interval - ADIS16460_TIMER_REDUCTION,
               (hrt_callout)&ADIS16460::measure_trampoline, this);
}

void
ADIS16460::stop()
{
    hrt_cancel(&_call);
}

void
ADIS16460::measure_trampoline(void *arg)
{
    ADIS16460 *dev = reinterpret_cast<ADIS16460 *>(arg);

    /* make another measurement */
    dev->measure();
}
int ADIS16460::reset()
{
    _gyro_range_scale = (GYROINITIALSENSITIVITY / 180.0f) * M_PI_F;//GYROINITIALSENSITIVITY;
    _gyro_range_rad_s = (100.0f / 180.0f) * M_PI_F;

    _accel_range_scale = ADIS16460_ONE_G * ACCELINITIALSENSITIVITY;//ACCELINITIALSENSITIVITY;
    _accel_range_m_s2  = ADIS16460_ONE_G * ACCELDYNAMICRANGE;

    usleep(1000);

    return OK;
}
int
ADIS16460::adi_read_reg(struct ADISReport *recv)
{
    recv->gyro_x=((read_reg16(X_GYRO_OUT)<<16)|(0xffff&read_reg16(X_GYRO_LOW)));
    recv->gyro_y=((read_reg16(Y_GYRO_OUT)<<16)|(0xffff&read_reg16(Y_GYRO_LOW)));
    recv->gyro_z=((read_reg16(Z_GYRO_OUT)<<16)|(0xffff&read_reg16(Z_GYRO_LOW)));
    //up_udelay(T_STALL);
    recv->accel_x=((read_reg16(X_ACCL_OUT)<<16)|(0xffff&read_reg16(X_ACCL_LOW)));
    recv->accel_y=((read_reg16(Y_ACCL_OUT)<<16)|(0xffff&read_reg16(Y_ACCL_LOW)));
    recv->accel_z=((read_reg16(Z_ACCL_OUT)<<16)|(0xffff&read_reg16(Z_ACCL_LOW)));

    recv->temp=read_reg16(TEMP_OUT);

    if (recv->gyro_x == 0 && recv->gyro_y == 0 && recv->gyro_z == 0 &&
        recv->accel_x == 0 && recv->accel_y == 0 && recv->accel_z == 0 &&
        recv->temp == 0) {
        perf_count(_bad_transfers);
        perf_end(_sample_perf);
        return -EIO;
    }

    return OK;
}
int
ADIS16460::measure()
{
    struct ADISReport adis_report;

    struct Report {
        int32_t		gyro_x;
        int32_t		gyro_y;
        int32_t		gyro_z;
        int32_t		accel_x;
        int32_t		accel_y;
        int32_t		accel_z;
        int16_t		temp;
    }report;

    /* start measuring */
    perf_begin(_sample_perf);

    /*
     * Fetch the full set of measurements from the ADIS16460 in one pass.
     */
    if (OK != adi_read_reg(&adis_report)) {
        return -EIO;
    }

    report.gyro_x  = (int32_t) adis_report.gyro_x;
    report.gyro_y  =-(int32_t) adis_report.gyro_y;
    report.gyro_z  =-(int32_t) adis_report.gyro_z;
    report.accel_x = (int32_t) adis_report.accel_x;
    report.accel_y =-(int32_t) adis_report.accel_y;
    report.accel_z =-(int32_t) adis_report.accel_z;
    report.temp    = (int16_t) adis_report.temp;

    /*
     * Report buffers.
     */
    accel_report		arb;
    gyro_report		grb;

    grb.timestamp = arb.timestamp = hrt_absolute_time();
    grb.error_count = arb.error_count = perf_event_count(_bad_transfers);

    arb.x_raw = report.accel_x>>16;
    arb.y_raw = report.accel_y>>16;
    arb.z_raw = report.accel_z>>16;

    float xraw_f = report.accel_x;
    float yraw_f = report.accel_y;
    float zraw_f = report.accel_z;

    // apply user specified rotation
    rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

//    DEVICE_DEBUG("ars: %0.5f , as: %0.5f , aso: %0.5f",_accel_range_scale,_accel_scale.x_scale,_accel_scale.x_offset);
    float x_in_new = ((xraw_f * _accel_range_scale) - _accel_scale.x_offset) * _accel_scale.x_scale;
    float y_in_new = ((yraw_f * _accel_range_scale) - _accel_scale.y_offset) * _accel_scale.y_scale;
    float z_in_new = ((zraw_f * _accel_range_scale) - _accel_scale.z_offset) * _accel_scale.z_scale;
    arb.x = _accel_filter_x.apply(x_in_new);
    arb.y = _accel_filter_y.apply(y_in_new);
    arb.z = _accel_filter_z.apply(z_in_new);

    math::Vector<3> aval(x_in_new, y_in_new, z_in_new);
    math::Vector<3> aval_integrated;

    bool accel_notify = _accel_int.put(arb.timestamp, aval, aval_integrated, arb.integral_dt);
    arb.x_integral = aval_integrated(0);
    arb.y_integral = aval_integrated(1);
    arb.z_integral = aval_integrated(2);

    arb.scaling = _accel_range_scale;
    arb.range_m_s2 = _accel_range_m_s2;

    arb.temperature_raw = report.temp;
    arb.temperature = (report.temp) * 0.05f + 25.0f;

    grb.x_raw = report.gyro_x>>16;
    grb.y_raw = report.gyro_y>>16;
    grb.z_raw = report.gyro_z>>16;

    xraw_f = report.gyro_x;
    yraw_f = report.gyro_y;
    zraw_f = report.gyro_z;

    // apply user specified rotation
    rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

//    DEVICE_DEBUG("grs: %0.5f , gs: %0.5f , gso: %0.5f",_gyro_range_scale,_gyro_scale.x_scale,_gyro_scale.x_offset);
    float x_gyro_in_new = ((xraw_f * _gyro_range_scale) - _gyro_scale.x_offset) * _gyro_scale.x_scale;
    float y_gyro_in_new = ((yraw_f * _gyro_range_scale) - _gyro_scale.y_offset) * _gyro_scale.y_scale;
    float z_gyro_in_new = ((zraw_f * _gyro_range_scale) - _gyro_scale.z_offset) * _gyro_scale.z_scale;
    grb.x = _gyro_filter_x.apply(x_gyro_in_new);
    grb.y = _gyro_filter_y.apply(y_gyro_in_new);
    grb.z = _gyro_filter_z.apply(z_gyro_in_new);

    math::Vector<3> gval(x_gyro_in_new, y_gyro_in_new, z_gyro_in_new);
    math::Vector<3> gval_integrated;

    bool gyro_notify = _gyro_int.put(arb.timestamp, gval, gval_integrated, grb.integral_dt);
    grb.x_integral = gval_integrated(0);
    grb.y_integral = gval_integrated(1);
    grb.z_integral = gval_integrated(2);

    grb.scaling = _gyro_range_scale;
    grb.range_rad_s = _gyro_range_rad_s;

    grb.temperature_raw = report.temp;
    grb.temperature = (report.temp) * 0.05f + 25.0f;

    _accel_reports->force(&arb);
    _gyro_reports->force(&grb);

    /* notify anyone waiting for data */
    if (accel_notify) {
        poll_notify(POLLIN);
    }

    if (gyro_notify) {
        _gyro->parent_poll_notify();
    }

    if (accel_notify && !(_pub_blocked)) {
        /* publish it */
        orb_publish(ORB_ID(sensor_accel), _accel_topic, &arb);
    }

    if (gyro_notify && !(_pub_blocked)) {
        /* publish it */
        orb_publish(ORB_ID(sensor_gyro), _gyro->_gyro_topic, &grb);
    }

    /* stop measuring */
    perf_end(_sample_perf);

    return OK;
}

ADIS16460_gyro::ADIS16460_gyro(ADIS16460 *parent, const char *path) :
    CDev("ADIS16460_gyro", path),
    _parent(parent),
    _gyro_topic(nullptr),
    _gyro_orb_class_instance(-1),
    _gyro_class_instance(-1)
{
}

ADIS16460_gyro::~ADIS16460_gyro()
{
    if (_gyro_class_instance != -1) {
        unregister_class_devname(GYRO_BASE_DEVICE_PATH, _gyro_class_instance);
    }
}

int
ADIS16460_gyro::init()
{
    int ret;

    // do base class init
    ret = CDev::init();

    /* if probe/setup failed, bail now */
    if (ret != OK) {
        warnx("gyro init failed");
        return ret;
    }

    _gyro_class_instance = register_class_devname(GYRO_BASE_DEVICE_PATH);

    return ret;
}

void
ADIS16460_gyro::parent_poll_notify()
{
    poll_notify(POLLIN);
}

ssize_t
ADIS16460_gyro::read(struct file *filp, char *buffer, size_t buflen)
{
    return _parent->gyro_read(filp, buffer, buflen);
}

int
ADIS16460_gyro::ioctl(struct file *filp, int cmd, unsigned long arg)
{

    switch (cmd) {
    case DEVIOCGDEVICEID:
        return (int)CDev::ioctl(filp, cmd, arg);
        break;

    default:
        return _parent->gyro_ioctl(filp, cmd, arg);
    }
}


/**
 * Local functions in support of the shell command.
 */
namespace adis16460
{

ADIS16460	*g_dev_int;
ADIS16460	*g_dev_ext;

void	start(bool, enum Rotation);
void	stop(bool);
void	test(bool);
void	reset(bool);
void	usage();
/**
 * Start the driver.
 */
void
start(bool external_bus, enum Rotation rotation)
{
    int fd;
    ADIS16460 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;
    const char *path_accel = external_bus ? ADI_DEVICE_PATH_ACCEL_EXT : ADI_DEVICE_PATH_ACCEL;
    const char *path_gyro  = external_bus ? ADI_DEVICE_PATH_GYRO_EXT : ADI_DEVICE_PATH_GYRO;

    /* if already started, the still command succeeded */
    if (*g_dev_ptr != nullptr)
    {
        errx(0, "already started");
    }

    /* create the driver */
    if (external_bus) {
#ifdef PX4_SPI_BUS_EXT
        *g_dev_ptr = new ADIS16460(PX4_SPI_BUS_EXT, path_accel, path_gyro, (spi_dev_e)PX4_SPIDEV_EXT0, rotation);
        //warnx("External SPI available %u %u\n",PX4_SPI_BUS_EXT,PX4_SPIDEV_EXT0);
#else
        errx(0, "External SPI not available");
#endif

    } else {
        *g_dev_ptr = new ADIS16460(PX4_SPI_BUS_SENSORS, path_accel, path_gyro, (spi_dev_e)PX4_SPIDEV_MPU, rotation);
    }

    if (*g_dev_ptr == nullptr) {
        warnx("g_dev_ptr nullptr.");
        goto fail;
    }

    if (OK != (*g_dev_ptr)->init()) {
        warnx("cant init.");
        goto fail;
    }

    /* set the poll rate to default, starts automatic data collection */
    fd = open(path_accel, O_RDONLY);

    if (fd < 0) {
        warnx("open accel fail.");
        goto fail;
    }

    if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
        warnx("ioctl set rate fail.");
        goto fail;
    }

    close(fd);

    exit(0);
fail:

    if (*g_dev_ptr != nullptr) {
        delete(*g_dev_ptr);
        *g_dev_ptr = nullptr;
    }

    errx(1, "driver start failed");
}
void
stop(bool external_bus)
{
    ADIS16460 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

    if (*g_dev_ptr != nullptr) {
        delete *g_dev_ptr;
        *g_dev_ptr = nullptr;

    } else {
        /* warn, but not an error */
        warnx("already stopped.");
    }

    exit(0);
}
/**
 * Reset the driver.
 */
void
reset(bool external_bus)
{
    const char *path_accel = external_bus ? ADI_DEVICE_PATH_ACCEL_EXT : ADI_DEVICE_PATH_ACCEL;
    int fd = open(path_accel, O_RDONLY);

    if (fd < 0) {
        err(1, "failed ");
    }

    if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
        err(1, "driver reset failed");
    }

    if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
        err(1, "driver poll restart failed");
    }

    close(fd);

    exit(0);
}
/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test(bool external_bus)
{
    const char *path_accel = external_bus ? ADI_DEVICE_PATH_ACCEL_EXT : ADI_DEVICE_PATH_ACCEL;
    const char *path_gyro  = external_bus ? ADI_DEVICE_PATH_GYRO_EXT : ADI_DEVICE_PATH_GYRO;
    accel_report a_report;
    gyro_report g_report;
    ssize_t sz;

    /* get the driver */
    int fd = open(path_accel, O_RDONLY);warnx("accel_fd: %d ",fd);

    if (fd < 0) {
        err(1, "%s open failed1 (try 'm start')", path_accel);
    }

    /* get the driver */
    int fd_gyro = open(path_gyro, O_RDONLY);warnx("gyro_fd: %d ",fd_gyro);

    if (fd_gyro < 0) {
        err(1, "%s open failed2", path_gyro);
    }

    /* reset to manual polling */
    if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MANUAL) < 0) {
        err(1, "reset to manual polling");
    }

    /* do a simple demand read */
    sz = read(fd, &a_report, sizeof(a_report));

    if (sz != sizeof(a_report)) {
        warnx("ret: %d, expected: %d", sz, sizeof(a_report));
        err(1, "immediate acc read failed");
    }

    warnx("*ACC read");
    warnx("time:     %lld", a_report.timestamp);
    warnx("acc  x:  \t%8.4f\tm/s^2", (double)a_report.x);
    warnx("acc  y:  \t%8.4f\tm/s^2", (double)a_report.y);
    warnx("acc  z:  \t%8.4f\tm/s^2", (double)a_report.z);
    warnx("acc  x:  \t%d\traw 0x%0x", (short)a_report.x_raw, (unsigned short)a_report.x_raw);
    warnx("acc  y:  \t%d\traw 0x%0x", (short)a_report.y_raw, (unsigned short)a_report.y_raw);
    warnx("acc  z:  \t%d\traw 0x%0x", (short)a_report.z_raw, (unsigned short)a_report.z_raw);
    warnx("acc range: %8.4f m/s^2 (%8.4f g)", (double)a_report.range_m_s2,
          (double)(a_report.range_m_s2 / ADIS16460_ONE_G));

    /* do a simple demand read */
    sz = read(fd_gyro, &g_report, sizeof(g_report));

    if (sz != sizeof(g_report)) {
        warnx("ret: %d, expected: %d", sz, sizeof(g_report));
        err(1, "immediate gyro read failed");
    }
    warnx("*GYRO read");
    warnx("gyro x: \t% 9.5f\trad/s", (double)g_report.x);
    warnx("gyro y: \t% 9.5f\trad/s", (double)g_report.y);
    warnx("gyro z: \t% 9.5f\trad/s", (double)g_report.z);
    warnx("gyro x: \t%d\traw", (int)g_report.x_raw);
    warnx("gyro y: \t%d\traw", (int)g_report.y_raw);
    warnx("gyro z: \t%d\traw", (int)g_report.z_raw);
    warnx("gyro range: %8.4f rad/s (%d deg/s)", (double)g_report.range_rad_s,
          (int)((g_report.range_rad_s / M_PI_F) * 180.0f + 0.5f));

    warnx("temp:  \t%8.4f\tdeg celsius", (double)a_report.temperature);
    warnx("temp:  \t%d\traw 0x%0x", (short)a_report.temperature_raw, (unsigned short)a_report.temperature_raw);

    /* reset to default polling */
    if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
        err(1, "reset to default polling");
    }

    close(fd);
    close(fd_gyro);

    /* XXX add poll-rate tests here too */

    reset(external_bus);
    errx(0, "PASS");
}
void
usage()
{
    warnx("missing command: try 'start', 'test',\n");
    warnx("options:");
    warnx("    -R rotation");
}

}
// namespace

int
adis16460_main(int argc, char *argv[])
{
    bool external_bus = true;
    int ch;
    enum Rotation rotation = ROTATION_NONE;

    /* jump over start/off/etc and look at options first */
    while ((ch = getopt(argc, argv, "XR:")) != EOF) {
        switch (ch) {
        case 'X':
            external_bus = true;
            break;

        case 'R':
            rotation = (enum Rotation)atoi(optarg);
            break;
        default:
            adis16460::usage();
            exit(0);
        }
    }
    const char *verb = argv[optind];

    /*
     * Start/load the driver.

     */
    if (!strcmp(verb, "start")) {
        adis16460::start(external_bus, rotation);
    }
    if (!strcmp(verb, "stop")) {
        adis16460::stop(external_bus);
    }
    /*
     * Test the driver/device.
     */
    if (!strcmp(verb, "test")) {
        adis16460::test(external_bus);
    }
    /*
     * Reset the driver.
     */
    if (!strcmp(verb, "reset")) {
        adis16460::reset(external_bus);
    }
    adis16460::usage();
    exit(1);
}
