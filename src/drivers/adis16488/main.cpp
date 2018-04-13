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
 * @file main.cpp
 *
 * Driver for the Invensense adis16488 connected via SPI.
 *
 * @authors Andrew Tridgell
 *          Robert Dickenson
 *
 * based on the adis16488 driver
 */

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <getopt.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/conversions.h>

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
#include "mag.h"
#include "gyro.h"
#include "adis16488.h"

#define ADI_DEVICE_PATH_ACCEL		"/dev/adis16488_accel"
#define ADI_DEVICE_PATH_GYRO		"/dev/adis16488_gyro"
#define ADI_DEVICE_PATH_MAG         "/dev/adis16488_mag"
#define ADI_DEVICE_PATH_BARO         "/dev/adis16488_baro"
#define ADI_DEVICE_PATH_ACCEL_EXT	"/dev/adis16488_accel_ext"
#define ADI_DEVICE_PATH_GYRO_EXT	"/dev/adis16488_gyro_ext"
#define ADI_DEVICE_PATH_MAG_EXT 	"/dev/adis16488_mag_ext"
#define ADI_DEVICE_PATH_BARO_EXT 	"/dev/adis16488_baro_ext"

/** driver 'main' command */
extern "C" { __EXPORT int adis16488_main(int argc, char *argv[]); }

/**
 * Local functions in support of the shell command.
 */
namespace adis16488
{

ADIS16488	*g_dev_int; // on internal bus
ADIS16488	*g_dev_ext; // on external bus

void	start(bool, enum Rotation);
void	stop(bool);
void	test(bool);
void	reset(bool);
void	info(bool);
void	regdump(bool);
void	testerror(bool);
void	usage();

/**
 * Start the driver.
 *
 * This function only returns if the driver is up and running
 * or failed to detect the sensor.
 */
void
start(bool external_bus, enum Rotation rotation)
{
    int fd;
    ADIS16488 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;
    const char *path_accel = external_bus ? ADI_DEVICE_PATH_ACCEL_EXT : ADI_DEVICE_PATH_ACCEL;
    const char *path_gyro  = external_bus ? ADI_DEVICE_PATH_GYRO_EXT : ADI_DEVICE_PATH_GYRO;
    const char *path_mag   = external_bus ? ADI_DEVICE_PATH_MAG_EXT : ADI_DEVICE_PATH_MAG;
    const char *path_baro   = external_bus ? ADI_DEVICE_PATH_BARO_EXT : ADI_DEVICE_PATH_BARO;

    if (*g_dev_ptr != nullptr)
        /* if already started, the still command succeeded */
    {
        errx(0, "already started");
    }

    /* create the driver */
    if (external_bus) {
#ifdef PX4_SPI_BUS_EXT
        *g_dev_ptr = new ADIS16488(PX4_SPI_BUS_EXT, path_accel, path_gyro, path_mag, path_baro, (spi_dev_e)PX4_SPIDEV_EXT0, rotation);
//        warnx("External SPI available %u %u\n",PX4_SPI_BUS_EXT,PX4_SPIDEV_EXT0);
#else
        errx(0, "External SPI not available");
#endif

    } else {
        *g_dev_ptr = new ADIS16488(PX4_SPI_BUS_SENSORS, path_accel, path_gyro, path_mag, path_baro, (spi_dev_e)PX4_SPIDEV_MPU, rotation);
    }

    if (*g_dev_ptr == nullptr) {
        goto fail;
    }

    if (OK != (*g_dev_ptr)->init()) {
        goto fail;
    }

    /* set the poll rate to default, starts automatic data collection */
    fd = open(path_accel, O_RDONLY);

    if (fd < 0) {
        goto fail;
    }

    if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
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
    ADIS16488 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

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
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test(bool external_bus)
{
    const char *path_accel = external_bus ? ADI_DEVICE_PATH_ACCEL_EXT : ADI_DEVICE_PATH_ACCEL;
    const char *path_gyro  = external_bus ? ADI_DEVICE_PATH_GYRO_EXT : ADI_DEVICE_PATH_GYRO;
    const char *path_mag   = external_bus ? ADI_DEVICE_PATH_MAG_EXT : ADI_DEVICE_PATH_MAG;
    const char *path_baro   = external_bus ? ADI_DEVICE_PATH_BARO_EXT : ADI_DEVICE_PATH_BARO;
    accel_report a_report;
    gyro_report g_report;
    mag_report m_report;
    baro_report b_report;
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

    /* get the driver */
    int fd_mag = open(path_mag, O_RDONLY);warnx("mag_fd: %d ",fd_mag);

    if (fd_mag < 0) {
        err(1, "%s open failed3", path_mag);
    }

    /* get the driver */
    int fd_baro = open(path_baro, O_RDONLY);warnx("baro_fd: %d ",fd_baro);

    if (fd_baro < 0) {
        err(1, "%s open failed4", path_baro);
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
          (double)(a_report.range_m_s2 / ADIS16488_ONE_G));

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

    /* do a simple demand read */
    sz = read(fd_mag, &m_report, sizeof(m_report));

    if (sz != sizeof(m_report)) {
        warnx("ret: %d, expected: %d", sz, sizeof(m_report));
        err(1, "immediate mag read failed");
    }
    warnx("*MAG read");
    warnx("mag x: \t% 9.5f\trad/s", (double)m_report.x);
    warnx("mag y: \t% 9.5f\trad/s", (double)m_report.y);
    warnx("mag z: \t% 9.5f\trad/s", (double)m_report.z);
    warnx("mag x: \t%d\traw", (int)m_report.x_raw);
    warnx("mag y: \t%d\traw", (int)m_report.y_raw);
    warnx("mag z: \t%d\traw", (int)m_report.z_raw);
    warnx("mag range: %8.4f Ga", (double)m_report.range_ga);
    warnx("mag temp:  %8.4f\tdeg celsius", (double)m_report.temperature);

    /* do a simple demand read */
    sz = read(fd_baro, &b_report, sizeof(b_report));

    if (sz != sizeof(b_report)) {
        warnx("ret: %d, expected: %d", sz, sizeof(b_report));
        err(1, "immediate baro read failed");
    }
    warnx("*BARO read");
    warnx("pressure:    %10.4f", (double)b_report.pressure);
    warnx("altitude:    %11.4f", (double)b_report.altitude);
    warnx("temperature: %8.4f", (double)b_report.temperature);
    warnx("time:        %lld", b_report.timestamp);

    /* reset to default polling */
    if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
        err(1, "reset to default polling");
    }

    close(fd);
    close(fd_gyro);
    close(fd_mag);
    close(fd_baro);

    /* XXX add poll-rate tests here too */

    reset(external_bus);
    errx(0, "PASS");
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
 * Print a little info about the driver.
 */
void
info(bool external_bus)
{
    ADIS16488 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

    if (*g_dev_ptr == nullptr) {
        errx(1, "driver not running");
    }

    (*g_dev_ptr)->print_info();

    exit(0);
}

/**
 * Dump the register information
 */
void
regdump(bool external_bus)
{
    ADIS16488 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

    if (*g_dev_ptr == nullptr) {
        errx(1, "driver not running");
    }

    (*g_dev_ptr)->print_registers();

    exit(0);
}

/**
 * deliberately produce an error to test recovery
 */
void
testerror(bool external_bus)
{
    ADIS16488 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

    if (*g_dev_ptr == nullptr) {
        errx(1, "driver not running");
    }

    (*g_dev_ptr)->test_error();

    exit(0);
}

void
usage()
{
    warnx("missing command: try 'start', 'info', 'test', 'stop',\n'reset', 'regdump', 'testerror'");
    warnx("options:");
    warnx("    -X    (external bus)");
    warnx("    -R rotation");
}

} // namespace

int
adis16488_main(int argc, char *argv[])
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
            adis16488::usage();
            exit(0);
        }
    }
    const char *verb = argv[optind];

    /*
     * Start/load the driver.
     */
    if (!strcmp(verb, "start")) {
        adis16488::start(external_bus, rotation);
    }
    if (!strcmp(verb, "stop")) {
        adis16488::stop(external_bus);
    }

    /*
     * Test the driver/device.
     */
    if (!strcmp(verb, "test")) {
        adis16488::test(external_bus);
    }
    /*
     * Reset the driver.
     */
    if (!strcmp(verb, "reset")) {
        adis16488::reset(external_bus);
    }

    /*
     * Print driver information.
     */
    if (!strcmp(verb, "info")) {
        adis16488::info(external_bus);
    }

    /*
     * Print register information.
     */
    if (!strcmp(verb, "regdump")) {
        adis16488::regdump(external_bus);
    }

    if (!strcmp(verb, "testerror")) {
        adis16488::testerror(external_bus);
    }

    adis16488::usage();
    exit(1);
}
