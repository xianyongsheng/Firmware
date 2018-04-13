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

class ADIS16488;

#pragma pack(push, 1)
struct adis16488_mag_regs {
	uint8_t st1;
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t st2;
};
#pragma pack(pop)

/**
 * Helper class implementing the magnetometer driver node.
 */
class ADIS16488_mag : public device::CDev
{
public:
    ADIS16488_mag(ADIS16488 *parent, const char *path);
    ~ADIS16488_mag();

	virtual ssize_t read(struct file *filp, char *buffer, size_t buflen);
	virtual int ioctl(struct file *filp, int cmd, unsigned long arg);
	virtual int init();

	void read_block(uint8_t reg, uint8_t *val, uint8_t count);

protected:
    friend class ADIS16488;

    void measure(struct adis16488_mag_regs data);
	int self_test(void);

private:
    ADIS16488 *_parent;
	orb_advert_t _mag_topic;
	int _mag_orb_class_instance;
	int _mag_class_instance;
	bool _mag_reading_data;
	ringbuffer::RingBuffer *_mag_reports;
	struct mag_calibration_s _mag_scale;
	float _mag_range_scale;
	perf_counter_t _mag_reads;
	perf_counter_t _mag_errors;
	float _mag_asa_x;
	float _mag_asa_y;
	float _mag_asa_z;

	bool check_duplicate(uint8_t *mag_data);

	// keep last mag reading for duplicate detection
	uint8_t			_last_mag_data[6];

	/* do not allow to copy this class due to pointer data members */
    ADIS16488_mag(const ADIS16488_mag &);
    ADIS16488_mag operator=(const ADIS16488_mag &);
};
