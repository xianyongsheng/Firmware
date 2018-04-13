#ifdef PX4_SPI_BUS_EXT
#define EXTERNAL_BUS PX4_SPI_BUS_EXT
#else
#define EXTERNAL_BUS PX4_SPI_BUS_EXT
#endif

//#include "mag.h"

#define ADIS16488_BUS_SPEED				10*1000*1000

#define ADIS16488_ONE_G					9.80665f

#define ADIREG_WHOAMI   0x7E00
#define ADI16488_WHOAMI 0x4068
// ADIS16488A registers

#define PROD_ID 		0x7E00
#define TEMP_OUT    0x0E00
#define X_GYRO_LOW	0x1000
#define X_GYRO_OUT	0x1200
#define Y_GYRO_LOW	0x1400
#define Y_GYRO_OUT	0x1600
#define Z_GYRO_LOW	0x1800
#define Z_GYRO_OUT	0x1A00
#define X_ACCL_LOW	0x1C00
#define X_ACCL_OUT	0x1E00
#define Y_ACCL_LOW	0x2000
#define Y_ACCL_OUT	0x2200
#define Z_ACCL_LOW	0x2400
#define Z_ACCL_OUT	0x2600
#define X_MAGN_OUT	0x2800
#define Y_MAGN_OUT	0x2A00
#define Z_MAGN_OUT	0x2C00
#define BAROM_LOW   0x2E00
#define BAROM_OUT   0x3000


class ADIS16488_mag;
class ADIS16488_gyro;
class ADIS16488_baro;

class ADIS16488 : public device::SPI
{
public:
    ADIS16488(int bus, const char *path_accel, const char *path_gyro, const char *path_mag, const char *path_baro, spi_dev_e device,
		enum Rotation rotation);
	virtual ~ADIS16488();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

	void			print_registers();

	// deliberately cause a sensor error
    void 			test_error();

protected:
	virtual int		probe();

    friend class ADIS16488_mag;
    friend class ADIS16488_gyro;
    friend class ADIS16488_baro;

	virtual ssize_t		gyro_read(struct file *filp, char *buffer, size_t buflen);
	virtual int		gyro_ioctl(struct file *filp, int cmd, unsigned long arg);

private:
    ADIS16488_gyro	*_gyro;
    ADIS16488_mag     *_mag;
    ADIS16488_baro     *_baro;
    uint16_t			_whoami;	/** whoami result */

	struct hrt_call		_call;
	unsigned		_call_interval;

	ringbuffer::RingBuffer	*_accel_reports;

	struct accel_calibration_s	_accel_scale;
	float			_accel_range_scale;
	float			_accel_range_m_s2;
	orb_advert_t		_accel_topic;
	int			_accel_orb_class_instance;
	int			_accel_class_instance;

	ringbuffer::RingBuffer	*_gyro_reports;

	struct gyro_calibration_s	_gyro_scale;
	float			_gyro_range_scale;
	float			_gyro_range_rad_s;

	unsigned		_dlpf_freq;

	unsigned		_sample_rate;
	perf_counter_t		_accel_reads;
	perf_counter_t		_gyro_reads;
	perf_counter_t		_sample_perf;
	perf_counter_t		_bad_transfers;
	perf_counter_t		_bad_registers;
	perf_counter_t		_good_transfers;
	perf_counter_t		_reset_retries;
	perf_counter_t		_duplicates;
	perf_counter_t		_controller_latency_perf;

	uint8_t			_register_wait;
	uint64_t		_reset_wait;

	math::LowPassFilter2p	_accel_filter_x;
	math::LowPassFilter2p	_accel_filter_y;
	math::LowPassFilter2p	_accel_filter_z;
	math::LowPassFilter2p	_gyro_filter_x;
	math::LowPassFilter2p	_gyro_filter_y;
	math::LowPassFilter2p	_gyro_filter_z;

	Integrator		_accel_int;
	Integrator		_gyro_int;

	enum Rotation		_rotation;

	// this is used to support runtime checking of key
	// configuration registers to detect SPI bus errors and sensor
	// reset
#define ADIS16488_NUM_CHECKED_REGISTERS 3
    static const uint8_t	_checked_registers[ADIS16488_NUM_CHECKED_REGISTERS];
    uint8_t			_checked_values[ADIS16488_NUM_CHECKED_REGISTERS];
    uint8_t			_checked_bad[ADIS16488_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_next;

	// last temperature reading for print_info()
	float			_last_temperature;

	bool check_null_data(uint32_t *data, uint8_t size);
	bool check_duplicate(uint8_t *accel_data);
	// keep last accel reading for duplicate detection
	uint8_t			_last_accel_data[6];
	bool			_got_duplicate;

	/**
	 * Start automatic measurement.
	 */
	void			start();

	/**
	 * Stop automatic measurement.
	 */
	void			stop();

	/**
	 * Reset chip.
	 *
	 * Resets the chip and measurements ranges, but not scale and offset.
	 */
	int			reset();

	/**
	 * Static trampoline from the hrt_call context; because we don't have a
	 * generic hrt wrapper yet.
	 *
	 * Called by the HRT in interrupt context at the specified rate if
	 * automatic polling is enabled.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		measure_trampoline(void *arg);

	/**
	 * Fetch measurements from the sensor and update the report buffers.
	 */
	void			measure();

    /**
	 * Read a register from the mpu
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
    uint8_t			read_reg(unsigned reg, uint32_t speed = ADIS16488_BUS_SPEED);
    int16_t		read_reg16(unsigned reg);
    int32_t		read_reg32(unsigned reg1,unsigned reg2);
	/**
	 * Write a register in the mpu
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_reg(unsigned reg, uint8_t value);

	/**
	 * Modify a register in the mpu
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Write a register in the mpu, updating _checked_values
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_checked_reg(unsigned reg, uint8_t value);

	/**
	 * Set the mpu measurement range.
	 *
	 * @param max_g		The maximum G value the range must support.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			set_accel_range(unsigned max_g);

	/**
	 * Swap a 16-bit value read from the mpu to native byte order.
	 */
	uint16_t		swap16(uint16_t val) { return (val >> 8) | (val << 8);	}

	/**
	 * Get the internal / external state
	 *
	 * @return true if the sensor is not on the main MCU board
	 */
	bool			is_external() { return (_bus == EXTERNAL_BUS); }

	/**
	 * Measurement self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			self_test();

	/**
	 * Accel self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			accel_self_test();

	/**
	 * Gyro self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			gyro_self_test();

	/*
	  set low pass filter frequency
	 */
	void _set_dlpf_filter(uint16_t frequency_hz);

	/*
	  set sample rate (approximate) - 1kHz to 5Hz
	*/
	void _set_sample_rate(unsigned desired_sample_rate_hz);

	/*
	  check that key registers still have the right value
	 */
	void check_registers(void);

	/* do not allow to copy this class due to pointer data members */
	ADIS16488(const ADIS16488 &);
	ADIS16488 operator=(const ADIS16488 &);


};
