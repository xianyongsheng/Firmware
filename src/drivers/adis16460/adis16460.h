#ifdef PX4_SPI_BUS_EXT
#define EXTERNAL_BUS PX4_SPI_BUS_EXT
#else
#define EXTERNAL_BUS PX4_SPI_BUS_EXT
#endif

class ADIS16460_gyro;

class ADIS16460 : public device::SPI
{
public:
    ADIS16460(int bus, const char *path_accel, const char *path_gyro, spi_dev_e device,
        enum Rotation rotation);
    virtual ~ADIS16460();

    virtual int		init();
    virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
    virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

protected:
    virtual int		probe();

    friend class ADIS16460_gyro;

    virtual ssize_t		gyro_read(struct file *filp, char *buffer, size_t buflen);
    virtual int		gyro_ioctl(struct file *filp, int cmd, unsigned long arg);

private:
    ADIS16460_gyro	*_gyro;
    int16_t			_product;	/** whoami result */

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

    unsigned		    _sample_rate;
    perf_counter_t		_accel_reads;
    perf_counter_t		_gyro_reads;
    perf_counter_t		_sample_perf;
    perf_counter_t		_bad_transfers;

    math::LowPassFilter2p	_accel_filter_x;
    math::LowPassFilter2p	_accel_filter_y;
    math::LowPassFilter2p	_accel_filter_z;
    math::LowPassFilter2p	_gyro_filter_x;
    math::LowPassFilter2p	_gyro_filter_y;
    math::LowPassFilter2p	_gyro_filter_z;

    Integrator		_accel_int;
    Integrator		_gyro_int;

    enum Rotation		_rotation;

#pragma pack(push, 1)
    /**
     * Report conversation with in the ADIS16448, including command byte and interrupt status.
     */
    struct ADISReport {
        uint32_t		gyro_x;
        uint32_t		gyro_y;
        uint32_t		gyro_z;
        uint32_t		accel_x;
        uint32_t		accel_y;
        uint32_t		accel_z;
        uint16_t		temp;
    };
#pragma pack(pop)

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
    int measure();

    /**
     * Read a register from the mpu
     *
     * @param		The register to read.
     * @return		The value that was read.
     */
    uint16_t		read_reg16(unsigned reg);
    /**
     * Write a register in the mpu
     *
     * @param reg		The register to write.
     * @param value		The new value to write.
     */
    void		write_reg16(unsigned reg, uint16_t value);

    /**
     * Modify a register in the mpu
     *
     * Bits are cleared before bits are set.
     *
     * @param reg		The register to modify.
     * @param clearbits	Bits in the register to clear.
     * @param setbits	Bits in the register to set.
     */
    void			modify_reg16(unsigned reg, uint16_t clearbits, uint16_t setbits);

    int        adi_read_reg(ADISReport *recv);
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


    /* do not allow to copy this class due to pointer data members */
    ADIS16460(const ADIS16460 &);
    ADIS16460 operator=(const ADIS16460 &);
};
/**
 * Helper class implementing the gyro driver node.
 */
class ADIS16460_gyro : public device::CDev
{
public:
    ADIS16460_gyro(ADIS16460 *parent, const char *path);
    virtual ~ADIS16460_gyro();

    virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
    virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

    virtual int		init();

protected:
    friend class ADIS16460;

    void			parent_poll_notify();
private:
    ADIS16460			*_parent;
    orb_advert_t		_gyro_topic;
    int					_gyro_orb_class_instance;
    int					_gyro_class_instance;

    /* do not allow to copy this class due to pointer data members */
    ADIS16460_gyro(const ADIS16460_gyro &);
    ADIS16460_gyro operator=(const ADIS16460_gyro &);

};
