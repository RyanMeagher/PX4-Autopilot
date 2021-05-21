/****************************************************************************
 *
 *   Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
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
 * @file ms5837.cpp
 * Driver for the MS5837 barometric pressure sensor connected via I2C or SPI.
 */

#include "MS5837.hpp"

/*
 * MS5837 I2C address
 */
#define MS5837_ADDRESS				    0x76

/*
 * MS5837 internal constants and data structures
 */
#define ADDR_RESET_CMD				    0x1E	/* write to this address to reset chip */
#define ADDR_CMD_CONVERT_D1_OSR256		0x40	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR512		0x42	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR1024		0x44	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR2048		0x46	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR4096		0x48	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D2_OSR256		0x50	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR512		0x52	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR1024		0x54	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR2048		0x56	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR4096		0x58	/* write to this address to start temperature conversion */
#define ADDR_PROM_SETUP				0xA0	/* address of seven 2 bytes factory and calibration data */

/*
  use an OSR of 1024 to reduce the self-heating effect of the
  sensor. Information from MS tells us that some individual sensors
  are quite sensitive to this effect and that reducing the OSR can
  make a big difference
 */
#define ADDR_CMD_CONVERT_D1			ADDR_CMD_CONVERT_D1_OSR1024
#define ADDR_CMD_CONVERT_D2			ADDR_CMD_CONVERT_D2_OSR1024

/*
 * Maximum internal conversion time for OSR 1024 is 2.28 ms. We set an update
 * rate of 100Hz which is be very safe not to read the ADC before the
 * conversion finished
 */
#define MS5837_CONVERSION_INTERVAL	10000	/* microseconds */
#define MS5837_MEASUREMENT_RATIO	3	/* pressure measurements per temperature measurement */


/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { __typeof__(_x) _tmp = _x+1; if (_tmp >= _lim) _tmp = 0; _x = _tmp; } while(0)

/* helper macro for arithmetic - returns the square of the argument */
#define POW2(_x)		((_x) * (_x))

MS5837::MS5837(I2CSPIBusOption bus_option, const int bus, int bus_frequency):
	I2C(DRV_BARO_DEVTYPE_MS5837, MODULE_NAME, bus, MS5837_ADDRESS, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_px4_barometer(get_device_id()),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_measure_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": measure")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err"))
{
}

MS5837::~MS5837()
{
	// free perf counters
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);
}

int MS5837::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_ERR("init failed");
		return PX4_ERROR;
	}

	/* do a first measurement cycle to populate reports with valid data */
	_measure_phase = 0;

	while (true) {
		/* do temperature first */
		if (PX4_OK != measure()) {
			ret = -EIO;
			break;
		}

		px4_usleep(MS5837_CONVERSION_INTERVAL);

		if (PX4_OK != collect()) {
			ret = -EIO;
			break;
		}

		/* now do a pressure measurement */
		if (PX4_OK != measure()) {
			ret = -EIO;
			break;
		}

		px4_usleep(MS5837_CONVERSION_INTERVAL);

		if (PX4_OK != collect()) {
			ret = -EIO;
			break;
		}

// TODO FIX THESE FOR THE MS5837  MS5837 range is 0 to 30 BAR
		/* state machine will have generated a report, copy it out */
		//const sensor_baro_s &brp = _px4_barometer.get();

		//if (brp.pressure < 0.0f) {
		//	/* This is likely not this device, abort */
		//	ret = -EINVAL;
		//	break;

		//} else if (brp.pressure > 3000.0f) {
		//	/* This is likely not this device, abort */
		//	ret = -EINVAL;
		//	break;
		//}
//////////

		ret = PX4_OK;

		break;
	}

	if (ret == 0) {
		start();
	}

	return ret;
}

int MS5837::probe()
{
	_retries = 10;

	/* send reset command */
	if (PX4_OK != reset()) {
		return -EIO;
	}

	/* read PROM */
	if (PX4_OK == read_prom()) {
		/*
		 * Disable retries; we may enable them selectively in some cases,
		 * but the device gets confused if we retry some of the commands.
		 */
		_retries = 0;
		return PX4_OK;
	}

	return -EIO;
}

int MS5837::read_prom()
{
	bool all_zeros = true;
	uint8_t buf[2];

	/*
	 * Wait for PROM contents to be in the device (2.8 ms) in the case we are
	 * called immediately after reset.
	 */
	px4_usleep(3000);

	/* Read and convert PROM words (7 words per datasheet) */
	for (int i = 0; i < 7; i++) {
		uint8_t cmd = ADDR_PROM_SETUP + (i * 2);

		if (PX4_OK != transfer(&cmd, 1, &buf[0], 2)) {
			break;
		}

		_prom.c[i] = (buf[0] << 8) | buf[1];

		/* CRC will erronously pass if the PROM is all zeros */
		if (_prom.c[i] != 0) {
			all_zeros = false;
		}
	}

	if (all_zeros) {
		return -EIO;
	}

	/* calculate CRC and return success/failure accordingly */
	return crc4(&_prom.c[0]) ? PX4_OK : -EIO;
}


int MS5837::register_read(uint8_t reg, void *data, unsigned count)
{
	uint8_t cmd = reg;
	int ret = transfer(&cmd, 1, (uint8_t *)data, count);
	return ret == PX4_OK ? count : ret;
}

int MS5837::register_write(uint8_t reg, uint8_t data)
{
	uint8_t buf[2] = { reg, data};
	int ret = transfer(buf, sizeof(buf), nullptr, 0);
	return ret == PX4_OK ? 2 : ret;
}

void MS5837::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_measure_phase = 0;

	/* schedule a cycle to start things */
	ScheduleDelayed(MS5837_CONVERSION_INTERVAL);
}

int MS5837::reset()
{
	unsigned prev_retries = _retries;
	uint8_t	cmd = ADDR_RESET_CMD;
	int ret;

	_retries = 10;
	ret = transfer(&cmd, 1, nullptr, 0);
	_retries = prev_retries;

	return ret;
}

void MS5837::RunImpl()
{
	int ret;

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		ret = collect();

		if (ret != PX4_OK) {
			// TODO is this true for the MS5837??  This comment was for the MS5611
			if (ret == -6) {
				/*
				 * The ms5837 seems to regularly fail to respond to
				 * its address; this happens often enough that we'd rather not
				 * spam the console with a message for this.
				 */
			} else {
				//DEVICE_LOG("collection error %d", ret);
			}

			/* issue a reset command to the sensor */
			reset();
			/* reset the collection state machine and try again - we need
			 * to wait 2.8 ms after issuing the sensor reset command
			 * according to the MS5611 datasheet
			 */
			ScheduleDelayed(2800);
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;
	}

	/* measurement phase */
	ret = measure();

	if (ret != PX4_OK) {
		/* issue a reset command to the sensor */
		reset();
		/* reset the collection state machine and try again */
		start();
		return;
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(MS5837_CONVERSION_INTERVAL);
}

int MS5837::measure()
{
	perf_begin(_measure_perf);

	/*
	 * In phase zero, request temperature; in other phases, request pressure.
	 */
	unsigned addr = (_measure_phase == 0) ? ADDR_CMD_CONVERT_D2 : ADDR_CMD_CONVERT_D1;

	/*
	 * Send the command to begin measuring.
	 */
	int ret = measure(addr);

	if (PX4_OK != ret) {
		perf_count(_comms_errors);
	}

	_px4_barometer.set_error_count(perf_event_count(_comms_errors));

	perf_end(_measure_perf);

	return ret;
}

int MS5837::collect()
{
	uint32_t raw;

	perf_begin(_sample_perf);

	/* read the most recent measurement - read offset/size are hardcoded in the interface */
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	int ret = read(0, (void *)&raw, 0);

	if (ret < 0) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	/* handle a measurement */
	if (_measure_phase == 0) {

		/* Perform MS5837 Caculation */

		/* temperature offset (in ADC units) */
		int32_t dT = (int32_t)raw - ((int32_t)_prom.s.c5_reference_temp << 8);

		/* absolute temperature in centidegrees - note intermediate value is outside 32-bit range */
		int32_t TEMP = 2000 + (int32_t)(((int64_t)dT * _prom.s.c6_temp_coeff_temp) >> 23);

		/* base sensor scale/offset values */
		/* Perform MS5611 Caculation */
		_OFF  = ((int64_t)_prom.s.c2_pressure_offset << 16) + (((int64_t)_prom.s.c4_temp_coeff_pres_offset * dT) >> 7);
		_SENS = ((int64_t)_prom.s.c1_pressure_sens << 15) + (((int64_t)_prom.s.c3_temp_coeff_pres_sens * dT) >> 8);

		/* MS5837 temperature compensation */
		if (TEMP < 2000) {

			int32_t T2 = ((int64_t)3 * ((int64_t)POW2(dT))) >> 33;

			int64_t f = POW2((int64_t)TEMP - 2000);
			int64_t OFF2 = 3 * f >> 1;
			int64_t SENS2 = 5 * f >> 3;

			if (TEMP < -1500) {

				int64_t f2 = POW2(TEMP + 1500);
				OFF2 += 7 * f2;
				SENS2 += f2 >> 2;
			}

			TEMP -= T2;
			_OFF  -= OFF2;
			_SENS -= SENS2;
		} else {
			//int32_t T2 = 2 *((int64_t)POW2(dT) >> 37);

			int32_t T2 = ((int64_t)2 * (int64_t)POW2(dT)) >> 37;
			int64_t f = POW2((int64_t)TEMP - 2000);
			int64_t OFF2 = 1 * f >> 4;
			int64_t SENS2 = 0;

			TEMP -= T2;
			_OFF  -= OFF2;
			_SENS -= SENS2;
		}

		float temperature = TEMP / 100.0f;
		_px4_barometer.set_temperature(temperature);

	} else {
		/* pressure calculation, result in Pa */
		int32_t P = (((int64_t)(raw * _SENS) >> 21) - _OFF) >> 13;

		float pressure = P / 10.0f;		/* convert to millibar */

		_px4_barometer.update(timestamp_sample, pressure);
	}

	/* update the measurement state machine */
	INCREMENT(_measure_phase, MS5837_MEASUREMENT_RATIO + 1);

	perf_end(_sample_perf);

	return PX4_OK;
}

void MS5837::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);

	printf("device:         %s\n", "ms5837");
}

bool MS5837::crc4(uint16_t *n_prom)
{
	int16_t cnt;
	uint16_t n_rem;
	uint16_t crc_read;
	uint8_t n_bit;

	n_rem = 0x00;

	/* save the read in CRC */
	crc_read = (n_prom[0] & 0xf000) >> 12;
	n_prom[0] = (0x0fff & n_prom[0]);
	n_prom[7] = 0;

	for (cnt = 0; cnt < 16; cnt++) {
		/* uneven bytes */
		if (cnt & 1) {
			n_rem ^= (uint8_t)((n_prom[cnt >> 1]) & 0x00FF);
		} else {
			n_rem ^= (uint8_t)(n_prom[cnt >> 1] >> 8);
		}

		for (n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & 0x8000) {
				n_rem = (n_rem << 1) ^ 0x3000;
			} else {
				n_rem = (n_rem << 1);
			}
		}
	}


	/* final 4 bit remainder is CRC value */
	n_rem = (0x000F & (n_rem >> 12));

	/* Put the CRC back in the PROM data */
	n_prom[0] |= (crc_read << 12) ;

	/* return true if CRCs match */
	return (crc_read == n_rem);
}


int MS5837::read(unsigned offset, void *data, unsigned count)
{
	union _cvt {
		uint8_t	b[4];
		uint32_t w;
	} *cvt = (_cvt *)data;
	uint8_t buf[3];

	/* read the most recent measurement */
	uint8_t cmd = 0;
	int ret = transfer(&cmd, 1, &buf[0], 3);

	if (ret == PX4_OK) {
		/* fetch the raw value */
		cvt->b[0] = buf[2];
		cvt->b[1] = buf[1];
		cvt->b[2] = buf[0];
		cvt->b[3] = 0;
	}

	return ret;
}

int MS5837::measure(unsigned addr)
{
	/*
	 * Disable retries on this command; we can't know whether failure
	 * means the device did or did not see the command.
	 */
	_retries = 0;

	uint8_t cmd = addr;
	return transfer(&cmd, 1, nullptr, 0);
}
