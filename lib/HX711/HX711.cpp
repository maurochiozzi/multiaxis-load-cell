#include <Arduino.h>
#include <HX711.h>

Bridge::Bridge(byte dout, byte pd_sck, byte gain)
{
	begin(dout, pd_sck, gain);
}

Bridge::Bridge()
{
}

Bridge::~Bridge()
{
}

void Bridge::begin(byte dout, byte pd_sck, byte gain)
{
	PD_SCK = pd_sck;
	DOUT = dout;

	pinMode(PD_SCK, OUTPUT);
	pinMode(DOUT, INPUT);

	set_gain(gain);
}

bool Bridge::is_ready()
{
	return digitalRead(DOUT) == LOW;
}

void Bridge::set_gain(byte gain)
{
	switch (gain)
	{
	case 128: // channel A, gain factor 128
		GAIN = 1;
		break;
	case 64: // channel A, gain factor 64
		GAIN = 3;
		break;
	case 32: // channel B, gain factor 32
		GAIN = 2;
		break;
	}

	digitalWrite(PD_SCK, LOW);
	read();
}

long Bridge::read()
{
	// wait for the chip to become ready
	while (!is_ready())
	{
		// Will do nothing on Arduino but prevent resets of ESP8266 (Watchdog Issue)
		yield();
	}

	unsigned long value = 0;
	uint8_t data[3] = {0};
	uint8_t filler = 0x00;

	// pulse the clock pin 24 times to read the data
	data[2] = shiftIn(DOUT, PD_SCK, MSBFIRST);
	data[1] = shiftIn(DOUT, PD_SCK, MSBFIRST);
	data[0] = shiftIn(DOUT, PD_SCK, MSBFIRST);

	// set the channel and the gain factor for the next reading using the clock pin
	for (unsigned int i = 0; i < GAIN; i++)
	{
		digitalWrite(PD_SCK, HIGH);
		digitalWrite(PD_SCK, LOW);
	}

	// Replicate the most significant bit to pad out a 32-bit signed integer
	if (data[2] & 0x80)
	{
		filler = 0xFF;
	}
	else
	{
		filler = 0x00;
	}

	// Construct a 32-bit signed integer
	value = (static_cast<unsigned long>(filler) << 24 | static_cast<unsigned long>(data[2]) << 16 | static_cast<unsigned long>(data[1]) << 8 | static_cast<unsigned long>(data[0]));

	return static_cast<long>(value);
}

long Bridge::read_average(byte times)
{
	long sum = 0;
	for (byte i = 0; i < times; i++)
	{
		sum += read();
		yield();
	}
	return sum / times;
}

double Bridge::get_value(byte times)
{
	return read_average(times) - OFFSET;
}

float Bridge::get_units(byte times)
{
	return get_value(times) / SCALE;
}

void Bridge::tare(byte times)
{
	double sum = read_average(times);
	set_offset(sum);
}

void Bridge::set_scale(float scale)
{
	SCALE = scale;
}

float Bridge::get_scale()
{
	return SCALE;
}

void Bridge::set_offset(long offset)
{
	OFFSET = offset;
}

long Bridge::get_offset()
{
	return OFFSET;
}

void Bridge::power_down()
{
	digitalWrite(PD_SCK, LOW);
	digitalWrite(PD_SCK, HIGH);
}

void Bridge::power_up()
{
	digitalWrite(PD_SCK, LOW);
}
