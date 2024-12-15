/**************************************************************************************
The MIT License (MIT)

Copyright (c) 2021 CNDN Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
**************************************************************************************/

#pragma once
#ifndef __CNDN_HPP__
#define __CNDN_HPP__

#define VALDEFINE(str) #str
#define STRDEFINE(str) VALDEFINE(str)

#include <Arduino.h>
#include <stdarg.h>
#if ARDUINO > 100
#include <Wire.h>
#endif

#ifndef lowByte
#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))
#endif

#ifndef bitRead
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#endif

#ifndef bitRead
#define bitRead(value, bit) ((value) |= (1UL << (bit)))
#endif

#ifndef bitClear
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#endif

#ifndef bitToggle
#define bitToggle(value, bit) ((value) ^= (1UL << (bit)))
#endif

#ifndef bitWrite
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
#endif

#ifndef bit
#define bit(b) (1UL << (b))
#endif

class CNTimeout
{
public:
	static uint32_t& lastms() {
		static uint32_t s_ss = 0;
		return s_ss;
	}
	static CNTimeout& DISABLE()
	{
		static CNTimeout s_dis(CNTimeout().disable());
		return s_dis;
	}

	static uint32_t update() {
		lastms() = millis();
		return lastms();
	}
private:
	uint32_t  m_sta = 0;
	uint32_t	m_bit = 1;

public:
	CNTimeout& disable() {
		bitClear(m_bit, 0);
		return *this;
	}

	bool isTimeout(uint32_t now, int32_t out) {
		if (isEnable() && ((m_sta + out) <= now)) {
			m_sta = now;
			return true;
		}
		return false;
	}

	void reset(uint32_t now) {
		bitSet(m_bit, 0);
		m_sta = now;
	}

	inline bool isEnable() { return bitRead(m_bit,0); }
	bool toggle(uint8_t bit) { bitToggle(m_bit,bit); return bitRead(m_bit,bit); }
	bool isset(uint8_t bit) { return bitRead(m_bit,bit); }
	bool bitset(uint8_t bit) { return bitWrite(m_bit,bit, 1); }
	bool bitclr(uint8_t bit) { return bitWrite(m_bit,bit, 0); }

	inline uint32_t ellipsed(uint32_t _now) { return _now - m_sta; }
};

template<uint8_t numIOs>
class CNDigital {
public:
	struct CNPIN {
		int8_t  	_pin;			// 핀번호
		uint8_t 	_low:1;		// Low Active flag
		uint8_t 	_bON:1;		// Button on flag
		uint8_t 	_bO1:1;		// Button On Once flag
		uint8_t 	_bO2:1;		// Button On Timeout flag
		uint8_t 	_bO3:1;		// Button On Timeout once flag
		uint8_t 	_bF1:1;		// Button Off Once flag
		uint8_t 	_bF2:1;		// Button Off Timeout flag
		uint8_t 	_bF3:1;		// Button Off Timeout once flag
		uint16_t	_tO1;			// Button On Timeout time (x 10ms)
		uint16_t	_tF1;			// Button Off Timeout time (x 10ms)
		uint32_t	_tO0;			// Button On time
		uint32_t 	_tF0;			// Button Off Time
		uint32_t 	_tC0;			// Button Click Times
	};
private:
	CNPIN		pins[numIOs];
	CNDigital() {
		memset(pins, 0, sizeof(pins));
		for(int i=0; i < numIOs; i++) {
			CNPIN& p = pins[i];
			p._pin = -1;
			p._tF0 = 1;
			p._bF1 = p._bF2 = p._bF3 = 1;
		}
	}

public:
	static CNDigital& get() {
		static CNDigital s_instance;
		return s_instance;
	}

	void update(uint32_t now) {
		for(int i=0; i < numIOs; i++) {
			CNPIN& p = pins[i];
			if (p._pin == -1) continue;
			bool bRD = digitalRead(p._pin) != LOW;
			if (p._low) bRD = !bRD;
			if (bRD) {
				if (p._tO0 == 0) {
					p._tO0 = now;
					p._bO1 = 1;
				} else {
					p._bO1 = 0;
					if ((p._tO0+p._tO1*10) < now) {
						p._bF2 = 0;
						p._bF3 = 0;
						if (p._bO2 == 0) {
							if (++p._tC0 == 0) p._tC0++;
							p._bO3 = 1;
						} else {
							p._bO3 = 0;
						}
						p._bO2 = 1;
					}
				}
				p._bF3 = 0;
				p._bF2 = 0;
				p._bF1 = 0;
				p._tF0 = 0;
			} else {
				if (p._tF0 == 0) {
					p._tF0 = now;
					p._bF1 = 1;
				} else {
					p._bF1 = 0;
					if ((p._tF0+p._tF1*10) < now) {
						p._bO2 = 0;
						p._bO3 = 0;
						if (p._bF2 == 0) {
							p._bF3 = 1;
						} else {
							p._bF3 = 0;
						}
						p._bF2 = 1;
					}
				}
				p._bO3 = 0;
				p._bO2 = 0;
				p._bO1 = 0;
				p._tO0 = 0;
			}
			p._bON = bRD ? 1 : 0;
		}
	}

	void set(uint8_t idx, int8_t pin, bool lowActive, uint16_t ontime = 10, uint16_t offtime = 10)
	{
		if (idx >= numIOs) return;
		CNPIN& p = pins[idx];
		p._pin = pin;
		p._low = lowActive ? 1 : 0;
		p._tO1 = ontime;
		p._tF1 = offtime;
		if (p._pin != -1) {
			pinMode(p._pin, p._low?INPUT_PULLUP:INPUT);
		}
	}

	uint32_t count(uint8_t idx) {
		if (idx >= numIOs) return false;
		CNPIN& p = pins[idx];
		return p._tC0;
	}

	bool isPress(uint8_t idx) {
		if (idx >= numIOs) return false;
		CNPIN& p = pins[idx];
		return p._bON;
	}

	bool isDownOnce(uint8_t idx) {
		if (idx >= numIOs) return false;
		CNPIN& p = pins[idx];
		return p._bO1;
	}

	bool isOn(uint8_t idx) {
		if (idx >= numIOs) return false;
		CNPIN& p = pins[idx];
		return p._bO2;
	}

	bool isOnOnce(uint8_t idx) {
		if (idx >= numIOs) return false;
		CNPIN& p = pins[idx];
		return p._bO3;
	}

	bool isUpOnce(uint8_t idx) {
		if (idx >= numIOs) return false;
		CNPIN& p = pins[idx];
		return p._tC0 && p._bF1;
	}

	bool isOff(uint8_t idx) {
		if (idx >= numIOs) return false;
		CNPIN& p = pins[idx];
		return p._tC0 && p._bF2;
	}

	bool isOffOnce(uint8_t idx) {
		if (idx >= numIOs) return false;
		CNPIN& p = pins[idx];
		return p._tC0 && p._bF3;
	}

	bool isOnTime(uint8_t idx, uint32_t now, uint32_t tout) {
		if (idx >= numIOs) return false;
		CNPIN& p = pins[idx];
		return p._bO1 && (p._tO0+tout) < now;
	}

	bool isOffTime(uint8_t idx, uint32_t now, uint32_t tout) {
		if (idx >= numIOs) return false;
		CNPIN& p = pins[idx];
		return p._tC0 && p._bF1 && (p._tF0+tout) < now;
	}
};

class CNNTC {
private:
	int 	norminalRegister;
	int 	nCoefficient;
public:
	CNNTC(int norm=10000, int coef=3950): norminalRegister(norm), nCoefficient(coef) {}
	float temp(int registerValue) {
		float t = registerValue * 1.0f / norminalRegister;
		t = log(t);
		t = t / nCoefficient;
		t += 1.0 / (25 + 273.15);
		t = 1 / t;
		t -= 273.15;
		return t;
	}
};

class CNLineParser {
protected:
	String  data;
	String  cmds;
public:
	bool readline(char c, const char _dlim='\n', uint16_t sz=128) {
		if (c == _dlim) {
			cmds = data;
			clear();
			return true;
		} else if (data.length() < sz) {
			data.concat(c);
		}
		return false;
	}

	const String& getCommand() { return cmds; }

	void clear() {
		if (data.length()>0)
			data.remove(0,data.length());
	}

	void clearCommand() {
		if (cmds.length()>0)
			cmds.remove(0,cmds.length());
	}

	virtual bool process(Stream& strm) = 0;
};

#define START_POS(X,Y)	int npos = -1; int cpos = (X).indexOf(Y, npos+1); if (cpos == -1) break; npos = cpos;
#define	GET_ARG(X)	(X).substring(cpos+1)
#define NEXT_POS(X,Y)		cpos = (X).indexOf(Y, npos + 1); if (cpos == -1) break; npos = cpos;

class CNUtils {
public:	
	static void scanWire(TwoWire& wire, Stream& strm, uint8_t* en=nullptr) {
		for(uint8_t i=1; i < 127; i++) {
			wire.beginTransmission(i);
			if (wire.endTransmission() == 0x00) {
#if ARDUINO_ARCH_AVR
				strm.print("I2C found: 0x");
				strm.print(i, HEX);
				strm.print("\n");
#else
				strm.printf("I2C found: 0x%02X(%d)\n", i, i);
#endif
				if (en) en[i] = 1;
			} else {
				if (en) en[i] = 0;
			}
		}
	}

	static uint16_t CRC16(uint16_t crc, const void *ptr, size_t len) {
		const uint8_t *crcdata = (const uint8_t *)ptr;
		for(size_t i=0;i<len;i++){
			crc = crc ^ ((*crcdata)<<8);
			crcdata ++;
			for(uint8_t j=0;j<8;j++){
				if(crc & 0x8000){
					crc = (crc<<1) ^ 0x1021;
				}else{
					crc = crc<<1;
				}
			}
		}
		return crc;
	}

	template<int _size>
  static String print_v(const char *form, va_list args) {
    char buff[_size];
    buff[0] = 0;
    int size = vsnprintf(buff, _size, form, args);
    if (size>=0) buff[size] = 0;
    return buff;
  }

  static String print_r(const char *form, ...) {
    va_list args; va_start(args, form);
		return print_v<128>(form, args);
  }

	static void pulsePin(uint8_t _pin, uint32_t onTime=100000, uint32_t offTime=100000) {
		digitalWrite(_pin, HIGH);
		if (onTime>1000) delay(onTime/1000);
		delayMicroseconds(onTime%1000);
		digitalWrite(_pin, LOW);
		if (offTime>1000) delay(offTime/1000);
		delayMicroseconds(offTime%1000);
	}
};

class CNStream : public Stream
{
private:
	uint8_t		*mem;
	size_t 		tx, rx, ws;
	size_t 		memsize;
public:
	CNStream(size_t _size): tx(0), rx(0), ws(0) {
		mem = (uint8_t*)malloc(memsize=_size);
	}

	~CNStream() {
		if (mem != nullptr) {
			free(mem);
			mem = nullptr;
		}
	}

	virtual int available() { return ws; }
	virtual int availableForWrite() { return memsize-available(); }

	virtual int read() {
		if (available()>0) {
			size_t pos = rx;
			rx = (rx+1) % memsize;
			ws --;
			return mem[pos];
		}
		return -1;
	}

	virtual int peek() {
		if (available()>0) {
			return mem[rx];
		}
		return -1;
	}

	virtual size_t write(uint8_t dat) {
		if (availableForWrite() == 0) return 0;
		mem[tx] = dat;
		tx = (tx+1) % memsize;
		ws ++;
		return 1;
	}

	virtual void flush() {
		tx = rx = ws = 0;
	}
};

class RTStream : public Stream
{
private:
	CNStream	rxs;
public:
	CNStream	txs;
	RTStream(size_t txSize, size_t rxSize) : rxs(rxSize), txs(txSize) {}

	size_t recvice(uint8_t dat) { return rxs.write(dat); }

	virtual int available() { return rxs.available(); }
	virtual int read() { return rxs.read(); }
	virtual int peek() { return rxs.peek(); }
	virtual size_t write(uint8_t dat) { return txs.write(dat); }
	virtual void flush() {}
};

#define	CN_UNUSED(_X)		(void(_X))
#define	CNUNUSED(_X)		((_X)=(_X))

struct t3param {
	double	_DA;
	double	_DB;
	double	_DC;
	double	_DD;
	t3param(double _a,double _b,double _c,double _d):_DA(_a),_DB(_b),_DC(_c),_DD(_d){}
};

inline double t3curve(const t3param& v,double x) {
		double lnx = log(x);
		double y = 1.0 / (v._DA + v._DB * pow(lnx,1) + v._DC * pow(lnx,2) + v._DD * pow(lnx,3));
		return y;
}


#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Stream.h>

#if (USE_RTU_LOG)
#define RTU_LOG(...)             \
	{                              \
		Serial.print("[");           \
		Serial.print(__FUNCTION__);  \
		Serial.print("(): ");        \
		Serial.print(__LINE__);      \
		Serial.print(" ] ");         \
		Serial.println(__VA_ARGS__); \
	}
#else
#define RTU_LOG(...)
#endif

#ifndef RTU_BROADCAST_ADDRESS
#define RTU_BROADCAST_ADDRESS 0x00
#endif

class CNRTU
{
protected:
	typedef struct
	{
		uint16_t len;
		uint8_t id;
		uint8_t cmd;
		uint8_t payload[0];
		uint16_t cs;
	} __attribute__((packed)) tagRTUHeader, *pRTUHeader;

	typedef enum
	{
		eERROR_ILLEGAL_FUNCTION = 0x01,
		eERROR_ILLEGAL_ADDRESS,
		eERROR_ILLEGAL_VALUE,
		eERROR_RTU_CRC = 0x08,
		eERROR_RTU_RECV,
		eERROR_MEMORY,
		eERROR_RTU_ID
	} ENUM_RTU_ERRORS;

	typedef enum
	{
		eCMD_READ_COILS 					= 0x01,
		eCMD_READ_DISCRETE 				= 0x02,
		eCMD_READ_HOLDING 				= 0x03,
		eCMD_READ_INPUT 					= 0x04,
		eCMD_WRITE_COILS 					= 0x05,
		eCMD_WRITE_HOLDING 				= 0x06,
		eCMD_WRITE_MULTI_COILS 		= 0x0F,
		eCMD_WRITE_MULTI_HOLDING 	= 0x10
	} ENUM_RTU_COMMAND;

	void clearRecvBuffer()
	{
		while (_s->available())
		{
			_s->read();
			delay(2);
		}
	}

	uint16_t calculateCRC(uint8_t *data, uint8_t len)
	{
		uint16_t crc = 0xFFFF;
		for (uint8_t pos = 0; pos < len; pos++)
		{
			crc ^= (uint16_t)data[pos];
			for (uint8_t i = 8; i != 0; i--)
			{
				if ((crc & 0x0001) != 0)
				{
					crc >>= 1;
					crc ^= 0xA001;
				}
				else
				{
					crc >>= 1;
				}
			}
		}
		crc = ((crc & 0x00FF) << 8) | ((crc & 0xFF00) >> 8);
		return crc;
	}

	pRTUHeader packed(uint8_t id, ENUM_RTU_COMMAND cmd, void *data, uint16_t size)
	{
		return packed(id, (uint8_t)cmd, data, size);
	}

	pRTUHeader packed(uint8_t id, uint8_t cmd, void *data, uint16_t size)
	{
		pRTUHeader header = NULL;
		uint16_t crc = 0;
		if ((data == NULL) || (size == 0))
			return NULL;
		if ((header = (pRTUHeader)malloc(sizeof(tagRTUHeader) + size)) == NULL)
		{
			RTU_LOG("Memory ERROR");
			return NULL;
		}
		header->len = sizeof(tagRTUHeader) + size - 2;
		header->id = id;
		header->cmd = cmd;
		memcpy(header->payload, data, size);
		crc = calculateCRC((uint8_t *)&(header->id), (header->len) - 2);
		header->payload[size] = (crc >> 8) & 0xFF;
		header->payload[size + 1] = crc & 0xFF;
		return header;
	}

	void sendPackage(pRTUHeader header)
	{
		clearRecvBuffer();
		if (header != NULL)
		{
			if (_dePin > 0)
			{
				digitalWrite(_dePin, HIGH);
				delayMicroseconds(50);
			}
			_s->write((uint8_t *)&(header->id), header->len);
			_s->flush();

			free(header);
			if (_dePin > 0)
			{
				// delayMicroseconds(50);
				digitalWrite(_dePin, LOW);
			}
		}
	}

	pRTUHeader recvAndParsePackage(uint8_t id, uint8_t cmd, uint16_t data, uint8_t *error)
	{
		if (id > 0xF7)
			return NULL;
		if (id == 0)
		{
			if (error != NULL)
				*error = 0;
			return NULL;
		}

		uint8_t head[4] = {0, 0, 0, 0};
		uint16_t crc = 0;
		pRTUHeader header = NULL;

LOOP:
		uint16_t remain, index = 0;
		uint32_t time = millis();
		for (int i = 0; i < 4;)
		{
			if (_s->available())
			{
				head[index++] = (uint8_t)_s->read();
				RTU_LOG(head[index - 1], HEX);
				if ((index == 1) && (head[0] != id))
				{
					index = 0;
				}
				else if ((index == 2) && ((head[1] & 0x7F) != cmd))
				{
					index = 0;
				}
				i = index;
				time = millis();
			}
			if ((millis() - time) > _timeout)
			{
				RTU_LOG("ERROR");
				RTU_LOG(millis() - time);
				break;
			}
		}

		if (index != 4)
		{
			RTU_LOG();
			if (error != NULL)
				*error = eERROR_RTU_RECV;
			return NULL;
		}
		switch (head[1])
		{
		case eCMD_READ_COILS:
		case eCMD_READ_DISCRETE:
		case eCMD_READ_HOLDING:
		case eCMD_READ_INPUT:
			if (head[2] != (data & 0xFF))
			{
				index = 0;
				goto LOOP;
			}
			index = 5 + head[2];
			break;
		case eCMD_WRITE_COILS:
		case eCMD_WRITE_HOLDING:
		case eCMD_WRITE_MULTI_COILS:
		case eCMD_WRITE_MULTI_HOLDING:
			if (((head[2] << 8) | (head[3])) != data)
			{
				index = 0;
				goto LOOP;
			}
			index = 8;
			break;
		default:
			index = 5;
			break;
		}
		if ((header = (pRTUHeader)malloc(index + 2)) == NULL)
		{
			RTU_LOG("Memory ERROR");
			if (error != NULL)
				*error = eERROR_RTU_RECV;
			return NULL;
		}
		header->len = index;
		memcpy((uint8_t *)&(header->id), head, 4);
		remain = index - 4;
		index = 2;
		time = millis();
		while (remain)
		{
			RTU_LOG(_s->available());
			if (_s->available())
			{
				*(header->payload + index) = (uint8_t)_s->read();
				index++;
				time = millis();
				remain--;
			}
			if ((millis() - time) > _timeout)
			{
				free(header);
				if (error != NULL)
					*error = eERROR_RTU_RECV;
				RTU_LOG();
				return NULL;
			}
		}
		crc = (header->payload[(header->len) - 4] << 8) | header->payload[(header->len) - 3];

		if (crc != calculateCRC((uint8_t *)&(header->id), (header->len) - 2))
		{
			free(header);
			RTU_LOG("CRC ERROR");
			if (error != NULL)
				*error = eERROR_RTU_RECV;
			return NULL;
		}
		if (error != NULL)
			*error = 0;
		if (head[1] & 0x80)
		{
			*error = head[2];
		}

		return header;
	}

public:
	CNRTU(Stream *s, int dePin) : _timeout(100), _s(s), _dePin(dePin)
	{
		if (_dePin > 0)
		{
			pinMode(_dePin, OUTPUT);
		}
	}
	CNRTU(Stream *s) : _timeout(100), _s(s), _dePin(-1)
	{
		if (_dePin > 0)
		{
			pinMode(_dePin, OUTPUT);
		}
	}
	CNRTU()
			: _timeout(100), _s(NULL), _dePin(-1)
	{
		if (_dePin > 0)
		{
			pinMode(_dePin, OUTPUT);
		}
	}
	~CNRTU() {}

	void setTimeoutTimeMs(uint32_t timeout = 100)
	{
		_timeout = timeout;
	}

	/**
	 * @brief Read a coils Register.
	 * @param id:  device ID. (0~247), 0 is broadcast
	 * @param reg: Coils register address.
	 * @return Return the value of the coils register value.
	 * @n      true: The value of the coils register value is 1.
	 * @n      false: The value of the coils register value is 0.
	 */
	bool readCoilsRegister(uint8_t id, uint16_t reg)
	{
		uint8_t temp[] = {(uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF), 0x00, 0x01};
		bool val = false;
		uint8_t ret = 0;
		if ((id == 0) && (id > 0xF7))
		{
			RTU_LOG("Device ID error");
			return 0;
		}
		pRTUHeader header = packed(id, eCMD_READ_COILS, temp, sizeof(temp));
		sendPackage(header);
		header = recvAndParsePackage(id, (uint8_t)eCMD_READ_COILS, 1, &ret);
		if ((ret == 0) && (header != NULL))
		{
			if (header->payload[1] & 0x01)
				val = true;
			free(header);
		}
		RTU_LOG(val, HEX);
		return val;
	}

	/**
	 * @brief Read a discrete input register.
	 * @param id:  device ID. (0~247), 0 is broadcast
	 * @return Return the value of the coils register value.
	 * @n      true: The value of the coils register value is 1.
	 * @n      false: The value of the coils register value is 0.
	 */
	bool readDiscreteInputsRegister(uint8_t id, uint16_t reg)
	{
		uint8_t temp[] = {(uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF), 0x00, 0x01};
		bool val = false;
		uint8_t ret = 0;
		if ((id == 0) && (id > 0xF7))
		{
			RTU_LOG("Device ID error");
			return 0;
		}
		pRTUHeader header = packed(id, eCMD_READ_DISCRETE, temp, sizeof(temp));
		sendPackage(header);
		header = recvAndParsePackage(id, (uint8_t)eCMD_READ_DISCRETE, 1, &ret);
		if ((ret == 0) && (header != NULL))
		{
			if (header->payload[1] & 0x01)
				val = true;
			free(header);
		}
		RTU_LOG(val, HEX);
		return val;
	}
	/**
	 * @brief Read a holding Register.
	 * @param id:  device ID. (0~247), 0 is broadcast
	 * @param reg: Holding register address.
	 * @return Return the value of the holding register value.
	 */
	uint16_t readHoldingRegister(uint8_t id, uint16_t reg)
	{
		uint8_t temp[] = {(uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF), 0x00, 0x01};
		uint16_t val = 0;
		uint8_t ret = 0;
		if ((id == 0) && (id > 0xF7))
		{
			RTU_LOG("Device ID error");
			return 0;
		}
		pRTUHeader header = packed(id, eCMD_READ_HOLDING, temp, sizeof(temp));
		sendPackage(header);
		header = recvAndParsePackage(id, (uint8_t)eCMD_READ_HOLDING, 2, &ret);
		if ((ret == 0) && (header != NULL))
		{
			val = (header->payload[1] << 8) | header->payload[2];
			free(header);
		}
		// RTU_LOG(val, HEX);
		return val;
	}

	/**
	 * @brief Read a input Register.
	 * @param id:  device ID. (0~247), 0 is broadcast
	 * @param reg: input register address.
	 * @return Return the value of the input register value.
	 */
	uint16_t readInputRegister(uint8_t id, uint16_t reg)
	{
		uint8_t temp[] = {(uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF), 0x00, 0x01};
		uint16_t val = 0;
		uint8_t ret = 0;
		if ((id == 0) && (id > 0xF7))
		{
			RTU_LOG("Device ID error");
			return 0;
		}
		pRTUHeader header = packed(id, eCMD_READ_INPUT, temp, sizeof(temp));
		RTU_LOG(header->len, HEX);
		RTU_LOG(header->id, HEX);
		RTU_LOG(header->cmd, HEX);
		for (uint8_t i = 0; i < sizeof(temp) + 2; i++)
			RTU_LOG(header->payload[i], HEX);
		sendPackage(header);
		header = recvAndParsePackage(id, (uint8_t)eCMD_READ_INPUT, 2, &ret);
		if ((ret == 0) && (header != NULL))
		{
			val = (header->payload[1] << 8) | header->payload[2];
			free(header);
		}
		RTU_LOG(val, HEX);
		return val;
	}
	/**
	 * @brief Write a coils Register.
	 * @param id:  device ID. (0~247), 0 is broadcast
	 * @param reg: Coils register address.
	 * @param flag: The value of the register value which will be write, 0 ro 1.
	 * @return ENUM_RTU_ERRORS
	 */
	uint8_t writeCoilsRegister(uint8_t id, uint16_t reg, bool flag)
	{
		uint16_t val = flag ? 0xFF00 : 0x0000;
		uint8_t temp[] = {(uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF), (uint8_t)((val >> 8) & 0xFF), (uint8_t)(val & 0xFF)};
		uint8_t ret = 0;
		if (id > 0xF7)
		{
			RTU_LOG("Device ID error");
			return 0;
		}
		pRTUHeader header = packed(id, eCMD_WRITE_COILS, temp, sizeof(temp));
		sendPackage(header);
		header = recvAndParsePackage(id, (uint8_t)eCMD_WRITE_COILS, reg, &ret);
		if ((ret == 0) && (header != NULL))
		{
			free(header);
		}
		return ret;
	}
	/**
	 * @brief Write a holding register.
	 * @param id:  device ID. (0~247), 0 is broadcast
	 * @param reg: Holding register address.
	 * @param val: The value of the register value which will be write.
	 * @return ENUM_RTU_ERRORS
	 */
	uint8_t writeHoldingRegister(uint8_t id, uint16_t reg, uint16_t val)
	{
		uint8_t temp[] = {(uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF), (uint8_t)((val >> 8) & 0xFF), (uint8_t)(val & 0xFF)};
		uint8_t ret = 0;
		if (id > 0xF7)
		{
			RTU_LOG("Device ID error");
			return 0;
		}
		pRTUHeader header = packed(id, eCMD_WRITE_HOLDING, temp, sizeof(temp));
		uint8_t *data = NULL;
		data = (uint8_t *)malloc(sizeof(tagRTUHeader));
		memcpy(data, header, sizeof(tagRTUHeader));
		while (data != NULL)
		{
			RTU_LOG(*data, HEX);
			data++;
		}
		free(data);
		sendPackage(header);
		header = recvAndParsePackage(id, (uint8_t)eCMD_WRITE_HOLDING, reg, &ret);
		val = 0xFFFF;
		if ((ret == 0) && (header != NULL))
		{
			val = (header->payload[2] << 8) | header->payload[3];
			free(header);
		}
		// RTU_LOG(val, HEX);
		return ret;
	}

	/**
	 * @brief Read multiple coils Register.
	 * @param id:  device ID. (0~247), 0 is broadcast
	 * @param reg: Coils register address.
	 * @param regNum: Number of coils Register.
	 * @param data: Storage register worth pointer.
	 * @param size: Cache size of data.
	 * @return ENUM_RTU_ERRORS
	 */
	uint8_t readCoilsRegister(uint8_t id, uint16_t reg, uint16_t regNum, uint8_t *data, uint16_t size)
	{
		uint8_t length = regNum / 8 + ((regNum % 8) ? 1 : 0);
		uint8_t temp[] = {(uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF), (uint8_t)((regNum >> 8) & 0xFF), (uint8_t)(regNum & 0xFF)};
		uint8_t ret = 0;
		if ((id == 0) && (id > 0xF7))
		{
			RTU_LOG("Device ID error");
			return eERROR_RTU_ID;
		}
		pRTUHeader header = packed(id, eCMD_READ_COILS, temp, sizeof(temp));
		sendPackage(header);
		header = recvAndParsePackage(id, (uint8_t)eCMD_READ_COILS, length, &ret);
		if ((ret == 0) && (header != NULL))
		{
			if (data != NULL)
			{
				size = (size > length) ? length : size;
				memcpy(data, (uint8_t *)&(header->payload[1]), size);
			}
			free(header);
		}
		return ret;
	}
	/**
	 * @brief Read multiple discrete inputs register.
	 * @param id:  device ID. (0~247), 0 is broadcast
	 * @param reg: Discrete inputs register. address.
	 * @param regNum: Number of coils Register.
	 * @param data: Storage register worth pointer.
	 * @param size: Cache size of data.
	 * @return ENUM_RTU_ERRORS
	 */
	uint8_t readDiscreteInputsRegister(uint8_t id, uint16_t reg, uint16_t regNum, uint8_t *data, uint16_t size)
	{
		uint8_t length = regNum / 8 + ((regNum % 8) ? 1 : 0);
		uint8_t temp[] = {(uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF), (uint8_t)((regNum >> 8) & 0xFF), (uint8_t)(regNum & 0xFF)};
		uint8_t ret = 0;
		if ((id == 0) && (id > 0xF7))
		{
			RTU_LOG("Device ID error");
			return eERROR_RTU_ID;
		}
		pRTUHeader header = packed(id, eCMD_READ_DISCRETE, temp, sizeof(temp));
		sendPackage(header);
		header = recvAndParsePackage(id, (uint8_t)eCMD_READ_DISCRETE, length, &ret);
		if ((ret == 0) && (header != NULL))
		{
			if (data != NULL)
			{
				size = (size > length) ? length : size;
				memcpy(data, (uint8_t *)&(header->payload[1]), size);
			}
			free(header);
		}
		return ret;
	}
	/**
	 * @brief Read multiple Holding register.
	 * @param id:  device ID. (0~247), 0 is broadcast
	 * @param reg: Holding register.
	 * @param data: Storage register worth pointer.
	 * @param size: Cache size.
	 * @return ENUM_RTU_ERRORS
	 */
	uint8_t readHoldingRegister(uint8_t id, uint16_t reg, void *data, uint16_t size)
	{
		uint8_t length = size / 2 + ((size % 2) ? 1 : 0);
		uint8_t temp[] = {(uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF), (uint8_t)((length >> 8) & 0xFF), (uint8_t)(length & 0xFF)};
		uint8_t ret = 0;
		if ((id == 0) && (id > 0xF7))
		{
			RTU_LOG("Device ID error");
			return eERROR_RTU_ID;
		}
		pRTUHeader header = packed(id, eCMD_READ_HOLDING, temp, sizeof(temp));
		sendPackage(header);
		header = recvAndParsePackage(id, (uint8_t)eCMD_READ_HOLDING, length * 2, &ret);
		if ((ret == 0) && (header != NULL))
		{
			if (data != NULL)
				memcpy(data, (uint8_t *)&(header->payload[1]), size);
			free(header);
		}
		return ret;
	}

	/**
	 * @brief Read multiple Input register.
	 * @param id:  device ID. (0~247), 0 is broadcast
	 * @param reg: Input register.
	 * @param data: Storage register worth pointer.
	 * @param regNum: register numbers.
	 * @return ENUM_RTU_ERRORS
	 */
	uint8_t readInputRegister(uint8_t id, uint16_t reg, void *data, uint16_t size)
	{
		uint8_t length = size / 2 + ((size % 2) ? 1 : 0);
		uint8_t temp[] = {(uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF), (uint8_t)((length >> 8) & 0xFF), (uint8_t)(length & 0xFF)};
		uint8_t ret = 0;
		if ((id == 0) && (id > 0xF7))
		{
			RTU_LOG("Device ID error");
			return eERROR_RTU_ID;
		}
		pRTUHeader header = packed(id, eCMD_READ_INPUT, temp, sizeof(temp));
		sendPackage(header);
		header = recvAndParsePackage(id, (uint8_t)eCMD_READ_INPUT, length * 2, &ret);
		if ((ret == 0) && (header != NULL))
		{
			if (data != NULL)
				memcpy(data, (uint8_t *)&(header->payload[1]), size);
			free(header);
		}
		RTU_LOG(val, HEX);
		return ret;
	}

	/**
	 * @brief Read multiple Holding register.
	 * @param id:  device ID. (0~247), 0 is broadcast
	 * @param reg: Holding register.
	 * @param data: Storage register worth pointer.
	 * @param regNum: register numbers.
	 * @return ENUM_RTU_ERRORS
	 */
	uint8_t readHoldingRegister(uint8_t id, uint16_t reg, uint16_t *data, uint16_t regNum)
	{
		uint8_t temp[] = {(uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF), (uint8_t)((regNum >> 8) & 0xFF), (uint8_t)(regNum & 0xFF)};
		uint8_t ret = 0;
		if ((id == 0) && (id > 0xF7))
		{
			RTU_LOG("Device ID error");
			return eERROR_RTU_ID;
		}
		pRTUHeader header = packed(id, eCMD_READ_HOLDING, temp, sizeof(temp));
		sendPackage(header);
		header = recvAndParsePackage(id, (uint8_t)eCMD_READ_HOLDING, regNum * 2, &ret);
		if ((ret == 0) && (header != NULL))
		{
			if (data != NULL)
			{
				for (int i = 0; i < regNum; i++)
				{
					data[i] = ((header->payload[1 + 2 * i]) << 8) | (header->payload[2 + 2 * i]);
				}
			}
			free(header);
		}
		return ret;
	}

	/**
	 * @brief Read multiple Input register.
	 * @param id:  device ID. (0~247), 0 is broadcast
	 * @param reg: Input register.
	 * @param data: Storage register worth pointer.
	 * @param regNum: register numbers.
	 * @return ENUM_RTU_ERRORS
	 */
	uint8_t readInputRegister(uint8_t id, uint16_t reg, uint16_t *data, uint16_t regNum)
	{
		uint8_t temp[] = {(uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF), (uint8_t)((regNum >> 8) & 0xFF), (uint8_t)(regNum & 0xFF)};
		uint8_t ret = 0;
		if ((id == 0) && (id > 0xF7))
		{
			RTU_LOG("Device ID error");
			return eERROR_RTU_ID;
		}
		pRTUHeader header = packed(id, eCMD_READ_INPUT, temp, sizeof(temp));
		sendPackage(header);
		header = recvAndParsePackage(id, (uint8_t)eCMD_READ_INPUT, regNum * 2, &ret);
		if ((ret == 0) && (header != NULL))
		{
			if (data != NULL)
			{
				for (int i = 0; i < regNum; i++)
				{
					data[i] = ((header->payload[1 + 2 * i]) << 8) | (header->payload[2 + 2 * i]);
				}
			}
			free(header);
		}
		return ret;
	}

	/**
	 * @brief Write multiple coils Register.
	 * @param id:  device ID. (0~247), 0 is broadcast
	 * @param reg: Coils register address.
	 * @param regNum: Numbers of Coils register.
	 * @param data: Storage register worth pointer.
	 * @param size: Cache size.
	 * @return ENUM_RTU_ERRORS
	 */
	uint8_t writeCoilsRegister(uint8_t id, uint16_t reg, uint16_t regNum, uint8_t *data, uint16_t size)
	{
		uint16_t length = regNum / 8 + ((regNum % 8) ? 1 : 0);
		if (size < length)
			return (uint8_t)eERROR_ILLEGAL_VALUE;
		uint8_t temp[size + 5];
		temp[0] = (uint8_t)((reg >> 8) & 0xFF);
		temp[1] = (uint8_t)(reg & 0xFF);
		temp[2] = (uint8_t)((regNum >> 8) & 0xFF);
		temp[3] = (uint8_t)(regNum & 0xFF);
		temp[4] = (uint8_t)length;
		uint8_t ret = 0;
		if (id > 0xF7)
		{
			RTU_LOG("Device ID error");
			return (uint8_t)eERROR_RTU_ID;
		}
		memcpy(temp + 5, data, size);
		pRTUHeader header = packed(id, eCMD_WRITE_MULTI_COILS, temp, sizeof(temp));
		sendPackage(header);
		header = recvAndParsePackage(id, (uint8_t)eCMD_WRITE_MULTI_COILS, reg, &ret);
		size = 0;
		if ((ret == 0) && (header != NULL))
		{
			size = (header->payload[2] << 8) | header->payload[3];
			free(header);
		}
		return ret;
	}
	/**
	 * @brief Write multiple Holding Register.
	 * @param id:  device ID. (0~247), 0 is broadcast
	 * @param reg: Holding register address.
	 * @param data: Storage register worth pointer.
	 * @param size: Cache size.
	 * @return ENUM_RTU_ERRORS
	 */
	uint8_t writeHoldingRegister(uint8_t id, uint16_t reg, void *data, uint16_t size)
	{
		if (((size % 2) != 0) || (size > 250) || data == NULL)
			return (uint8_t)eERROR_ILLEGAL_VALUE;
		// #if defined(ESP8266)
		uint8_t temp[size + 5];
		temp[0] = (uint8_t)((reg >> 8) & 0xFF);
		temp[1] = (uint8_t)(reg & 0xFF);
		temp[2] = (uint8_t)(((size / 2) >> 8) & 0xFF);
		temp[3] = (uint8_t)((size / 2) & 0xFF);
		temp[4] = (uint8_t)size;
		// #else
		// uint8_t temp[size + 5] = {(uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF), (uint8_t)(((size/2) >> 8) & 0xFF), (uint8_t)((size/2) & 0xFF),(uint8_t)size};
		// #endif
		uint8_t ret = 0;
		if (id > 0xF7)
		{
			RTU_LOG("Device ID error");
			return (uint8_t)eERROR_RTU_ID;
		}
		memcpy(temp + 5, data, size);
		pRTUHeader header = packed(id, eCMD_WRITE_MULTI_HOLDING, temp, sizeof(temp));
		sendPackage(header);
		header = recvAndParsePackage(id, (uint8_t)eCMD_WRITE_MULTI_HOLDING, reg, &ret);
		size = 0;
		if ((ret == 0) && (header != NULL))
		{
			size = (header->payload[2] << 8) | header->payload[3];
			free(header);
		}
		return ret;
	}
	/**
	 * @brief Write multiple Holding Register.
	 * @param id:  device ID. (0~247), 0 is broadcast
	 * @param reg: Holding register address.
	 * @param data: Storage register worth pointer.
	 * @param regNum: Number of coils Register..
	 * @return ENUM_RTU_ERRORS
	 */
	uint8_t writeHoldingRegister(uint8_t id, uint16_t reg, uint16_t *data, uint16_t regNum)
	{
		uint16_t size = regNum * 2;
		uint8_t *pBuf = (uint8_t *)data;
		uint8_t temp[size + 5];
		temp[0] = (uint8_t)((reg >> 8) & 0xFF);
		temp[1] = (uint8_t)(reg & 0xFF);
		temp[2] = (uint8_t)(((size / 2) >> 8) & 0xFF);
		temp[3] = (uint8_t)((size / 2) & 0xFF);
		temp[4] = (uint8_t)size;
		uint8_t ret = 0;
		if (id > 0xF7)
		{
			RTU_LOG("Device ID error");
			return (uint8_t)eERROR_RTU_ID;
		}
		// memcpy(temp+5, data, size);
		for (int i = 0; i < regNum; i++)
		{
			temp[5 + i * 2] = pBuf[2 * i + 1];
			temp[6 + i * 2] = pBuf[2 * i];
		}
		pRTUHeader header = packed(id, eCMD_WRITE_MULTI_HOLDING, temp, sizeof(temp));
		sendPackage(header);
		header = recvAndParsePackage(id, (uint8_t)eCMD_WRITE_MULTI_HOLDING, reg, &ret);
		size = 0;
		if ((ret == 0) && (header != NULL))
		{
			size = (header->payload[2] << 8) | header->payload[3];
			free(header);
		}
		return ret;
	}

private:
	uint32_t _timeout;
	Stream *_s;
	int _dePin;
};

#if defined(ARDUINO)
template<size_t siz>
class CNSPISlave : public Stream
{
private:
	CNStream	rx, tx;
	static void isrNSS() { get().nss(); }
	static void isrSCK() { get().sck(); }
	int16_t iMOSI, iMISO, iSCK, iNSS;
	uint8_t	uTX, uRX, uCNT;

	void nss()
	{
		uTX = uRX = uCNT = 0;
		pinMode(iMISO, (digitalRead(iNSS)==LOW) ? OUTPUT : INPUT);
	}

	void delayNano(uint32_t nano) {
		for(uint32_t n=0;n<nano;n++) __asm("nop");
	}

	void sck()
	{
		uRX <<= 1;
		uRX |= (digitalRead(iMOSI)==LOW) ? 0 : 1;
		digitalWrite(iMOSI,uTX&0x80?HIGH:LOW);
		uTX <<= 1;
		if (++uCNT>7) {
			uCNT = 0;
			rx.write(uRX);
			uRX = 0;
			uTX = tx.available() ? tx.read() : 0;
		}
	}

public:
	static CNSPISlave& get() { static CNSPISlave s_inst; return s_inst; }

	CNSPISlave(): rx(siz), tx(siz) {}

	void begin(int16_t _iMOSI, int16_t _iMISO, int16_t _iSCK, int16_t _iNSS)
	{
		iMOSI = _iMOSI;
		iMISO = _iMISO;
		iSCK	= _iSCK;
		iNSS 	= _iNSS;

		pinMode(iMOSI, INPUT);
		pinMode(iMISO, INPUT);
		pinMode(iSCK, INPUT);
		pinMode(iNSS, INPUT);

		attachInterrupt(iSCK, isrSCK, FALLING);
		attachInterrupt(iNSS, isrNSS, CHANGE);
	}

	virtual size_t write(uint8_t val) { return tx.write(val); }
	virtual int available() { return rx.available(); }
	virtual int read() { return rx.read(); }
	virtual int peek() { return rx.peek(); }
};
#endif

#endif
