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
	static void scanWire(TwoWire& wire, Stream& strm) {
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

  static String print_v(const char *form, va_list args) {
    char buff[4096];
    buff[0] = 0;
    int size = vsprintf(buff, form, args);
    if (size>=0) buff[size] = 0;
    return buff;
  }

  static String print_r(const char *form, ...) {
    va_list args; va_start(args, form);
		return print_v(form, args);
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
	size_t 		tx, rx;
	size_t 		memsize;
public:
	CNStream(size_t _size): tx(0), rx(0) {
		mem = (uint8_t*)malloc(memsize=_size);
	}

	~CNStream() {
		if (mem != nullptr) {
			free(mem);
			mem = nullptr;
		}
	}

	virtual int available() { return (rx>tx)?(memsize+tx-rx):(tx-rx); }
	virtual int read() { 
		size_t pos = rx;
		rx=(rx+1)%memsize;
		return mem[pos];
	}
	virtual int peek() { return mem[rx]; }
	virtual size_t write(uint8_t dat) {
		mem[tx++]=dat;
		tx=tx%memsize;
		if (tx==rx) rx=(rx+1)%memsize;
		return 1;
	}
};

class RTStream : public Stream
{
private:
	CNStream	rxs;
public:
	CNStream	txs;
	RTStream(size_t txSize, size_t rxSize) : rxs(rxSize), txs(txSize) {}

	size_t writeRx(uint8_t dat) { return rxs.write(dat); }

	virtual int available() { return rxs.available(); }
	virtual int read() { return rxs.read(); }
	virtual int peek() { return rxs.peek(); }
	virtual size_t write(uint8_t dat) { return txs.write(dat); }
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

#endif
