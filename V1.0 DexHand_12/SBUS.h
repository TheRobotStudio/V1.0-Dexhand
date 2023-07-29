/*
SBUS.h
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2016 Bolder Flight Systems (ESP32 Support enhanced by TheDIYGuy999)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef SBUS_h
#define SBUS_h

#include "Arduino.h"
#include "elapsedMillis.h"

/*
* Hardware Serial Supported:
* Teensy 3.0 || Teensy 3.1/3.2 || Teensy 3.5 || Teensy 3.6 || Teensy LC  || STM32L4 || Maple Mini || Arduino Mega 2560 || ESP32
*/
#if defined(__MK20DX128__) 	|| defined(__MK20DX256__) || defined(__MK64FX512__)	\
	|| defined(__MK66FX1M0__) || defined(__MKL26Z64__) 	|| defined(__IMXRT1052__) \
	|| defined(STM32L496xx)		|| defined(STM32L476xx) 	|| defined(STM32L433xx) \
	|| defined(STM32L432xx)		|| defined(_BOARD_MAPLE_MINI_H_) \
	|| defined(__AVR_ATmega2560__) || defined(ESP32)
#endif

class SBUS{
	public:
        SBUS(HardwareSerial& bus);
        void begin(uint8_t RX_PIN = 16, uint8_t TX_PIN = 17, bool INVERTED = false, uint32_t SBUSBAUD = 100000); // 16, 17 = UART 2, if not specified (for ESP32 only)
		bool read(uint16_t* channels, bool* failsafe, bool* lostFrame);
		bool readCal(float* calChannels, bool* failsafe, bool* lostFrame);
		void write(uint16_t* channels);
		void writeCal(float *channels);
		void setEndPoints(uint8_t channel,uint16_t min,uint16_t max);
		void getEndPoints(uint8_t channel,uint16_t *min,uint16_t *max);
		void setReadCal(uint8_t channel,float *coeff,uint8_t len);
		void getReadCal(uint8_t channel,float *coeff,uint8_t len);
		void setWriteCal(uint8_t channel,float *coeff,uint8_t len);
		void getWriteCal(uint8_t channel,float *coeff,uint8_t len);
		~SBUS();
  private:
		uint32_t _sbusBaud = 100000;
		static const uint8_t _numChannels = 16;
		const uint8_t _sbusHeader = 0x0F;
		const uint8_t _sbusFooter = 0x00;
		const uint8_t _sbus2Footer = 0x04;
		const uint8_t _sbus2Mask = 0x0F;
		const uint32_t SBUS_TIMEOUT_US = 7000;
		uint8_t _parserState, _prevByte = _sbusFooter, _curByte;
		static const uint8_t _payloadSize = 24;
		uint8_t _payload[_payloadSize];
		const uint8_t _sbusLostFrame = 0x04;
		const uint8_t _sbusFailSafe = 0x08;
		const uint16_t _defaultMin = 172;
		const uint16_t _defaultMax = 1811;
		uint16_t _sbusMin[_numChannels];
		uint16_t _sbusMax[_numChannels];
		float _sbusScale[_numChannels];
		float _sbusBias[_numChannels];
		float **_readCoeff, **_writeCoeff;
		uint8_t _readLen[_numChannels],_writeLen[_numChannels];
		bool _useReadCoeff[_numChannels], _useWriteCoeff[_numChannels];
		HardwareSerial* _bus;
		bool parse();
		void scaleBias(uint8_t channel);
		float PolyVal(size_t PolySize, float *Coefficients, float X);
};

#endif
