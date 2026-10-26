#pragma once


#ifndef SERIAL_CONTROLLER_HPP
#define SERIAL_CONTROLLER_HPP


#include <Arduino.h>
#include <RHReliableDatagram.h>
#include <RH_RF69.h>
#include <SPI.h>

#define MAX_STRING_LEN 64
#define MAX_RETRIES 3 
#define TX_POWER 18

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

typedef enum {
	WAIT_FOR_START,
	PARSE_KEY,
	PARSE_KEY_OVERFLOW,
	PARSE_VALUE,
	PARSE_VALUE_OVERFLOW,
	N_PARSE_STATES
} parseState;

static void cleanString(char* string);

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

class RadioSerial {
	private:
	byte pin_rst;
	RH_RF69 driver;
	RHReliableDatagram radio;
	float freq;
	byte partner_addr;

	parseState state;
	char key[MAX_STRING_LEN];
	char value[MAX_STRING_LEN];
	int keyIndex, valueIndex;
	uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
	
	public:
	RadioSerial(
		byte pin_cs, byte pin_rst, byte pin_int, float freq)
	: state(WAIT_FOR_START), pin_rst(pin_rst), driver(pin_cs, pin_int), freq(freq), radio(driver, 0) {
		memset(key, 0, sizeof(key));
		memset(value, 0, sizeof(value));
		memset(buf, 0, sizeof(buf));
	}

	void setup() {
		// reset radio
		Serial.println(F("  reset"));
		pinMode(pin_rst, OUTPUT);
		digitalWrite(pin_rst, 0);
		delay(10);
		digitalWrite(pin_rst, 1);
		delay(100);
		digitalWrite(pin_rst, 0);
		delay(250);

		// initialize & configure radio
		Serial.println(F("  init"));
		if (!driver.init()) {
			Serial.println(F("radio initialization failed!"));
		}

		Serial.println(F("  set frequency & power"));
		driver.setFrequency(freq);
		driver.setTxPower(TX_POWER, true);
		this->radio.setRetries(MAX_RETRIES);
	}
	
	void set_addrs(byte self, byte partner) {
		radio.setThisAddress(self);
		partner_addr = partner;
	}
	
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
	void sendMessage(char* messageKey, char* messageValue) {
		cleanString(messageKey);
		cleanString(messageValue);
		
		char result[2*MAX_STRING_LEN+3];
		strcpy(result,"{");
		strcat(result,messageKey);
		strcat(result,":");
		strcat(result,messageValue);
		strcat(result,"}");
		
		size_t len = strlen(result);
		if (!radio.sendtoWait((uint8_t*) result, len+1, partner_addr)) {
			Serial.println(F("WARNING: radio transmission failed!"));
		}
	}
	
	void sendMessage(char* messageKey, int messageValue) {
		char stringValue[MAX_STRING_LEN];
		snprintf(stringValue, MAX_STRING_LEN, "%d", messageValue);
		sendMessage(messageKey, stringValue);
	}
	
	void sendMessage(char* messageKey, unsigned int messageValue) {
		char stringValue[MAX_STRING_LEN];
		snprintf(stringValue, MAX_STRING_LEN, "%u", messageValue);
		sendMessage(messageKey, stringValue);
	}
	
	void sendMessage(char* messageKey, long int messageValue) {
		char stringValue[MAX_STRING_LEN];
		snprintf(stringValue, MAX_STRING_LEN, "%ld", messageValue);
		sendMessage(messageKey, stringValue);
	}
	
	void sendMessage(char* messageKey, long unsigned int messageValue) {
		char stringValue[MAX_STRING_LEN];
		snprintf(stringValue, MAX_STRING_LEN, "%lu", messageValue);
		sendMessage(messageKey, stringValue);
	}
	
	void sendMessage(char* messageKey, float messageValue) {
		char stringValue[MAX_STRING_LEN];
		snprintf(stringValue, MAX_STRING_LEN, "%f", messageValue);
		sendMessage(messageKey, stringValue);
	}

	void update() {
		// needed because len only gets set if it is greater than message length!
		// this makes sense (set len to your buffer size and it will never exceed that)
		// but it annoyingly undocumented...
		uint8_t len = sizeof(buf);
		if(radio.recvfromAck(buf, &len)) {
			Serial.println(len);
			for (int i=0; i<len; i++) {
				char c = (char) buf[i];
				Serial.print(i); Serial.print(" "); Serial.println(c);
				eatChar(c);
				Serial.println("_");
			}
		}
	}
	
	void eatChar(char c) {
		switch (state) {
			case WAIT_FOR_START:
				if (c == '{') {
					state = PARSE_KEY;
					strcpy(key,"");
					strcpy(value,"");
					keyIndex = 0;
					valueIndex = 0;
				}
				break;
			
			case PARSE_KEY:
				if (keyIndex == MAX_STRING_LEN-1) {
					key[keyIndex] = 0;
					state = PARSE_KEY_OVERFLOW;
				}
				else if (c == '{') {
					strcpy(key,"");
					strcpy(value,"");
					keyIndex = 0;
					valueIndex = 0;
				}
				else if (c == ':') {
					key[keyIndex] = 0;
					state = PARSE_VALUE;
				}
				else if (c == '}') {
					key[keyIndex] = 0;
					onMessage(key,value);
					state = WAIT_FOR_START;
				}
				else {
					key[keyIndex] = c;
					keyIndex++;
				}
				break;
			
			case PARSE_VALUE:
				if (valueIndex == MAX_STRING_LEN - 1) {
					value[valueIndex] = 0;
					state = PARSE_VALUE_OVERFLOW;
				}
				else if (c == '}') {
					value[valueIndex] = 0;
					onMessage(key,value);
					state = WAIT_FOR_START;
				}
				else {
					value[valueIndex] = c;
					valueIndex++;
				}
				break;
			
			case PARSE_KEY_OVERFLOW:
				if (c == ':') {
					state = PARSE_VALUE;
				}
				else if (c == '{') {
					strcpy(key,"");
					strcpy(value,"");
					keyIndex = 0;
					valueIndex = 0;
					state = PARSE_KEY;
				}
				break;
			
			case PARSE_VALUE_OVERFLOW:
				if (c == '{') {
					strcpy(key,"");
					strcpy(value,"");
					keyIndex = 0;
					valueIndex = 0;
					state = PARSE_KEY;
				}
				else if (c == '}') {
					onMessage(key,value);
					state = WAIT_FOR_START;
				}
				break;
			
			default:
				// something's gone wrong, reset
				state = WAIT_FOR_START;
				break;
		}
	}

	virtual void onMessage(const char *key, const char *value) = 0;
};

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void cleanString(char* string) {
	// remove '{', '}', and ':' delimiter characters from a string
	char result[MAX_STRING_LEN];
	
	int i = 0;
	int j = 0;
	while(string[i] != 0) {
		if (string[i] == '{'
			|| string[i] == '}'
			|| string[i] == ':') {
			i++;
			}
		else {
			result[j] = string[i];
			i++; j++;
		}
	}
	
	// close string
	result[j] = 0;
	
	string = result;
}

#endif
