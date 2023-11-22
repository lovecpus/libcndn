#include <Arduino.h>
#include <libcndn.hpp>

CNTimeout toSAMPLE1(CNTimeout().disable());
CNTimeout toSAMPLE2;

void setup() {
	Serial.begin(115200);
	Serial.setTimeout(0);

	toSAMPLE1.reset(CNTimeout::update());
	toSAMPLE2.reset(CNTimeout::lastms());
	Serial.println("CNTimeout sample!");
}

void loop() {
	CNTimeout::update();
	if (toSAMPLE1.isTimeout(CNTimeout::lastms(), 500)) {
		toSAMPLE1.disable();
		Serial.println("true ONESHOT 500ms");
	}

	if (toSAMPLE2.isTimeout(CNTimeout::lastms(), 1000)) {
		Serial.println("true EVERY 1000ms");
	}
}
