#pragma once

#include "../pins.h"
#include "../scheduler.h"


template<unsigned int pin>
void toggle() {
	static bool state = true;
	digitalWrite(pin, state);
	state = !state;
}

namespace Programmer {
class Leds {
	public:
	Leds() : sch() {
		// initialize LED pins
		pinMode(PROG_LED1, OUTPUT);
		pinMode(PROG_LED2, OUTPUT);
		pinMode(PROG_LED3, OUTPUT);
		pinMode(PROG_LED4, OUTPUT);
		clearAll();

		// initialize interval indices
		for (int i=0; i<4; i++) { intervals[i] = -1; }
	}

	void flashForward()  { setFlashing(0, true); }
	void clearForward()  { setFlashing(0, false); digitalWrite(PROG_LED1, 0); }

	void flashBackward() { setFlashing(1, true); }
	void clearBackward() { setFlashing(1, false); digitalWrite(PROG_LED2, 0); }

	void flashLeft()     { setFlashing(2, true); }
	void clearLeft()     { setFlashing(2, false); digitalWrite(PROG_LED3, 0); }

	void flashRight()    { setFlashing(3, true); }
	void clearRight()    { setFlashing(3, false); digitalWrite(PROG_LED4, 0); }

	void clearAll() {
		clearForward();
		clearBackward();
		clearLeft();
		clearRight();
	}

	void update() {
		sch.update();
	}

	private:
	Scheduler sch;
	int intervals[4];

	void setFlashing(int index, bool flashing) {
		if (!flashing) {
			sch.clearInterval(intervals[index]);
		} else {
			switch(index) {
			case 0:
				intervals[0] = sch.setInterval(toggle<PROG_LED1>, 200);
				return;
			case 1:
				intervals[1] = sch.setInterval(toggle<PROG_LED2>, 200);
				return;
			case 2:
				intervals[2] = sch.setInterval(toggle<PROG_LED3>, 200);
				return;
			case 3:
				intervals[3] = sch.setInterval(toggle<PROG_LED4>, 200);
				return;
			default:
				return;
			}
		}
	}
};
} // end namespace Programmer
