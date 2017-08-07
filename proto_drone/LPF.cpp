/*
 * lpf.cpp
 *
 *  Created on: Aug 7, 2017
 *      Author: Kris
 *
 *        Desc: "Queue-like" Expo. Low pass filter
 */

#define LPF_BUFF_SIZE 10
class LPF {
private:
	int _buff[LPF_BUFF_SIZE] = {0};       // for
	uint8_t _size = 0;
	uint8_t _pos = 0;                     // pointer at last value
	int avg = 0;
	void _updateAvg(int val) {
		avg = val + (avg >> 1);
	  //avg = val + (avg / 2);
	}
public:
	void push(int val) {
		if (_size < LPF_BUFF_SIZE) {
			if (_size > 0) {
				_buff[++_pos] = val;
				_updateAvg(val);
			} else {
				_buff[0] = val;
				avg = val;
			}
			++_size;
		} else {
			_pos = (_pos + 1) % LPF_BUFF_SIZE;
			_buff[_pos] = val;
			_updateAvg(val);
		}

	}
	int peek() {
		return _buff[_pos];
	}
	int get() {
		return avg;
	}
};

