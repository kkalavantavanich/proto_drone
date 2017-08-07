/*
 * lpf.cpp
 *
 *  Created on: Aug 7, 2017
 *      Author: Kris
 *
 *        Desc: "Queue-like" Expo. Low pass filter
 */

#define LPF_BUFF_SIZE 10

template <typename T = int>
class LPF {
private:
	T _buff[LPF_BUFF_SIZE] = {0};       // for future use
	uint8_t _size = 0;
	uint8_t _pos = 0;                   // pointer at last value
	T avg = 0;                          // average value
	void _updateAvg(T val) {
		avg = val + (avg >> 1);
	  //avg = val + (avg / 2);
	}
public:
	void push(T val) {
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
	T peek() {
		return _buff[_pos];
	}
	T get() {
		return avg;
	}
};

