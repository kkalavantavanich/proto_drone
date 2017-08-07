/*
 * lpf.h
 *
 *  Created on: Aug 7, 2017
 *      Author: Kris
 */

#ifndef PROTO_DRONE_LPF_H_
#define PROTO_DRONE_LPF_H_

template <typename T = int>
class LPF {
public:
	/** input new value into LPF */
	void push(T val);

	/** peek at last value inputed */
	T peek();

	/** Get the low-passed value */
	T get();
};



#endif /* PROTO_DRONE_LPF_H_ */
