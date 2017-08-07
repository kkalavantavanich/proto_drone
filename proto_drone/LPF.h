/*
 * lpf.h
 *
 *  Created on: Aug 7, 2017
 *      Author: Kris
 */

#ifndef PROTO_DRONE_LPF_H_
#define PROTO_DRONE_LPF_H_

class LPF {
public:
	void push(int val);
	int peek();
	int get();
};



#endif /* PROTO_DRONE_LPF_H_ */
