
/*
 * fces.h
 *
 * Created: 11.06.2024 14:55:17
 *  Author: Ivan
 */ 

#ifndef FCES
#define FCES

template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

template <typename T> T mabs(T val) {
	return val < T(0) ? -val : val;
}

template <typename T> T max(T val,T val2) {
	return val > val2 ?  val : val2;
}

template <typename T> T min(T val,T val2) {
	return val < val2 ?  val : val2;
}

#endif