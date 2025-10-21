
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



/** The constant \a pi.	*/
#define M_PI		3.14159265358979323846	/* pi */

/** The constant \a pi/2.	*/
#define M_PI_2		1.57079632679489661923	/* pi/2 */

/** The constant \a pi/4.	*/
#define M_PI_4		0.78539816339744830962	/* pi/4 */

#define M_E		2.7182818284590452354



#endif