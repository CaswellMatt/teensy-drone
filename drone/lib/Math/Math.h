#ifndef _MATH_H
#define _MATH_H


class Math {
public:
  static float fastInverseSquareRoot(float x) {
  	float halfx = 0.5f * x;
  	float y = x;
  	long i = *(long*)&y;
  	i = 0x5f3759df - (i>>1);
  	y = *(float*)&i;
  	y = y * (1.5f - (halfx * y * y));
  	return y;
  }
};
#endif
