//
// Created by LEI XU on 4/9/19.
//

#ifndef RASTERIZER_GLOBAL_H
#define RASTERIZER_GLOBAL_H

typedef unsigned char u08;
#define MY_PI 3.1415926
#define TWO_PI (2.0* MY_PI)

template<typename T>
T fastpow(T a, int p) {
  // without MOD
  T ans = 1;
  for (; p; p >>= 1) {
    if (p & 1)
      ans *= a;
    a *= a;
  }
  return ans;
}

#endif //RASTERIZER_GLOBAL_H
