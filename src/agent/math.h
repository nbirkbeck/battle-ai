#ifndef BATTLE_AI_MATH_H
#define BATTLE_AI_MATH_H

#include <math.h>

inline double urand(double min_value = 0, double max_value = 1) {
  return ((double)rand()) / RAND_MAX * (max_value - min_value) + min_value;
}

#endif
