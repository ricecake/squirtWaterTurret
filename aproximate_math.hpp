#include <sys/_types.h>
#include <functional>
#include <stdint.h>
#include "fpm/math.hpp"

using fixed = fpm::fixed_16_16;
// using fixed = fpm::fixed_24_8;

/*
Consider something like a function struct to abstract it away from being fixed point.
*/

/*

Evaluate function at 0, 1, 2, 4, 8, etc until sign changes

*/

namespace Approximate {

fixed sin(fixed);
fixed cos(fixed);
fixed tan(fixed);
fixed atan(fixed);
fixed sqrt(fixed);

// newton approximation
template<typename T>
uint8_t newton(T &result, std::function<T(T)> func, std::function<T(T)> derivative, T initial, T error = T(0.001), uint8_t rounds = 16) {
  bool err = false;
  std::function<T(T)> unifiedFunction = [&](T inputValue) -> T {
    if (err) {
      return 0;
    }

    auto d = derivative(inputValue);

    if (d == 0) {
      err = true;
      return 0;
    }

    auto f = func(inputValue);

    return f / d;
  };

  T lastValue = initial;
  for (uint8_t round = 0; round <= rounds; ++round) {
    T nextValue = lastValue - unifiedFunction(lastValue);

    if (err) {
      return 0;
    }

    // Serial.printf("%d \t %0.10f --- %0.10f = %0.10f\n", round, double(lastValue), double(nextValue), double(lastValue - nextValue));

    if (fpm::abs(lastValue - nextValue) < error) {
      result = nextValue;
      return 1;
    }

    lastValue = nextValue;
  }

  return 0;
};

template<typename T>
bool first_root(T &result, std::function<T(T)> func, std::function<T(T)> derivative, T error = T(0.001), uint8_t rounds = 16) {
  bool done = false;
  T lastInput = 0;
  T nextInput = 1;
  T nextValue;
  T lastValue = func(lastInput);
  T guess;

  T bisect_error = 100*error;
  uint8_t bisect_rounds = rounds / 2;
  uint8_t round = 0;
  do {
    // for (uint8_t round = 0; round <= rounds; ++round) {
    nextValue = func(nextInput);

    // Serial.printf("%d\t%f[%f] %f[%f]\n", round, double(lastInput), double(lastValue), double(nextInput), double(nextValue));

    if (fpm::signbit(lastValue) != fpm::signbit(nextValue)) {
      // There's a root inside the interval
      // done = fpm::abs(nextValue) < bisect_error;
      done = fpm::abs(  2*(nextInput-lastInput) / (nextInput+lastInput) ) < bisect_error;
      if (!done) {
        nextInput = (lastInput + nextInput) / 2;
      }
    } else {
      auto tmp = lastInput;
      lastInput = nextInput;
      lastValue = nextValue;
      nextInput = (2*nextInput) - tmp;
    }

  } while ((round++ <= bisect_rounds) && !done);

    // guess = (lastInput+nextInput / 2);

  // guess = lastInput+(lastInput-nextInput)/2;
  if (fpm::abs(lastValue) < fpm::abs(nextValue)) {
    guess = lastInput;
  }
  else {
    guess = nextInput;
  }

  return newton(result, func, derivative, guess, error, rounds) > 0;
}


template<typename T>
bool small_root(T &result, std::function<T(T)> func, T error = T(0.001), uint8_t rounds = 16) {
  T leftInput = 0;
  T rightInput = 1;
  T midInput;

  T leftValue = func(leftInput);
  T rightValue= func(rightInput);
  T midValue;

  uint8_t round = 0;

  // Find the interval containing first root
  while ((fpm::signbit(leftValue) == fpm::signbit(rightValue)) && (round < rounds)) {
    Serial.printf("%d-\t%f[%f] %f[%f]\n", round, double(leftInput), double(leftValue), double(rightInput), double(rightValue));

    leftInput = rightInput;
    leftValue = rightValue;
    rightInput *= 2;
    rightValue = func(rightInput);
    round++;
  }


  do {
    midInput = rightInput - rightValue*((rightInput - leftInput)/(rightValue-leftValue));
    if (midInput <= leftInput || midInput >= rightInput) {
      midInput = leftInput + (rightInput - leftInput) / 2;
    }
    midValue = func(midInput);

    Serial.printf("%d+\t%f[%f] == %f[%f] == %f[%f]\n", round, double(leftInput), double(leftValue), double(midInput), double(midValue), double(rightInput), double(rightValue));

    if (fpm::signbit(leftValue) == fpm::signbit(midValue)) {
      leftInput = midInput;
      leftValue = midValue;
    }
    else {
      rightInput = midInput;
      rightValue = midValue;
    }

    if((rightInput-leftInput)/rightInput < error) {
      result = midInput;
      return true;
    }

  }
  while (round++ < rounds);
  result = midInput;
  return false;
}


}


/*
1/4 (  (0,0,-9.814) dot (0,0,-9.814) )  t^4+( (I,j,0) dot (0,0,-9.814) )t^3+((x,y,z) dot (0,0,-9.814) +( (I,j,0) dot (I,j,0)) - s^2)t^2+2((x,y,z) dot (I,j,0))t+(x,y,z) dot (x,y,z)

differentiate 1/4 (  (0,0,-9.814) dot (0,0,-9.814) )  t^4+( (I,j,0) dot (0,0,-9.814) )t^3+((x,y,z) dot (0,0,-9.814) +( (I,j,0) dot (I,j,0)) - s^2)t^2+2((x,y,z) dot (I,j,0))t+(x,y,z) dot (x,y,z) wrt t

t^2 (x (j^2 - 1 s^2 - 1) + y (j^2 - 1 s^2 - 1) + z (j^2 - 1 s^2 - 10.814)) + 2 j t y + 24.0786 t^4 + i (2 t x + 0) + x^2 + y^2 + z^2 + 0

2 t (x (j^2 - 1 s^2 - 1) + y (j^2 - 1 s^2 - 1) + z (j^2 - 1 s^2 - 10.814)) + 2 j y + 96.3146 t^3 + i (2 x + 0) + 0

(t^2(x(j^2-1s^2-1)+y(j^2-1s^2-1)+z(j^2-1s^2-10.814))+2jty+24.0786t^4+i(2tx)+x^2+y^2+z^2)/(2t(x(j^2-1s^2-1)+y(j^2-1s^2-1)+z(j^2-1s^2-10.814))+2jy+96.3146t^3+i(2x))


2 i t x + j^2 t^2 x + j^2 t^2 y + j^2 t^2 z + 2 j t y - s^2 t^2 x - s^2 t^2 y - s^2 t^2 z + 24.0786 t^4 - t^2 x - t^2 y - 10.814 t^2 z + x^2 + y^2 + z^2

2 j^2 t x + 2 j^2 t y + 2 j^2 t z + 2 j y - 2 s^2 t x - 2 s^2 t y - 2 s^2 t z + 96.3146 t^3 - 2 t x - 2 t y - 21.628 t z + 2 i x





t (2 i x + 2 j y) + t^2 (j^2 (x + y + z) + s^2 (-x - y - z) - x - y - 10.814 z) + 24.0786 t^4 + x^2 + y^2 + z^2

2 i x + t (j^2 (2 x + 2 y + 2 z) + s^2 (-2 x - 2 y - 2 z) - 2 x - 2 y - 21.628 z) + 2 j y + 96.3146 t^3



(t(2 i x + 2 j y) + t^2 (j^2 (x + y + z) + s^2 (-x - y - z) - x - y - 10.814 z) + 24.0786 t^4 + x^2 + y^2 + z^2) / (2 i x + t (j^2 (2 x + 2 y + 2 z) + s^2 (-2 x - 2 y - 2 z) - 2 x - 2 y - 21.628 z) + 2 j y + 96.3146 t^3)



(t^2 (j^2 (x + y + z) + s^2 (-x - y - z) - x - y - 10.814 z) + t (2 j y + 2 i x) + 24.0786 t^4 + x^2 + y^2 + z^2)  /  (t (j^2 (2 x + 2 y + 2 z) + s^2 (-2 x - 2 y - 2 z) - 2 x - 2 y - 21.628 z) + 2 j y + 96.3146 t^3 + 2 i x)





(
  t^2 (j^2 (x + y + z) + s^2 (-x - y - z) - x - y - 10.814 z) +
  t (2 j y + 2 i x) +
  24.0786 t^4 +
  x^2 + y^2 + z^2
)

/

(
  t (j^2 (2 x + 2 y + 2 z) + s^2 (-2 x - 2 y - 2 z) - 2 x - 2 y - 21.628 z) + 2 j y +
  96.3146 t^3 +
  2 i x
)




(
  24.0786 t^4 +
  t^2 (j^2(x+y+z) - s^2(x+y+z) - (x+y) - 10.814z) +
  t 2(jy+ix) +
  x^2 + y^2 + z^2
)
/
(
  96.3146 t^3 +
  t 2(j^2(x+y+z) - s^2(x+y+z) - (x+y) - 10.814z) +
  2(jy + ix)
)

xy = x+y;
xyz = xy+z

coOne = pow(j, 2)*xyz - pow(s, 2)*xyz - xy - 10.814*z;
coTwo = 2*coOne
coThree = 2*(j*y+i*x)

coFour = pow(x, 2) + pow(y, 2) + pow(z, 2)

eq = (24.0786 * pow(t, 4) + pow(t, 2) * coOne + t * coThree + coFour) / (96.3146 * pow(t, 3) + t*2*coOne + coThree)

*/