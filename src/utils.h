#ifndef __utils_h__
#define __utils_h__

#include <iostream>
#include <inttypes.h>
#include <cstdlib>
#include <vector>
#include <set>
#include <map>
#include <cassert>
#include <sys/time.h>
using namespace std;

#define RANDF   (rand()/(RAND_MAX+1.0))

typedef uint8_t symbol;

typedef struct tt{
  struct timeval _time;
  void tic()
  {
    gettimeofday(&_time, NULL);
  }
  double toc() const
  {
    struct timeval t1;
    gettimeofday(&t1, NULL);
    float sec = t1.tv_sec - _time.tv_sec;
    float usec = t1.tv_usec - _time.tv_usec;
     
    return (sec*1000.0 + usec/1000.0);
  }
}tt;

#endif
