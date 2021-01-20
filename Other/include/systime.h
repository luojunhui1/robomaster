#ifndef _PLATFORM_H_
#define _PLATFORM_H_

typedef double systime;

void getsystime(systime& t);
double getTimeIntervalms(const systime& now, const systime& last);

#if defined(__linux__) || defined(Darwin) || defined(Debian) || defined(Linux)
#include <sys/time.h>
#elif defined(Windows) || defined(_WIN32)
#include <Windows.h>
#else
#error "nonsupport platform."
#endif

#endif /* _PLATFORM_H_ */

