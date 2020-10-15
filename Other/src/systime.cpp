#include <systime.h>

#if defined(__linux__) || defined(Linux) || defined(Darwin) ||defined(Debian)

static systime getsystime() {
	timeval tv;
	gettimeofday(&tv, nullptr);
	return tv.tv_usec / 1000.0 + tv.tv_sec * 1000.0;
}

void getsystime(systime& t) {
	static systime time_base = getsystime();
	timeval tv;
	gettimeofday(&tv, nullptr);
	t = tv.tv_usec / 1000.0 + tv.tv_sec * 1000.0 - time_base;
}

#elif defined(Windows) || defined(_WIN32)

void getsystime(systime& t) {
	SYSTEMTIME tv;
	GetLocalTime(&tv);
	t = tv.wMilliseconds + tv.wSecond * 1000.0;
}

#else
#error "nonsupport platform."
#endif


double getTimeIntervalms(const systime& now, const systime& last) {
	return now - last;
}