#pragma once

#ifdef WIN32
#include "Windows.h"
#endif

#ifdef __GLIBC__
#include <time.h>
#endif

#ifdef __APPLE__
#include <mach/mach_time.h>
#endif

namespace Utility
{
	class CodeTimer
	{
	public:
		CodeTimer()
		{
#ifdef WIN32
			QueryPerformanceFrequency(&freq);
#endif
#ifdef __GLIBC__
			clock_getres(CLOCK_REALTIME, &freq);	
#endif
#ifdef __APPLE__
			mach_timebase_info(&timebase);
#endif
		}

		void start() {
#ifdef WIN32
			QueryPerformanceCounter(&t1);
#endif
#ifdef __GLIBC__
			clock_gettime(CLOCK_REALTIME, &t1);
#endif
#ifdef __APPLE__
			t1 = mach_absolute_time();
#endif
		}

		void stop() {
#ifdef WIN32
			QueryPerformanceCounter(&t2);
#endif
#ifdef __GLIBC__
			clock_gettime(CLOCK_REALTIME, &t2);
#endif
#ifdef __APPLE__
			t2 = mach_absolute_time();
#endif
		}

		double elapsed() {
#ifdef WIN32
			return static_cast<double>(t2.QuadPart - t1.QuadPart) / static_cast<double>(freq.QuadPart);
#endif
#ifdef __GLIBC__
			return static_cast<double>(t2.tv_sec - t1.tv_sec) + 
						static_cast<double>(t2.tv_nsec - t1.tv_nsec)/1e9;
#endif
#ifdef __APPLE__
			uint64_t elapsed = t2 - t1;
			// Convert to nanoseconds using timebase info
			double elapsed_ns = static_cast<double>(elapsed * timebase.numer) / timebase.denom;
			return elapsed_ns / 1e9; // Convert to seconds
#endif
		}

	private:
#ifdef WIN32
		LARGE_INTEGER t1, t2, freq;
#endif
#ifdef __GLIBC__
		timespec freq, t1, t2;
#endif
#ifdef __APPLE__
		uint64_t t1, t2;
		mach_timebase_info_data_t timebase;
#endif
	};
}
