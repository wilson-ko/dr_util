#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/date_time/gregorian/gregorian_types.hpp>

#include <chrono>

namespace dr {

/// Get the POSIX epoch as ptime.
inline boost::posix_time::ptime ptimeEpoch() {
	return {boost::gregorian::date(1970, 1, 1), boost::posix_time::seconds(0)};
}

/// Convert a time point to a tuple of ptime {seconds, subseconds}.
#ifdef BOOST_DATE_TIME_HAS_NANOSECONDS
template<typename Duration>
inline std::tuple<boost::posix_time::seconds, boost::posix_time::nanoseconds> ptimeDuration(Duration duration) {
	using rep = std::chrono::nanoseconds::rep;
	rep ticks = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
	rep sec  = ticks / 1000000000;
	rep nsec = ticks % 1000000000;
	return {boost::posix_time::seconds(sec), boost::posix_time::nanoseconds(nsec)};
}
#else
template<typename Duration>
inline std::tuple<boost::posix_time::seconds, boost::posix_time::microseconds> ptimeDuration(Duration duration) {
	using rep = std::chrono::nanoseconds::rep;
	rep ticks = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
	rep sec  = ticks / 1000000000;
	rep nsec = ticks % 1000000000;
	return {boost::posix_time::seconds(sec), boost::posix_time::microseconds((nsec+500)/1000)};
}
#endif

/// Convert a time point to ptime.
template<class Clock, class Duration>
boost::posix_time::ptime toPtime(std::chrono::time_point<Clock, Duration> from) {
	auto offset = ptimeDuration(from.time_since_epoch());
	return ptimeEpoch() + std::get<0>(offset) + std::get<1>(offset);
}

}
