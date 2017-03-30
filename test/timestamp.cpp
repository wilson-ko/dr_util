#include "../src/chrono_ptime.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <ctime>
#include <stdlib.h>

int main(int argc, char ** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

TEST(timestamp, system_clock_epoch) {
	std::time_t epoch = std::chrono::system_clock::to_time_t(std::chrono::system_clock::time_point{});
	std::tm * date = std::gmtime(&epoch);
	ASSERT_EQ(date->tm_year,  70);
	ASSERT_EQ(date->tm_mon,   0);
	ASSERT_EQ(date->tm_mday,  1);
	ASSERT_EQ(date->tm_hour,  0);
	ASSERT_EQ(date->tm_min,   0);
	ASSERT_EQ(date->tm_sec,   0);
	ASSERT_EQ(date->tm_yday,  0);
	ASSERT_EQ(date->tm_isdst, 0);
}

TEST(timestamp, chrono_ptime_epoch) {
	boost::posix_time::ptime epoch = toPtime(std::chrono::system_clock::time_point{});
	ASSERT_EQ(epoch.date().year(), 1970);
	ASSERT_EQ(epoch.date().month(), 1);
	ASSERT_EQ(epoch.date().day(), 1);
	ASSERT_EQ(epoch.date().day_of_year(), 1);
	ASSERT_EQ(epoch.time_of_day(), boost::posix_time::seconds(0));
}

TEST(timestamp, chrono_ptime_2017) {
	setenv("TZ", "UTC", true);
	::tzset();

	std::tm tm;
	tm.tm_year = 117;
	tm.tm_mon  = 2;
	tm.tm_mday = 30;
	tm.tm_wday = 4;
	tm.tm_hour = 13;
	tm.tm_min  = 37;
	tm.tm_sec  = 42;

	boost::posix_time::ptime time = toPtime(std::chrono::system_clock::from_time_t(std::mktime(&tm)));

	ASSERT_EQ(time.date().year(),           2017);
	ASSERT_EQ(time.date().month(),          boost::gregorian::Mar);
	ASSERT_EQ(time.date().day(),            30);
	ASSERT_EQ(time.date().day_of_year(),    89);
	ASSERT_EQ(time.date().day_of_week(),    boost::gregorian::Thursday);
	ASSERT_EQ(time.time_of_day().hours(),   13);
	ASSERT_EQ(time.time_of_day().minutes(), 37);
	ASSERT_EQ(time.time_of_day().seconds(), 42);
}


}
