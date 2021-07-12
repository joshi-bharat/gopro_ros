//
// Created by bjoshi on 8/26/20.
//
#include <chrono>
#include "utils.h"
#include <iostream>
#include "date.h"
#include <sstream>

uint64_t parseISO(const std::string &iso_date)
{

	using namespace date;
	using namespace std::chrono;

	date::sys_time<std::chrono::nanoseconds> tp;
	std::istringstream in(iso_date);
	in >> date::parse("%FT%TZ", tp);
	if (in.fail())
	{
		in.clear();
		in.exceptions(std::ios::failbit);
		in.str(iso_date);
		in >> date::parse("%FT%T%Ez", tp);
	}

	uint64_t time = tp.time_since_epoch().count();

	return time;
}

std::string uint64_to_string(uint64_t value)
{
	std::ostringstream os;
	os << value;
	return os.str();
}

uint64_t get_offset_1904()
{
	using namespace date;
	using namespace std::chrono;
	constexpr auto offset = sys_days{January / 1 / 1970} - sys_days{January / 1 / 1904};
	uint64_t offset_secs = duration_cast<seconds>(offset).count();
	return offset_secs;
}