//
// Created by bjoshi on 8/26/20.
//
#include <chrono>
#include "include/utils.h"
#include <iostream>
#include "date.h"
#include <sstream>

uint64_t parseISO(const std::string& iso_date) {

	using namespace date;
	using namespace std::chrono;

	date::sys_time<std::chrono::microseconds> tp;
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

std::string uint64_to_string(uint64_t value){
	std::ostringstream os;
	os << value;
	return os.str();
}