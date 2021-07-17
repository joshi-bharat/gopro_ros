//
// Created by bjoshi on 8/26/20.
//

#pragma once

#include <cstdint>
#include <string>

uint64_t parseISO(const std::string &iso_date);
std::string uint64_to_string(uint64_t value);
uint64_t get_offset_1904();

#include <cmath>
#include <iomanip>
#include <ostream>
#include <string>

class ProgressBar
{

private:
    static const auto overhead = sizeof " [100%]";
    std::ostream &os;
    const std::size_t bar_width;
    std::string message;
    const std::string full_bar;

public:
    ProgressBar(std::ostream &os, std::size_t line_width,
                std::string message_, const char symbol = '.');

        

    // not copyable
    ProgressBar(const ProgressBar &) = delete;
    ProgressBar &operator=(const ProgressBar &) = delete;

    ~ProgressBar();

    void write(double fraction);
};
