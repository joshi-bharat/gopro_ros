//
// Created by bjoshi on 8/26/20.
//

#pragma once

#include <cstdint>
#include <string>

uint64_t parseISO(const std::string &iso_date);
std::string uint64_to_string(uint64_t value);
uint64_t get_offset_1904();
