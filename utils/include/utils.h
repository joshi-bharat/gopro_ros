//
// Created by bjoshi on 8/26/20.
//

#ifndef VIDEO_SYNCHRONIZATION_UTILS_H
#define VIDEO_SYNCHRONIZATION_UTILS_H

#pragma once

#include <cstdint>
#include <string>


uint64_t parseISO(const std::string& iso_date);
std::string uint64_to_string(uint64_t value);


#endif //VIDEO_SYNCHRONIZATION_UTILS_H
