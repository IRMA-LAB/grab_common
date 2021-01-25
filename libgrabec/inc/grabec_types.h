/**
 * @file grabec_types.h
 * @author Edoardo Idà, Simone Comari
 * @date 25 Jan 2021
 * @brief This file includes common types and struct of libgrabec.
 */

#ifndef GRABCOMMON_LIBGRABEC_GRABEC_TYPES_H
#define GRABCOMMON_LIBGRABEC_GRABEC_TYPES_H

#include <iostream>
#include <stdarg.h>
#include <stdio.h>
#include <string>

#include "threads.h"

/**
 * @brief Namespace for GRAB EtherCAT library.
 */
namespace grabec {

// clang-format off
/** Working Counter state's corresponding meaning. */
constexpr char* WcStateStr[] = {
  const_cast<char*>("ZERO"),
  const_cast<char*>("INCOMPLETE"),
  const_cast<char*>("COMPLETE")};
// clang-format on

/**
 * @brief The RtThreadsParams struct
 */
struct RtThreadsParams
{
  int8_t main_cpu_id = 0; /**< CPU ID for main (parent) thread */
  std::vector<int8_t> rt_cpu_id {
    2, grabrt::END_CORE};        /**< CPU ID for RT (child) master thread */
  uint8_t rt_priority      = 98; /**< scheduler priority of RT (child) master thread */
  uint32_t cycle_time_nsec = 1000000; /**< RT master thread cycle time */
};

/**
 * @brief The Status enum
 */
enum Status : uint8_t
{
  NOT_OPERATIONAL,
  OPERATIONAL
};

/**
 * @brief The Commands enum for bit assignment.
 */
enum Commands : uint8_t
{
  UNSET,
  SET
};

} // end namespace grabec

#endif // GRABCOMMON_LIBGRABEC_GRABEC_TYPES_H
