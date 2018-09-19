/**
 * @file types.h
 * @author Edoardo Idà, Simone Comari
 * @date 18 Sep 2018
 * @brief This file includes common types and struct of libgrabec.
 */

#ifndef GRABCOMMON_LIBGRABEC_TYPES_H
#define GRABCOMMON_LIBGRABEC_TYPES_H

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdarg.h>
#include "grabcommon.h"
#include "threads.h"

namespace grabec
{

/**
 * @brief The RtThreadsParams struct
 */
struct RtThreadsParams
{
  int8_t gui_cpu_id = 0;               /**< CPU ID for GUI (parent) thread */
  int8_t rt_cpu_id = grabrt::END_CORE; /**< CPU ID for RT (child) master thread */
  uint8_t gui_priority = 60;           /**< scheduler priority of GUI (parent) thread */
  uint8_t rt_priority = 98; /**< scheduler priority of RT (child) master thread */
  uint32_t cycle_cime_nsec = 1000000; /**< RT master thread cycle time */
};

/**
 * @brief The Status enum
 */
enum Status: uint8_t
{
  NOT_OPERATIONAL,
  OPERATIONAL
};

/**
*@brief The RetVal enum
*/
enum RetVal: uint8_t
{
  OK,      /**< success */
  ECONFIG, /**< configuration error */
  EREG,    /**< registration error */
  EACTIVE, /**< activation error */
  EINIT,   /**< initialization error */
  EINV,    /**< invalid error */
  FAIL     /**< generic error */
};

/**
 * @brief DispRetVal
 * @param err
 * @param msg
 */
void DispRetVal(const int err, const char* msg, ...);

} // end namespace grabec

#endif // GRABCOMMON_LIBGRABEC_TYPES_H
