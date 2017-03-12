/*
 * Copyright 2013, Broadcom Corporation
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 */

/** @file
 *  Defines functions to set and get the current time
 */

#pragma once

#include "wiced_utilities.h"
#include "RTOS/wwd_rtos_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                    Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/** @cond !ADDTHIS*/
#define MILLISECONDS      (1)
#define SECONDS           (1000)
#define MINUTES           (60 * SECONDS)
#define HOURS             (60 * MINUTES)
#define DAYS              (24 * HOURS)
/** @endcond */

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/**< UTC Time */
typedef uint32_t  wiced_utc_time_t;
typedef uint64_t  wiced_utc_time_ms_t;

/******************************************************
 *                    Structures
 ******************************************************/

/** ISO8601 Time Structure
 */
typedef struct
{
    char year[4];        /**< Year         */
    char dash1;          /**< Dash1        */
    char month[2];       /**< Month        */
    char dash2;          /**< Dash2        */
    char day[2];         /**< Day          */
    char T;              /**< T            */
    char hour[2];        /**< Hour         */
    char colon1;         /**< Colon1       */
    char minute[2];      /**< Minute       */
    char colon2;         /**< Colon2       */
    char second[2];      /**< Second       */
    char decimal;        /**< Decimal      */
    char sub_second[6];  /**< Sub-second   */
    char Z;              /**< UTC timezone */
} wiced_iso8601_time_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 *
 ******************************************************/

/*****************************************************************************/
/** @addtogroup time       Time management functions
 *
 * Functions to get and set the real-time-clock time.
 *
 *
 *  @{
 */
/*****************************************************************************/


/** Get the current system tick time in milliseconds
 *
 * @note The time will roll over every 49.7 days
 *
 * @param[out] time : A pointer to the variable which will receive the time value
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_time_get_time( wiced_time_t* time );


/** Set the current system tick time in milliseconds
 *
 * @param[in] time : the time value to set
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_time_set_time( wiced_time_t* time );


/** Get the current UTC time in seconds
 *
 * This will only be accurate if the time has previously been set by using @ref wiced_time_set_utc_time_ms
 *
 * @param[out] utc_time : A pointer to the variable which will receive the time value
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_time_get_utc_time( wiced_utc_time_t* utc_time );


/** Get the current UTC time in milliseconds
 *
 * This will only be accurate if the time has previously been set by using @ref wiced_time_set_utc_time_ms
 *
 * @param[out] utc_time_ms : A pointer to the variable which will receive the time value
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_time_get_utc_time_ms( wiced_utc_time_ms_t* utc_time_ms );


/** Set the current UTC time in milliseconds
 *
 * @param[in] utc_time_ms : the time value to set
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_time_set_utc_time_ms( wiced_utc_time_ms_t* utc_time_ms );


/** Get the current UTC time in iso 8601 format e.g. "2012-07-02T17:12:34.567890Z"
 *
 * @note The time will roll over every 49.7 days
 *
 * @param[out] iso8601_time : A pointer to the structure variable that
 *                            will receive the time value
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_time_get_iso8601_time( wiced_iso8601_time_t* iso8601_time );

/** @} */

#ifdef __cplusplus
} /*extern "C" */
#endif
