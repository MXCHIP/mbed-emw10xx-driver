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
 *  Defines functions to manage the WICED system
 */

#pragma once

#include "wiced_network.h"
#include "wwd_network_interface.h"
#include "wiced_rtos.h"
#include "wiced_tcpip.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                     Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/** @cond !ADDTHIS*/
#define WICED_FACTORY_RESET_MAGIC_VALUE       0xA6C5A54E
/** @endcond */

/******************************************************
 *                   Enumerations
 ******************************************************/

/** IP address configuration options */
typedef enum
{
    WICED_USE_EXTERNAL_DHCP_SERVER, /**< Client interface: use an external DHCP server  */
    WICED_USE_STATIC_IP,            /**< Client interface: use a fixed IP address       */
    WICED_USE_INTERNAL_DHCP_SERVER  /**< softAP interface: use the internal DHCP server */
} wiced_network_config_t;


/** DCT app section configuration item data type */
typedef enum
{
    CONFIG_STRING_DATA,       /**< String data type */
    CONFIG_UINT8_DATA,        /**< uint8 data type  */
    CONFIG_UINT16_DATA,       /**< uint16 data type */
    CONFIG_UINT32_DATA        /**< uint32 data type */
} configuration_data_type_t;


/** WICED Network link subscription types denote whether
 *  to subscribe for link up or link down events */
typedef enum
{
    WICED_LINK_UP_SUBSCRIPTION,     /**< Link up event subscription   */
    WICED_LINK_DOWN_SUBSCRIPTION    /**< Link down event subscription */
} wiced_link_subscription_t;

/** WICED Network link status */
typedef enum
{
    WICED_LINK_UP,   /**< Link status up   */
    WICED_LINK_DOWN  /**< Link status down */
} wiced_link_status_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/** Network link callback */
typedef void (*wiced_network_link_callback_t)(void);

/******************************************************
 *                    Structures
 ******************************************************/

/** IP address settings */
typedef struct
{
    wiced_ip_address_t ip_address;  /**< IP address      */
    wiced_ip_address_t gateway;     /**< Gateway address */
    wiced_ip_address_t netmask;     /**< Netmask         */
} wiced_ip_setting_t;


/** DCT app section configuration item entry */
typedef struct
{
    char*                     name;        /**< Name of the entry              */
    uint32_t                  dct_offset;  /**< Offset of the entry in the DCT */
    uint32_t                  data_size;   /**< Size of the entry              */
    configuration_data_type_t data_type;   /**< Type of the entry              */
} configuration_entry_t;


/** Structure to hold information about a system monitor item */
typedef struct
{
    uint32_t last_update;              /**< Time of the last system monitor update */
    uint32_t longest_permitted_delay;  /**< Longest permitted delay between checkins with the system monitor */
} wiced_system_monitor_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 *
 ******************************************************/

/*****************************************************************************/
/** @defgroup mgmt       Management
 *
 *  WICED Management Functions
 */
/*****************************************************************************/

/** Starts a process to perform an over-the-air (OTA) upgrade
 *
 * @warning The current application immediately terminates when this function is called.
 *  The function does not return, and ALL state for the current application is discarded
 *
 *  @return Does not return!
 */
extern void wiced_start_ota_upgrade( void );


/** Restores the default factory application image
 *
 * This function restores the default factory application and additionally resets
 * all values stored in the DCT to factory defaults. \n
 *
 * Accidental usage of this function is protected by a magic number. If the
 * magic number argument does not match WICED_FACTORY_RESET_MAGIC_VALUE,
 * the function fails and normal program execution continues
 *
 * @warning The current application immediately terminates when this function is called.
 *  The function does not return, and ALL state for the current application is discarded
 *
 * @param[in] magic_value A magic value that must match WICED_FACTORY_RESET_MAGIC_VALUE for the function to complete successfully
 *
 * @return Does not return!
 */
extern void wiced_restore_factory_image( uint32_t magic_value );


/*****************************************************************************/
/** @addtogroup initconf       Initialisation & configuration
 *  @ingroup mgmt
 *
 * Initialisation/Deinitialisation of WICED and device configuration functions
 *
 *  @{
 */
/*****************************************************************************/


/** Initialises the WICED system
 *
 * This function sets up the system by :
 *  \li initialising the platform interface
 *  \li initialising the RTOS & Network Stack
 *  \li initialising the WLAN driver and chip
 *  \li starting the event processing thread
 *
 * @return @ref wiced_result_t
 */
extern wiced_result_t wiced_init( void );


/** De-initialises the WICED system
 *
 * This function de-initialises the WICED system by :
 *  \li bringing down all network interfaces
 *  \li deleting all packet pools
 *  \li tearing down the event thread
 *  \li powering down the WLAN chip
 *
 * @return @ref wiced_result_t
 */
extern wiced_result_t wiced_deinit( void );


/** Enables all powersave features
 *
 *  This is a convenience function that calls each of the powersave related functions listed below \n
 *  Please review the documentation for each function for further information
 *  \li @ref wiced_platform_mcu_enable_powersave()
 *  \li @ref wiced_wifi_enable_powersave()
 *  \li @ref wiced_network_suspend()
 *
 * @return @ref wiced_result_t
 */
extern wiced_result_t wiced_enable_powersave( void );


/** Disables all powersave features
 *
 *  This is a convenience functions that calls each of the powersave related functions listed below \n
 *  Please review the documentation for each function for further information
 *  \li @ref wiced_platform_mcu_disable_powersave()
 *  \li @ref wiced_wifi_disable_powersave()
 *  \li @ref wiced_network_resume()
 *
 * @return WICED_SUCCESS
 */
extern wiced_result_t wiced_disable_powersave( void );


/** Runs device configuration (if required)
 *
 * @param[in] config  : an array of user configurable variables in configuration_entry_t format.
 *                      The array must be terminated with a "null" entry {0,0,0,0}
 *
 * @return    WICED_SUCCESS
 */
extern wiced_result_t wiced_configure_device( const configuration_entry_t* config );


/** Re-runs device configuration
 *
 * @param[in] config  : an array of user configurable variables in configuration_entry_t format.
 *                      The array must be terminated with a "null" entry {0,0,0,0}
 *
 * @return    WICED_SUCCESS
 */
extern wiced_result_t wiced_reconfigure_device( const configuration_entry_t* config );


/** @} */



/*****************************************************************************/
/** @addtogroup netmgmt       Network management
 *  @ingroup mgmt
 *
 * Functions to manage the network interfaces
 *
 *  @{
 */
/*****************************************************************************/

/** Brings up a network interface
 *
 *
 * @param[in] interface     : the interface to bring up
 * @param[in] config        : the network IP configuration
 * @param[in] ip_settings   : static IP settings that are mandatory for the AP interface,
 *                        but are optional for the STA interface
 *
 * @return @ref wiced_result_t
 */
extern wiced_result_t wiced_network_up( wiced_interface_t interface, wiced_network_config_t config, const wiced_ip_setting_t* ip_settings );


/** Brings down a network interface
 *
 * @param[in] interface : the interface to bring down
 *
 * @return @ref wiced_result_t
 */
extern wiced_result_t wiced_network_down( wiced_interface_t interface );


/** Suspends network services and disables all network related timers
 *
 * This function must only be called before entering deep sleep. Prior to calling this function, ensure all
 * network sockets are in a disconnected state. After calling this function, networking functions
 * should not be used. To resume network operation, use the @ref wiced_network_resume() function.
 *
 * Example usage:
 *   @code
 *      wiced_network_suspend();
 *      wiced_rtos_delay_milliseconds(DEEP_SLEEP_TIME);
 *      wiced_network_resume();
 *   @endcode
 *
 * @return    WICED_SUCCESS : Network services are suspended.
 *            WICED_ERROR   : Network services were unable to be suspended, active sockets still exist
 */
extern wiced_result_t wiced_network_suspend( void );


/** Resumes network services
 *
 * This function resumes network services after suspension
 * with the wiced_network_suspend() function. After calling this function, all network functions
 * are available for use.
 *
 * Example usage:
 *   @code
 *      wiced_network_suspend();
 *      wiced_rtos_delay_milliseconds(DEEP_SLEEP_TIME);
 *      wiced_network_resume();
 *   @endcode
 *
 * @return @ref wiced_result_t
 */
extern wiced_result_t wiced_network_resume( void );

extern wiced_result_t wiced_ip_up         ( wiced_interface_t interface, wiced_network_config_t config, const wiced_ip_setting_t* ip_settings );
extern wiced_result_t wiced_ip_down( wiced_interface_t interface );

/** Checks if a network interface is up
 *
 * @param[in] interface : the interface to check
 *
 * @return @ref wiced_bool_t
 */
extern wiced_bool_t wiced_network_is_up( wiced_interface_t interface );

/** Register callback function/s that gets called when a change in network link status occurs
 *
 * @param link_up_callback   : the optional callback function to register for the link up event
 * @param link_down_callback : the optional callback function to register for the link down event
 *
 * @return    WICED_SUCCESS : on success.
 * @return    WICED_ERROR   : if an error occurred with any step
 */
extern wiced_result_t wiced_network_register_link_callback( wiced_network_link_callback_t link_up_callback, wiced_network_link_callback_t link_down_callback );

/** De-register network link status callback function/s
 *
 * @param link_up_callback   : the optional callback function to deregister for the link up event
 * @param link_down_callback : the optional callback function to deregister for the link down event
 *
 * @return @ref wiced_result_t
 */
extern wiced_result_t wiced_network_deregister_link_callback( wiced_network_link_callback_t link_up_callback, wiced_network_link_callback_t link_down_callback );


/** @} */

/*****************************************************************************/
/** @addtogroup sysmon       System Monitor
 *  @ingroup mgmt
 *
 * Functions to communicate with the system monitor
 *
 *  @{
 */
/*****************************************************************************/

/** Registers a system monitor with the system monitor thread
 *
 * @param[out] system_monitor          : A pointer to a system monitor object that will be watched
 * @param[in]  initial_permitted_delay : The maximum time in milliseconds allowed between monitor updates
 *
 * @return @ref wiced_result_t
 */
extern wiced_result_t wiced_register_system_monitor(wiced_system_monitor_t* system_monitor, uint32_t initial_permitted_delay);

/** Updates a system monitor and resets the last update time
 *
 * @param[out] system_monitor  : A pointer to a system monitor object to be updated
 * @param[in]  permitted_delay : The maximum time in milliseconds allowed between monitor updates
 *
 * @return @ref wiced_result_t
 */
extern wiced_result_t wiced_update_system_monitor(wiced_system_monitor_t* system_monitor, uint32_t permitted_delay);

extern void wiced_set_hostname(char *name);
extern void wiced_stop_dhcp(void);

/** @} */

#ifdef __cplusplus
} /*extern "C" */
#endif
