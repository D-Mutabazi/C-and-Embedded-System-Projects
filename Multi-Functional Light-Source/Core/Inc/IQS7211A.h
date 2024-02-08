/**
  ************************************************************************************
  * @file    IQS7211A.h
  * @author  Azoteq
  * @version V1.0.0
  * @date    2021-07-06
  * @brief   This file contains the header information for an IQS7211A Arduino library.
  *          The goal of the library is to provide easy functionality for 
  *          initializing and using the Azoteq IQS7211A capacitive touch device.
  ************************************************************************************
  ************************************************************************************
  * @attention  Makes use of the following standard Arduino libraries:
  *       - Arduino.h   -> included in IQS7211A.h, comes standard with Arduino
  *       - Wire.h      -> Included in IQS7211A.h, comes standard with Arduino
  *
  ************************************************************************************
  */

#ifndef IQS7211A_h
#define IQS7211A_h

// Include Files
#include "Arduino.h"
#include "Wire.h"
#include "./inc/iqs7211a_addresses.h"

// Public Global Definitions
#define STOP    true    // For use with Wire.h library. True argument with some functions closes the I2C communication window.
#define RESTART false   // For use with Wire.h library. False argument with some functions keeps the I2C communication window open.

//Device Info
#define IQS7211A_PRODUCT_NUM            0x02FB
#define MAJOR_VERSION_NUM	        0x01
#define MINOR_VERSION_NUM	        0x00
#define BUILD_NUM			0xC506F297

// Info Flags Byte Bits.
#define ATI_ERROR_BIT		        0x08
#define REATI_OCCURED_BIT		0x10
#define ALP_ATI_ERROR_BIT               0x20
#define ALP_REATI_OCCURRED_BIT		0x40
#define SHOW_RESET_BIT		        0x80

// Utility Bits
#define ACK_RESET_BIT		        0x80
#define SW_RESET_BIT		        0x02
#define TP_REATI_BIT                    0x20
#define EVENT_MODE_BIT                  0x01
#define GESTURE_EVENT_BIT               0x02
#define TP_EVENT_BIT                    0x04
#define COMMS_REQ_EN_BIT                0x10


#define FINGER_1                         1
#define FINGER_2                         2

// Type Definitions.
/* Infoflags - address 0x10 - Read Only */
typedef union
        {
                struct
                {
                        union
                        {
                                struct
                                {
                                        uint8_t charging_mode        : 3;
                                        uint8_t ati_error            : 1;
                                        uint8_t reati_occurred       : 1;
                                        uint8_t alp_ati_error        : 1;
                                        uint8_t alp_reati_occurred   : 1;
                                        uint8_t show_reset           : 1;
                                };
                                uint8_t iqs7211a_infoflags_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t no_fingers         : 2;
                                        uint8_t tp_movement        : 1;
                                        uint8_t infoflags_res1     : 1;
                                        uint8_t too_many_fingers   : 1;
                                        uint8_t infoflags_res2     : 1;
                                        uint8_t alp_prox           : 1;
                                        uint8_t alp_touch          : 1;
                                };
                                uint8_t iqs7211a_infoflags_msb;
                        };


                };
                uint8_t buffer[2];
}  IQS7211A_INFO_FLAGS;

/* Gesture Events - address 0x11 - Read Only */
typedef union
        {
                struct
                {
                        union
                        {
                                struct
                                {
                                        uint8_t single_tap          : 1;
                                        uint8_t press_and_hold      : 1;
                                        uint8_t swipe_x_neg         : 1;
                                        uint8_t swipe_x_pos         : 1;
                                        uint8_t swipe_y_pos         : 1;
                                        uint8_t swipe_y_neg         : 1;
                                        uint8_t gesture_events_res1 : 2; 
                                };
                                uint8_t iqs7211a_gesture_events_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t gesture_events_res2         : 8;
                   
                                };
                                uint8_t iqs7211a_gesture_events_msb;
                        };


                };
                uint8_t buffer[2];
}  IQS7211A_GESTURE_EVENTS;

/* Coordinate info - address 0x12 - 0x1B - Read Only */
typedef union
        {
                struct
                {
                        /* Relative X Coordinate */
                        union
                        {
                                struct
                                {
                                        uint8_t relative_x_lsb          : 8;
                                };
                                uint8_t iqs7211a_relative_x_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t relative_x_msb         : 8;
                   
                                };
                                uint8_t iqs7211a_relative_x_msb;
                        };

                        /* Relative Y Coordinate */
                        union
                        {
                                struct
                                {
                                        uint8_t relative_y_lsb          : 8;
                                };
                                uint8_t iqs7211a_relative_y_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t relative_y_msb         : 8;
                   
                                };
                                uint8_t iqs7211a_relative_y_msb;
                        };

                        /* Finger 1 X Coordinate */
                        union
                        {
                                struct
                                {
                                        uint8_t f1_x_lsb          : 8;
                                };
                                uint8_t iqs7211a_f1_x_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t f1_x_msb         : 8;
                   
                                };
                                uint8_t iqs7211a_f1_x_msb;
                        };

                        /* Finger 1 Y Coordinate */
                        union
                        {
                                struct
                                {
                                        uint8_t f1_y_lsb          : 8;
                                };
                                uint8_t iqs7211a_f1_y_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t f1_y_msb         : 8;
                   
                                };
                                uint8_t iqs7211a_f1_y_msb;
                        };

                        /* Finger 1 Touch Strength */
                        union
                        {
                                struct
                                {
                                        uint8_t f1_ts_lsb          : 8;
                                };
                                uint8_t iqs7211a_f1_ts_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t f1_ts_msb         : 8;
                   
                                };
                                uint8_t iqs7211a_f1_ts_msb;
                        };

                        
                        /* Finger 1 Area*/
                        union
                        {
                                struct
                                {
                                        uint8_t f1_area_lsb          : 8;
                                };
                                uint8_t iqs7211a_f1_area_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t f1_area_msb         : 8;
                   
                                };
                                uint8_t iqs7211a_f1_area_msb;
                        };

                        /* Finger 2 X Coordinate */
                        union
                        {
                                struct
                                {
                                        uint8_t f2_x_lsb          : 8;
                                };
                                uint8_t iqs7211a_f2_x_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t f2_x_msb         : 8;
                   
                                };
                                uint8_t iqs7211a_f2_x_msb;
                        };

                        /* Finger 2 Y Coordinate */
                        union
                        {
                                struct
                                {
                                        uint8_t f2_y_lsb          : 8;
                                };
                                uint8_t iqs7211a_f2_y_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t f2_y_msb         : 8;
                   
                                };
                                uint8_t iqs7211a_f2_y_msb;
                        };

                        /* Finger 2 Touch Strength */
                        union
                        {
                                struct
                                {
                                        uint8_t f2_ts_lsb          : 8;
                                };
                                uint8_t iqs7211a_f2_ts_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t f2_ts_msb         : 8;
                   
                                };
                                uint8_t iqs7211a_f2_ts_msb;
                        };

                        
                        /* Finger 2 Area*/
                        union
                        {
                                struct
                                {
                                        uint8_t f2_area_lsb          : 8;
                                };
                                uint8_t iqs7211a_f2_area_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t f2_area_msb         : 8;
                   
                                };
                                uint8_t iqs7211a_f2_area_msb;
                        };


                };
                uint8_t buffer[20];
}  IQS7211A_COORD_INFO;


/* Channel Touch Status and ALP Counts - address 0x20 -0x28 - Read Only */
typedef union
        {
                struct
                {
                        /* ALP Channel Counts */
                        union
                        {
                                struct
                                {
                                        uint8_t alp_cts_lsb          : 1;

                                };
                                uint8_t iqs7211a_alp_cts_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t alp_cts_msb         : 8;
                   
                                };
                                uint8_t iqs7211a_alp_cts_msb;
                        };

                        /* ALP Channel LTA */
                        union
                        {
                                struct
                                {
                                        uint8_t alp_lta_lsb          : 1;

                                };
                                uint8_t iqs7211a_alp_lta_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t alp_lta_msb         : 8;
                   
                                };
                                uint8_t iqs7211a_alp_lta_msb;
                        };

                        /* Touch Status Row0 */
                        union
                        {
                                struct
                                {
                                        uint8_t touch_stat_row0_lsb          : 1;

                                };
                                uint8_t iqs7211a_touch_stat_row0_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t touch_stat_row0_msb         : 8;
                   
                                };
                                uint8_t iqs7211a_touch_stat_row0_msb;
                        };

                        /* Touch Status Row1 */
                        union
                        {
                                struct
                                {
                                        uint8_t touch_stat_row1_lsb          : 1;

                                };
                                uint8_t iqs7211a_touch_stat_row1_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t touch_stat_row1_msb         : 8;
                   
                                };
                                uint8_t iqs7211a_touch_stat_row1_msb;
                        };

                        /* Touch Status Row2 */
                        union
                        {
                                struct
                                {
                                        uint8_t touch_stat_row2_lsb          : 1;

                                };
                                uint8_t iqs7211a_touch_stat_row2_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t touch_stat_row2_msb         : 8;
                   
                                };
                                uint8_t iqs7211a_touch_stat_row2_msb;
                        };

                        /* Touch Status Row3 */
                        union
                        {
                                struct
                                {
                                        uint8_t touch_stat_row3_lsb          : 1;

                                };
                                uint8_t iqs7211a_touch_stat_row3_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t touch_stat_row3_msb         : 8;
                   
                                };
                                uint8_t iqs7211atouch_stat_row3_msb;
                        };

                        /* Touch Status Row4 */
                        union
                        {
                                struct
                                {
                                        uint8_t touch_stat_row4_lsb          : 1;

                                };
                                uint8_t iqs7211a_touch_stat_row4_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t touch_stat_row4_msb         : 8;
                   
                                };
                                uint8_t iqs7211a_touch_stat_row4_msb;
                        };

                        /* Touch Status Row5 */
                        union
                        {
                                struct
                                {
                                        uint8_t touch_stat_row5_lsb          : 1;

                                };
                                uint8_t iqs7211a_touch_stat_row5_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t touch_stat_row5_msb         : 8;
                   
                                };
                                uint8_t iqs7211a_touch_stat_row5_msb;
                        };

                        /* Touch Status Row6 */
                        union
                        {
                                struct
                                {
                                        uint8_t touch_stat_row6_lsb          : 1;

                                };
                                uint8_t iqs7211a_touch_stat_row6_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t touch_stat_row6_msb         : 8;
                   
                                };
                                uint8_t iqs7211a_touch_stat_row6_msb;
                        };


                };
                uint8_t buffer[18];
}  IQS7211A_TOUCH_STAT_ALP_CTS;


/* Trackpad Prox Settings - address 0x30 - 0x35 */
typedef union
        {
                struct
                {
                        /* TP ATI Mirrors */
                        union
                        {
                                struct
                                {
                                        uint8_t tp_nmir_div      : 5;
                                        uint8_t tp_nmir_mult_l   : 3;
                                };
                                uint8_t iqs7211a_tp_ati_mirrors_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t tp_nmir_mult_h    : 1;
                                        uint8_t tp_pmir_div       : 5;
                                        uint8_t tp_ati_mir_res1   : 2;
                                };
                                uint8_t iqs7211a_tp_ati_mirrors_msb;
                        };

                        /* TP PCC DIV */
                        union
                        {
                                struct
                                {
                                        uint8_t tp_pcc_div_lsb    : 8;
                                };
                                uint8_t iqs7211a_tp_pcc_div_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t tp_pcc_div_msb    : 8;
                                };
                                uint8_t iqs7211a_tp_pcc_div_msb;
                        };

                        /* Trackpad ATI Target */
                        union
                        {
                                struct
                                {
                                        uint8_t tp_ati_target_lsb         : 8;
                                };
                                uint8_t iqs7211a_tp_ati_target_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t tp_ati_target_msb         : 8;
                                };
                                uint8_t iqs7211a_tp_ati_target_msb;
                        };

                        /* Trackpad reference drift limit */
                        union
                        {
                                struct
                                {
                                        uint8_t tp_ref_drift_limit_lsb         : 8;
                                };
                                uint8_t iqs7211a_tp_ref_drift_limit_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t tp_ref_drift_limit_msb         : 8;
                                };
                                uint8_t iqs7211a_tp_ref_drift_limit_msb;
                        };

                        /* min count re-ATI value */
                        union
                        {
                                struct
                                {
                                        uint8_t tp_min_cts_ati_lsb         : 8;
                                };
                                uint8_t iqs7211a_tp_min_cts_ati_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t tp_min_cts_ati_msb         : 8;
                                };
                                uint8_t iqs7211a_tp_min_cts_ati_msb;
                        };

                        /* Re-ATI retry Time */
                        union
                        {
                                struct
                                {
                                        uint8_t re_ati_retry_time_lsb         : 8;
                                };
                                uint8_t iqs7211a_re_ati_retry_time_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t re_ati_retry_time_msb         : 8;
                                };
                                uint8_t iqs7211a_re_ati_retry_time_msb;
                        };


                };
                uint8_t buffer[12];
}  IQS7211A_TP_PXS_SETTINGS;


/* ALP Prox Settings - address 0x36 - 0x3A */
typedef union
        {
                struct
                {

                        /* ALP ATI Mirrors */
                        union
                        {
                                struct
                                {
                                        uint8_t alp_nmir_div      : 5;
                                        uint8_t alp_nmir_mult_l   : 3;
                                };
                                uint8_t iqs7211a_alp_ati_mirrors_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t alp_nmir_mult_h    : 1;
                                        uint8_t alp_pmir_div       : 5;
                                        uint8_t alp_ati_mir_res1   : 2;
                                };
                                uint8_t iqs7211a_alp_ati_mirrors_msb;
                        };

                        /* ALP PCC DIV */
                        union
                        {
                                struct
                                {
                                        uint8_t alp_pcc_div_lsb    : 8;
                                };
                                uint8_t iqs7211a_alp_pcc_div_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t alp_pcc_div_msb    : 8;
                                };
                                uint8_t iqs7211a_alp_pcc_div_msb;
                        };

                        /* ALP ATI Target */
                        union
                        {
                                struct
                                {
                                        uint8_t alp_ati_target_lsb         : 8;
                                };
                                uint8_t iqs7211a_alp_ati_target_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t alp_ati_target_msb         : 8;
                                };
                                uint8_t iqs7211a_alp_ati_target_msb;
                        };

                        /* ALP LTA Drift Limit */
                        union
                        {
                                struct
                                {
                                        uint8_t alp_lta_drift_limit_lsb         : 8;
                                };
                                uint8_t iqs7211a_alp_lta_drift_limit_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t alp_lta_drift_limit_msb         : 8;
                                };
                                uint8_t iqs7211a_alp_lta_drift_limit_msb;
                        };

                        /* ALP PCC */
                        union
                        {
                                struct
                                {
                                        uint8_t alp_pcc_lsb         : 8;
                                };
                                uint8_t iqs7211a_alp_pcc_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t alp_pcc_msb         : 8;
                                };
                                uint8_t iqs7211a_alp_pcc_msb;
                        };
                };
                uint8_t buffer[10];
}  IQS7211A_ALP_PXS_SETTINGS;

/* Report Rates and Timeouts - address 0x40 - 0x4A */
typedef union
        {
                struct
                {
                        /* Active Mode Report Rate */
                        union
                        {
                                struct
                                {
                                        uint8_t active_rr_lsb   : 8;
                                };
                                uint8_t iqs7211a_active_rr_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t active_rr_msb    : 8;
                                };
                                uint8_t iqs7211a_active_rr_msb;
                        };

                        /* Idle-Touch Mode Report Rate */
                        union
                        {
                                struct
                                {
                                        uint8_t idle_touch_rr_lsb   : 8;
                                };
                                uint8_t iqs7211a_idle_touch_rr_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t idle_touch_rr_msb    : 8;
                                };
                                uint8_t iqs7211a_idle_touch_rr_msb;
                        };

                        /* Idle Mode Report Rate */
                        union
                        {
                                struct
                                {
                                        uint8_t idle_rr_lsb   : 8;
                                };
                                uint8_t iqs7211a_idle_rr_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t idle_rr_msb    : 8;
                                };
                                uint8_t iqs7211a_idle_rr_msb;
                        };


                        /* LP1 Report Rate */
                        union
                        {
                                struct
                                {
                                        uint8_t lp1_rr_lsb   : 8;
                                };
                                uint8_t iqs7211a_lp1_rr_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t lp1_rr_msb    : 8;
                                };
                                uint8_t iqs7211a_lp1_rr_msb;
                        };

                        /* LP2 Report Rate */
                        union
                        {
                                struct
                                {
                                        uint8_t lp2_rr_lsb   : 8;
                                };
                                uint8_t iqs7211a_lp2_rr_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t lp2_rr_msb    : 8;
                                };
                                uint8_t iqs7211a_lp2_rr_msb;
                        };

                        /* Active Mode Timeout */
                        union
                        {
                                struct
                                {
                                        uint8_t active_timout_lsb   : 8;
                                };
                                uint8_t iqs7211a_active_timout_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t active_timout_msb    : 8;
                                };
                                uint8_t iqs7211a_active_timout_msb;
                        };

                        /* Idle-Touch Mode Timeout */
                        union
                        {
                                struct
                                {
                                        uint8_t idle_touch_timout_lsb   : 8;
                                };
                                uint8_t iqs7211a_idle_touch_timout_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t idle_touch_timout_msb    : 8;
                                };
                                uint8_t iqs7211a_idle_touch_timout_msb;
                        };

                        /* Idle Mode Timeout */
                        union
                        {
                                struct
                                {
                                        uint8_t idle_timout_lsb   : 8;
                                };
                                uint8_t iqs7211a_idle_timout_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t idle_timout_msb    : 8;
                                };
                                uint8_t iqs7211a_idle_timout_msb;
                        };

                        /* LP 1 Mode Timeout */
                        union
                        {
                                struct
                                {
                                        uint8_t lp1_timout_lsb   : 8;
                                };
                                uint8_t iqs7211a_lp1_timout_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t lp1_timout_msb    : 8;
                                };
                                uint8_t iqs7211a_lp1_timout_msb;
                        };

                        /* Reference Update Time*/
                        union
                        {
                                struct
                                {
                                        uint8_t ref_update_time_lsb   : 8;
                                };
                                uint8_t iqs7211a_ref_update_time_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t ref_update_time_msb    : 8;
                                };
                                uint8_t iqs7211a_ref_update_time_msb;
                        };

                        /* I2C Timeout */
                        union
                        {
                                struct
                                {
                                        uint8_t i2c_timout_lsb   : 8;
                                };
                                uint8_t iqs7211a_i2c_timout_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t i2c_timout_msb    : 8;
                                };
                                uint8_t iqs7211a_i2c_timout_msb;
                        };

                };
                uint8_t buffer[22];
}  IQS7211A_RR_TIMEOUTS;


/* System Control and other settings Addresses from 0x50 to 0x5B */
typedef union
                {
                        struct
                        {
                                /* System Control*/
                                union
                                {
                                        struct
                                        {
                                                uint8_t mode_select                     : 3;
                                                uint8_t tp_reseed                       : 1;
                                                uint8_t alp_reseed                      : 1;
                                                uint8_t tp_re_ati                       : 1;
                                                uint8_t alp_re_ati                      : 1;
                                                uint8_t ack_reset                       : 1;

                                        };
                                        uint8_t iqs7211a_system_control_lsb;
                                };

                                union
                                {
                                        struct
                                        {
                                                uint8_t system_control_res1       : 1;
                                                uint8_t sw_reset                  : 1;
                                                uint8_t system_control_res2       : 6;
                                        };
                                        uint8_t iqs7211a_system_control_msb;
                                };

                                /* Config Settings*/
                                union
                                {
                                        struct
                                        {
                                                uint8_t config_settings_res0     : 2;
                                                uint8_t tp_re_ati_enable         : 1;
                                                uint8_t alp_re_ati_enable        : 1;
                                                uint8_t config_settings_res1     : 3;
                                                uint8_t manual_control           : 1;
                                        };
                                        uint8_t iqs7211a_config_settings_lsb;
                                };

                                union
                                {
                                        struct
                                        {
                                                uint8_t event_mode                       : 1;
                                                uint8_t gesture_event                    : 1;
                                                uint8_t tp_event                         : 1;
                                                uint8_t re_ati_event                     : 1;
                                                uint8_t config_settings_res2             : 1;
                                                uint8_t alp_event                        : 1;
                                                uint8_t tp_touch_event                   : 1;
                                                uint8_t config_settings_res3             : 1;
                                        };
                                        uint8_t iqs7211a_config_settings_msb;
                                };

                                /* Other Settings*/
                                union
                                {
                                        struct
                                        {
                                                uint8_t auto_prox_cycles        : 2;
                                                uint8_t other_settings_res0     : 1;
                                                uint8_t alp_count_filter        : 1;
                                                uint8_t other_settings_res1     : 4;

                                        };
                                        uint8_t iqs7211a_other_settings_lsb;
                                };

                                union
                                {
                                        struct
                                        {
                                                uint8_t tp_max_counts           : 2;
                                                uint8_t alp_max_counts          : 2;
                                                uint8_t other_settings_res2     : 4;

                                        };
                                        uint8_t iqs7211a_other_settings_msb;
                                };

                                /* Trackpad Touch Threshold (set and clear) */


                                union
                                {
                                        struct
                                        {
                                                uint8_t tp_touch_set_threshold                   : 8;
                                        };
                                        uint8_t iqs7211a_tp_touch_set_threshold;
                                };

                                union
                                {
                                        struct
                                        {
                                                uint8_t tp_touch_clear_threshold                 : 8;
                                        };
                                        uint8_t iqs7211a_tp_touch_clear_threshold;
                                };

                                /* ALP Prox Threshold*/

                                union
                                {
                                        struct
                                        {
                                                uint8_t alp_prox_threshold_lsb                   : 8;
                                        };
                                        uint8_t iqs7211a_alp_prox_threshold_lsb;
                                };

                                union
                                {
                                        struct
                                        {
                                                uint8_t alp_prox_threshold_msb                   : 8;
                                        };
                                        uint8_t iqs7211a_alp_prox_threshold_msb;
                                };

                                
                                /* ALP Touch Threshold */

                                union
                                {
                                        struct
                                        {
                                                uint8_t alp_touch_threshold_lsb                   : 8;
                                        };
                                        uint8_t iqs7211a_alp_touch_threshold_lsb;
                                };

                                union
                                {
                                        struct
                                        {
                                                uint8_t alp_touch_threshold_msb                   : 8;
                                        };
                                        uint8_t iqs7211a_alp_touch_threshold_msb;
                                };

                                /* ALP Prox Debounce*/

                                union
                                {
                                        struct
                                        {
                                                uint8_t alp_prox_debounce_lsb                   : 8;
                                        };
                                        uint8_t iqs7211a_alp_prox_debounce_lsb;
                                };

                                union
                                {
                                        struct
                                        {
                                                uint8_t alp_prox_debounce_msb                   : 8;
                                        };
                                        uint8_t iqs7211a_alp_prox_debounce_msb;
                                };



                                /* ALP Touch Debounce*/

                                union
                                {
                                        struct
                                        {
                                                uint8_t alp_touch_debounce_lsb                   : 8;
                                        };
                                        uint8_t iqs7211a_alp_touch_debounce_lsb;
                                };

                                union
                                {
                                        struct
                                        {
                                                uint8_t alp_touch_debounce_msb                   : 8;
                                        };
                                        uint8_t iqs7211a_alp_touch_debounce_msb;
                                };

                                /* TP Conversion Frequency */

                                union
                                {
                                        struct
                                        {
                                                uint8_t tp_up_pass_length               : 8;
                                        };
                                        uint8_t iqs7211a_tp_up_pass_length;
                                };

                                union
                                {
                                        struct
                                        {
                                                uint8_t tp_freq_frac               : 8;
                                        };
                                        uint8_t iqs7211a_tp_freq_frac;
                                };

                                /* ALP Conversion  Frequency */

                                union
                                {
                                        struct
                                        {
                                                uint8_t alp_up_pass_length                  : 8;
                                        };
                                        uint8_t iqs7211a_alp_up_pass_length;
                                };

                                union
                                {
                                        struct
                                        {
                                                uint8_t alp_freq_frac                : 8;
                                        };
                                        uint8_t iqs7211a_alp_freq_frac;
                                };

                                /* HW Settings */

                                union
                                {
                                        struct
                                        {
                                                uint8_t main_osc_adjust                 : 4;
                                                uint8_t fosc_select                     : 1;
                                                uint8_t cal_cap_size               	 	: 3;
                                        };
                                        uint8_t iqs7211a_hw_settings_lsb;
                                };

                                union
                                {
                                        struct
                                        {
                                                uint8_t cal_ch_select                : 8;
                                        };
                                        uint8_t iqs7211a_hw_settings_msb;
                                };

                        };
                        uint8_t buffer[22];
}  IQS7211A_SYS_CONFIG;


/* Filtering and TP Settings Addresses from 0x60 to 0x69 */
typedef union
                {
                        struct
                        {
                                /* Trackpad Settings 0 */
                                union
                                {
                                        struct
                                        {
                                                uint8_t tp_flip_x               : 1;
                                                uint8_t tp_flip_y               : 1;
                                                uint8_t switch_xy_axis          : 1;
                                                uint8_t iir_filter              : 1;
                                                uint8_t iir_static              : 1;
                                                uint8_t mav_filter              : 1;
                                                uint8_t tp_settings_0_res1      : 2;


                                        };
                                        uint8_t iqs7211a_tp_settings_0_lsb;
                                };

                                union
                                {
                                        struct
                                        {
                                                uint8_t total_rxs                     : 8;


                                        };
                                        uint8_t iqs7211a_tp_settings_0_msb;
                                };

                                /* Trackpad Settings 1 */
                                union
                                {
                                        struct
                                        {
                                                uint8_t total_txs                     : 8;


                                        };
                                        uint8_t iqs7211a_tp_settings_1_lsb;
                                };

                                union
                                {
                                        struct
                                        {
                                                uint8_t number_of_fingers                     : 8;


                                        };
                                        uint8_t iqs7211a_tp_settings_1_msb;
                                };

                                /* x resolution */
                                union
                                {
                                        struct
                                        {
                                                uint8_t x_resolution_lsb                     : 8;


                                        };
                                        uint8_t iqs7211a_x_resolution_lsb;
                                };

                                union
                                {
                                        struct
                                        {
                                                uint8_t x_resolution_msb                     : 8;


                                        };
                                        uint8_t iqs7211a_x_resolution_msb;
                                };

                                /* y resolution */
                                union
                                {
                                        struct
                                        {
                                                uint8_t y_resolution_lsb                     : 8;


                                        };
                                        uint8_t iqs7211a_y_resolution_lsb;
                                };

                                union
                                {
                                        struct
                                        {
                                                uint8_t y_resolution_msb                     : 8;


                                        };
                                        uint8_t iqs7211a_y_resolution_msb;
                                };

                                /* XY Dynamic Filter - bottom speed */
                                union
                                {
                                        struct
                                        {
                                                uint8_t xy_dyn_bottom_speed_lsb                     : 8;


                                        };
                                        uint8_t iqs7211a_xy_dyn_bottom_speed_lsb;
                                };

                                union
                                {
                                        struct
                                        {
                                                uint8_t xy_dyn_bottom_speed_msb                     : 8;


                                        };
                                        uint8_t iqs7211a_xy_dyn_bottom_speed_msb;
                                };

                                /* XY Dynamic Filter - top speed */
                                union
                                {
                                        struct
                                        {
                                                uint8_t xy_dyn_top_speed_lsb                     : 8;


                                        };
                                        uint8_t iqs7211a_xy_dyn_top_speed_lsb;
                                };

                                union
                                {
                                        struct
                                        {
                                                uint8_t xy_dyn_top_speed_msb                     : 8;


                                        };
                                        uint8_t iqs7211a_xy_dyn_top_speed_msb;
                                };

                                /* XY Dynamic Filter - Bottom Beta and Static Beta */
                                union
                                {
                                        struct
                                        {
                                                uint8_t xy_dyn_bottom_beta                    : 8;


                                        };
                                        uint8_t iqs7211a_xy_dyn_bottom_beta;
                                };

                                union
                                {
                                        struct
                                        {
                                                uint8_t xy_dyn_static_beta                    : 8;


                                        };
                                        uint8_t iqs7211a_xy_dyn_static_beta;
                                };

                                /* Travel Threshold & Finger Split Factor */
                                union
                                {
                                        struct
                                        {
                                                uint8_t travel_thresh_split_lsb                    : 8;


                                        };
                                        uint8_t iqs7211a_travel_thresh_split_lsb;
                                };

                                union
                                {
                                        struct
                                        {
                                                uint8_t travel_thresh_split_msb                    : 8;


                                        };
                                        uint8_t iqs7211a_travel_thresh_split_msb;
                                };

                                /* X Trim value */
                                union
                                {
                                        struct
                                        {
                                                uint8_t x_trim_lsb                    : 8;


                                        };
                                        uint8_t iqs7211a_x_trim_lsb;
                                };

                                union
                                {
                                        struct
                                        {
                                                uint8_t x_trim_msb                    : 8;


                                        };
                                        uint8_t iqs7211a_x_trim_msb;
                                };

                                /* Y Trim value */
                                union
                                {
                                        struct
                                        {
                                                uint8_t y_trim_lsb                    : 8;


                                        };
                                        uint8_t iqs7211a_y_trim_lsb;
                                };

                                union
                                {
                                        struct
                                        {
                                                uint8_t y_trim_msb                    : 8;


                                        };
                                        uint8_t iqs7211a_y_trim_msb;
                                };




                        };
                        uint8_t buffer[20];
}  IQS7211A_FILTERING;



typedef union
        {
                struct
                {
                        /* Gesture Enable */
                        union
                        {
                                struct
                                {
                                        uint8_t single_tap_en          : 1;
                                        uint8_t press_and_hold_en      : 1;
                                        uint8_t swipe_x_neg_en         : 1;
                                        uint8_t swipe_x_pos_en         : 1;
                                        uint8_t swipe_y_pos_en         : 1;
                                        uint8_t swipe_y_neg_en         : 1;
                                        uint8_t gesture_events_res1    : 2;
                                };
                                uint8_t iqs7211a_gesture_en_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t gesture_en_msb    : 8;
                                };
                                uint8_t iqs7211a_gesture_en_msb;
                        };

                        /* Tap Time */
                        union
                        {
                                struct
                                {
                                        uint8_t tap_time_lsb    : 8;
                                };
                                uint8_t iqs7211a_tap_time_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t tap_time_msb    : 8;
                                };
                                uint8_t iqs7211a_tap_time_msb;
                        };

                        /* Tap Distance */
                        union
                        {
                                struct
                                {
                                        uint8_t tap_distance_lsb         : 8;
                                };
                                uint8_t iqs7211a_tap_distance_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t tap_distance_msb         : 8;
                                };
                                uint8_t iqs7211a_tap_distance_msb;
                        };

                        /* Hold Time */
                        union
                        {
                                struct
                                {
                                        uint8_t hold_time_lsb         : 8;
                                };
                                uint8_t iqs7211a_hold_time_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t hold_time_msb         : 8;
                                };
                                uint8_t iqs7211a_hold_time_msb;
                        };

                        /* Swipe Initial Time */
                        union
                        {
                                struct
                                {
                                        uint8_t swipe_init_time_lsb         : 8;
                                };
                                uint8_t iqs7211a_swipe_init_time_lsb;
                        };

                     
                        union
                        {
                                struct
                                {
                                        uint8_t swipe_init_time_msb         : 8;
                                };
                                uint8_t iqs7211a_swipe_init_time_msb;
                        };

                        /* Swipe Initial X Distance */
                        union
                        {
                                struct
                                {
                                        uint8_t swipe_init_x_distance_lsb         : 8;
                                };
                                uint8_t iqs7211a_swipe_init_x_distance_lsb;
                        };

                       
                        union
                        {
                                struct
                                {
                                        uint8_t swipe_init_x_distance_msb         : 8;
                                };
                                uint8_t iqs7211a_swipe_init_x_distance_msb;
                        };

                        /* Swipe Initial Y Distance */
                        union
                        {
                                struct
                                {
                                        uint8_t swipe_init_y_distance_lsb         : 8;
                                };
                                uint8_t iqs7211a_swipe_init_y_distance_lsb;
                        };

                        
                        union
                        {
                                struct
                                {
                                        uint8_t swipe_init_y_distance_msb         : 8;
                                };
                                uint8_t iqs7211a_swipe_init_y_distance_msb;
                        };

                        /* Swipe Angle */
                        union
                        {
                                struct
                                {
                                        uint8_t swipe_angle_lsb         : 8;
                                };
                                uint8_t iqs7211a_swipe_angle_lsb;
                        };

                        union
                        {
                                struct
                                {
                                        uint8_t swipe_angle_msb         : 8;
                                };
                                uint8_t iqs7211a_swipe_angle_msb;
                        };

                };
                uint8_t buffer[16];
}  IQS7211A_GESTURE_SETTINGS;




typedef union
                {
                        struct
                        {
                                 IQS7211A_INFO_FLAGS                         iqs7211a_info_flags;
                                 IQS7211A_GESTURE_EVENTS                     iqs7211a_gesture_events;
                                 IQS7211A_COORD_INFO                         iqs7211a_coord_info;
                                 IQS7211A_TOUCH_STAT_ALP_CTS                 iqs7211a_touch_stat_alp_cts;
                                 IQS7211A_TP_PXS_SETTINGS                    iqs7211a_tp_pxs_settings;
                                 IQS7211A_ALP_PXS_SETTINGS                   iqs7211a_alp_pxs_settings;
                                 IQS7211A_RR_TIMEOUTS                        iqs7211a_rr_timeouts_settings;
                                 IQS7211A_SYS_CONFIG                         iqs7211a_sys_config_settings;
                                 IQS7211A_FILTERING                          iqs7211a_filtering_settings;
                                 IQS7211A_GESTURE_SETTINGS                   iqs7211a_gesture_settings;
                        };
                        uint8_t buffer[144];
}  IQS7211A_MEMORY_MAP;

// Class Prototype
class IQS7211A
{
public:
        // Public Constructors
        IQS7211A();

        // Public Variables
        IQS7211A_MEMORY_MAP IQSMemoryMap;

        // Public Methods
        bool begin(uint8_t deviceAddressIn, uint8_t readyPinIn, uint8_t mclrPinIn);
        bool waitForReady(void);
        uint16_t getProductNum(bool stopOrRestart);
        uint8_t getSoftwareMajorNum(bool stopOrRestart);
        uint8_t getSoftwareMinorNum(bool stopOrRestart);
        void acknowledgeReset(bool stopOrRestart);
        void TP_ReATI(bool stopOrRestart);
        void SW_Reset(bool stopOrRestart);
        void writeMM(bool stopOrRestart);
        void updateAbsCoordinates(bool stopOrRestart, uint8_t fingerNum);
        uint16_t getAbsXCoordinate(uint8_t fingerNum);
        uint16_t getAbsYCoordinate(uint8_t fingerNum);
        void updateGestures(bool stopOrRestart);
        void setStreamMode(bool stopOrRestart);
        void setEventMode(bool stopOrRestart);
        void IQS7211A::enableGestureEvent(bool stopOrRestart);
        void IQS7211A::disableGestureEvent(bool stopOrRestart);
        void IQS7211A::enableTPEvent(bool stopOrRestart);
        void IQS7211A::disableTPEvent(bool stopOrRestart);
        void IQS7211A::disableCommsReqEn(bool stopOrRestart);
        void IQS7211A::enableCommsReqEn(bool stopOrRestart);
        void IQS7211A::forceComms(void);
        void updateInfoFlags(bool stopOrRestart);
        bool checkReset(void);

private:
        // Private Variables
        uint8_t _deviceAddress;
        uint8_t _readyPin;

        // Private Methods
        void readRandomBytes(uint8_t memoryAddress, uint8_t numBytes, uint8_t bytesArray[], bool stopOrRestart);
        void writeRandomBytes(uint8_t memoryAddress, uint8_t numBytes, uint8_t bytesArray[], bool stopOrRestart);
};
#endif // IQS7211A_h  
