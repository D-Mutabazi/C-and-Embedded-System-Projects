/*
* This file contains all the necessary settings for the IQS7211A and this file can
* be changed from the GUI or edited here
* File:   IQS7211A_init.h
* Author: Azoteq
*/ 

#ifndef IQS7211A_INIT_H
#define IQS7211A_INIT_H

/* Change the ATI Settings */
/* Memory Map Position 0x30 - 0x39 */
#define TP_ATI_MULTIPLIERS_DIVIDERS_0            0xE1
#define TP_ATI_MULTIPLIERS_DIVIDERS_1            0x33
#define TP_COMPENSATION_DIV_0                    0x0C
#define TP_COMPENSATION_DIV_1                    0x00
#define TP_ATI_TARGET_0                          0x2C
#define TP_ATI_TARGET_1                          0x01
#define TP_REF_DRIFT_LIMIT_0                     0x32
#define TP_REF_DRIFT_LIMIT_1                     0x00
#define TP_MIN_COUNT_REATI_0                     0x32
#define TP_MIN_COUNT_REATI_1                     0x00
#define REATI_RETRY_TIME_0                       0x05
#define REATI_RETRY_TIME_1                       0x00
#define ALP_ATI_MULTIPLIERS_DIVIDERS_0           0x23
#define ALP_ATI_MULTIPLIERS_DIVIDERS_1           0x02
#define ALP_COMPENSATION_DIV_0                   0x02
#define ALP_COMPENSATION_DIV_1                   0x00
#define ALP_ATI_TARGET_0                         0xC8
#define ALP_ATI_TARGET_1                         0x00
#define ALP_LTA_DRIFT_LIMIT_0                    0x14
#define ALP_LTA_DRIFT_LIMIT_1                    0x00

/* Change the ALP ATI Compensation */
/* Memory Map Position 0x3A - 0x3B */
#define ALP_COMPENSATION_A_0                     0xE8
#define ALP_COMPENSATION_A_1                     0x01
#define ALP_COMPENSATION_B_0                     0xD3
#define ALP_COMPENSATION_B_1                     0x01

/* Change the Report Rates and Timing */
/* Memory Map Position 0x40 - 0x4A */
#define ACTIVE_MODE_REPORT_RATE_0                0x0A
#define ACTIVE_MODE_REPORT_RATE_1                0x00
#define IDLE_TOUCH_MODE_REPORT_RATE_0            0x32
#define IDLE_TOUCH_MODE_REPORT_RATE_1            0x00
#define IDLE_MODE_REPORT_RATE_0                  0x32
#define IDLE_MODE_REPORT_RATE_1                  0x00
#define LP1_MODE_REPORT_RATE_0                   0x50
#define LP1_MODE_REPORT_RATE_1                   0x00
#define LP2_MODE_REPORT_RATE_0                   0xA0
#define LP2_MODE_REPORT_RATE_1                   0x00
#define ACTIVE_MODE_TIMEOUT_0                    0x0A
#define ACTIVE_MODE_TIMEOUT_1                    0x00
#define IDLE_TOUCH_MODE_TIMEOUT_0                0x3C
#define IDLE_TOUCH_MODE_TIMEOUT_1                0x00
#define IDLE_MODE_TIMEOUT_0                      0x14
#define IDLE_MODE_TIMEOUT_1                      0x00
#define LP1_MODE_TIMEOUT_0                       0x0A
#define LP1_MODE_TIMEOUT_1                       0x00
#define REF_UPDATE_TIME_0                        0x08
#define REF_UPDATE_TIME_1                        0x00
#define I2C_TIMEOUT_0                            0x64
#define I2C_TIMEOUT_1                            0x00

/* Change the System Settings */
/* Memory Map Position 0x50 - 0x5B */
#define SYSTEM_CONTROL_0                         0x00
#define SYSTEM_CONTROL_1                         0x00
#define CONFIG_SETTINGS0                         0x3C
#define CONFIG_SETTINGS1                         0x06
#define OTHER_SETTINGS_0                         0x20
#define OTHER_SETTINGS_1                         0xFF
#define TRACKPAD_TOUCH_SET_THRESHOLD             0x32
#define TRACKPAD_TOUCH_CLEAR_THRESHOLD           0x14
#define ALP_THRESHOLD_0                          0x08
#define ALP_THRESHOLD_1                          0x00
#define OPEN_0_0                                 0xFF
#define OPEN_0_1                                 0xFF
#define ALP_SET_DEBOUNCE                         0x04
#define ALP_CLEAR_DEBOUNCE                       0x04
#define OPEN_1_0                                 0xFF
#define OPEN_1_1                                 0xFF
#define TP_CONVERSION_FREQUENCY_UP_PASS_LENGTH   0x02
#define TP_CONVERSION_FREQUENCY_FRACTION_VALUE   0x1A
#define ALP_CONVERSION_FREQUENCY_UP_PASS_LENGTH  0x02
#define ALP_CONVERSION_FREQUENCY_FRACTION_VALUE  0x1A
#define TRACKPAD_HARDWARE_SETTINGS_0             0x01
#define TRACKPAD_HARDWARE_SETTINGS_1             0x8D
#define ALP_HARDWARE_SETTINGS_0                  0x65
#define ALP_HARDWARE_SETTINGS_1                  0x9D

/* Change the Trackpad Settings */
/* Memory Map Position 0x60 - 0x69 */
#define TRACKPAD_SETTINGS_0_0                    0x28
#define TRACKPAD_SETTINGS_0_1                    0x08
#define TRACKPAD_SETTINGS_1_0                    0x04
#define TRACKPAD_SETTINGS_1_1                    0x02
#define X_RESOLUTION_0                           0x00
#define X_RESOLUTION_1                           0x07
#define Y_RESOLUTION_0                           0x00
#define Y_RESOLUTION_1                           0x03
#define XY_DYNAMIC_FILTER_BOTTOM_SPEED_0         0x06
#define XY_DYNAMIC_FILTER_BOTTOM_SPEED_1         0x00
#define XY_DYNAMIC_FILTER_TOP_SPEED_0            0x7C
#define XY_DYNAMIC_FILTER_TOP_SPEED_1            0x00
#define XY_DYNAMIC_FILTER_BOTTOM_BETA            0x07
#define XY_DYNAMIC_FILTER_STATIC_FILTER_BETA     0x80
#define STATIONARY_TOUCH_MOV_THRESHOLD           0x14
#define FINGER_SPLIT_FACTOR                      0x03
#define X_TRIM_VALUE_0                           0x14
#define X_TRIM_VALUE_1                           0x00
#define Y_TRIM_VALUE_0                           0x14
#define Y_TRIM_VALUE_1                           0x00

/* Change the ALP Settings */
/* Memory Map Position 0x70 - 0x73 */
#define ALP_COUNT_FILTER_BETA_0                  0xB4
#define OPEN_0                                   0x00
#define ALP_LTA_BETA_LP1                         0x06
#define ALP_LTA_BETA_LP2                         0x04
#define ALP_SETUP_0                              0xAA
#define ALP_SETUP_1                              0x03
#define ALP_TX_ENABLE_0                          0x00
#define ALP_TX_ENABLE_1                          0x0F

/* Change the Settings Version Numbers */
/* Memory Map Position 0x74 - 0x74 */
#define MINOR_VERSION                            0x00
#define MAJOR_VERSION                            0x00

/* Change the Gesture Settings */
/* Memory Map Position 0x80 - 0x87 */
#define GESTURE_ENABLE_0                         0x3F
#define GESTURE_ENABLE_1                         0x0F
#define TAP_TIME_0                               0x96
#define TAP_TIME_1                               0x00
#define TAP_DISTANCE_0                           0x32
#define TAP_DISTANCE_1                           0x00
#define HOLD_TIME_0                              0x2C
#define HOLD_TIME_1                              0x01
#define SWIPE_TIME_0                             0x96
#define SWIPE_TIME_1                             0x00
#define SWIPE_X_DISTANCE_0                       0xC8
#define SWIPE_X_DISTANCE_1                       0x00
#define SWIPE_Y_DISTANCE_0                       0xC8
#define SWIPE_Y_DISTANCE_1                       0x00
#define SWIPE_ANGLE_0                            0x17
#define GESTURE_OPEN_0                           0x00

/* Change the RxTx Mapping */
/* Memory Map Position 0x90 - 0x96 */
#define RX_TX_MAP_0                              0x07
#define RX_TX_MAP_1                              0x06
#define RX_TX_MAP_2                              0x05
#define RX_TX_MAP_3                              0x04
#define RX_TX_MAP_4                              0x03
#define RX_TX_MAP_5                              0x02
#define RX_TX_MAP_6                              0x01
#define RX_TX_MAP_7                              0x00
#define RX_TX_MAP_8                              0x08
#define RX_TX_MAP_9                              0x09
#define RX_TX_MAP_10                             0x0A
#define RX_TX_MAP_11                             0x0B
#define RX_TX_MAP_12                             0x00

/* Change the Allocation of channels into cycles 0-9 */
/* Memory Map Position 0xA0 - 0xAE */
#define PLACEHOLDER_0                            0x05
#define CH_1_CYCLE_0                             0x04
#define CH_2_CYCLE_0                             0x00
#define PLACEHOLDER_1                            0x05
#define CH_1_CYCLE_1                             0x05
#define CH_2_CYCLE_1                             0x01
#define PLACEHOLDER_2                            0x05
#define CH_1_CYCLE_2                             0x06
#define CH_2_CYCLE_2                             0x02
#define PLACEHOLDER_3                            0x05
#define CH_1_CYCLE_3                             0x07
#define CH_2_CYCLE_3                             0x03
#define PLACEHOLDER_4                            0x05
#define CH_1_CYCLE_4                             0x0C
#define CH_2_CYCLE_4                             0x08
#define PLACEHOLDER_5                            0x05
#define CH_1_CYCLE_5                             0x0D
#define CH_2_CYCLE_5                             0x09
#define PLACEHOLDER_6                            0x05
#define CH_1_CYCLE_6                             0x0E
#define CH_2_CYCLE_6                             0x0A
#define PLACEHOLDER_7                            0x05
#define CH_1_CYCLE_7                             0x0F
#define CH_2_CYCLE_7                             0x0B
#define PLACEHOLDER_8                            0x05
#define CH_1_CYCLE_8                             0x14
#define CH_2_CYCLE_8                             0x10
#define PLACEHOLDER_9                            0x05
#define CH_1_CYCLE_9                             0x15
#define CH_2_CYCLE_9                             0x11

/* Change the Allocation of channels into cycles 10-17 */
/* Memory Map Position 0xB0 - 0xBB */
#define PLACEHOLDER_10                           0x05
#define CH_1_CYCLE_10                            0x16
#define CH_2_CYCLE_10                            0x12
#define PLACEHOLDER_11                           0x05
#define CH_1_CYCLE_11                            0x17
#define CH_2_CYCLE_11                            0x13
#define PLACEHOLDER_12                           0x05
#define CH_1_CYCLE_12                            0x1C
#define CH_2_CYCLE_12                            0x18
#define PLACEHOLDER_13                           0x05
#define CH_1_CYCLE_13                            0x1D
#define CH_2_CYCLE_13                            0x19
#define PLACEHOLDER_14                           0x05
#define CH_1_CYCLE_14                            0x1E
#define CH_2_CYCLE_14                            0x1A
#define PLACEHOLDER_15                           0x05
#define CH_1_CYCLE_15                            0x1F
#define CH_2_CYCLE_15                            0x1B
#define PLACEHOLDER_16                           0x05
#define CH_1_CYCLE_16                            0xFF
#define CH_2_CYCLE_16                            0xFF
#define PLACEHOLDER_17                           0x05
#define CH_1_CYCLE_17                            0xFF
#define CH_2_CYCLE_17                            0xFF

#endif	/* IQS7211A_INIT_H */