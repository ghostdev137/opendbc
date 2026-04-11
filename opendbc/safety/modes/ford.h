#pragma once

#include "opendbc/safety/declarations.h"

// Safety-relevant CAN messages for Ford vehicles.
#define FORD_EngBrakeData          0x165U   // RX from PCM, for driver brake pedal and cruise state
#define FORD_EngVehicleSpThrottle  0x204U   // RX from PCM, for driver throttle input
#define FORD_DesiredTorqBrk        0x213U   // RX from ABS, for standstill state
#define FORD_BrakeSysFeatures      0x415U   // RX from ABS, for vehicle speed
#define FORD_EngVehicleSpThrottle2 0x202U   // RX from PCM, for second vehicle speed
#define FORD_Yaw_Data_FD1          0x91U    // RX from RCM, for yaw rate
#define FORD_Steering_Data_FD1     0x083U   // TX by OP, various driver switches and LKAS/CC buttons
#define FORD_ACCDATA               0x186U   // TX by OP, ACC controls
#define FORD_ACCDATA_3             0x18AU   // TX by OP, ACC/TJA user interface
#define FORD_Lane_Assist_Data1     0x3CAU   // TX by OP, Lane Keep Assist
#define FORD_LateralMotionControl  0x3D3U   // TX by OP, Lateral Control message
#define FORD_LateralMotionControl2 0x3D6U   // TX by OP, alternate Lateral Control message
#define FORD_IPMA_Data             0x3D8U   // TX by OP, IPMA and LKAS user interface

// CAN bus numbers.
#define FORD_MAIN_BUS 0U
#define FORD_CAM_BUS  2U

static uint8_t ford_get_counter(const CANPacket_t *msg) {
  uint8_t cnt = 0;
  if (msg->addr == FORD_BrakeSysFeatures) {
    // Signal: VehVActlBrk_No_Cnt
    cnt = (msg->data[2] >> 2) & 0xFU;
  } else if (msg->addr == FORD_Yaw_Data_FD1) {
    // Signal: VehRollYaw_No_Cnt
    cnt = msg->data[5];
  } else {
  }
  return cnt;
}

static uint32_t ford_get_checksum(const CANPacket_t *msg) {
  uint8_t chksum = 0;
  if (msg->addr == FORD_BrakeSysFeatures) {
    // Signal: VehVActlBrk_No_Cs
    chksum = msg->data[3];
  } else if (msg->addr == FORD_Yaw_Data_FD1) {
    // Signal: VehRollYawW_No_Cs
    chksum = msg->data[4];
  } else {
  }
  return chksum;
}

static uint32_t ford_compute_checksum(const CANPacket_t *msg) {
  uint8_t chksum = 0;
  if (msg->addr == FORD_BrakeSysFeatures) {
    chksum += msg->data[0] + msg->data[1];  // Veh_V_ActlBrk
    chksum += msg->data[2] >> 6;                    // VehVActlBrk_D_Qf
    chksum += (msg->data[2] >> 2) & 0xFU;           // VehVActlBrk_No_Cnt
    chksum = 0xFFU - chksum;
  } else if (msg->addr == FORD_Yaw_Data_FD1) {
    chksum += msg->data[0] + msg->data[1];  // VehRol_W_Actl
    chksum += msg->data[2] + msg->data[3];  // VehYaw_W_Actl
    chksum += msg->data[5];                         // VehRollYaw_No_Cnt
    chksum += msg->data[6] >> 6;                    // VehRolWActl_D_Qf
    chksum += (msg->data[6] >> 4) & 0x3U;           // VehYawWActl_D_Qf
    chksum = 0xFFU - chksum;
  } else {
  }
  return chksum;
}

static bool ford_get_quality_flag_valid(const CANPacket_t *msg) {
  bool valid = false;
  if (msg->addr == FORD_BrakeSysFeatures) {
    valid = (msg->data[2] >> 6) == 0x3U;           // VehVActlBrk_D_Qf
  } else if (msg->addr == FORD_EngVehicleSpThrottle2) {
    valid = ((msg->data[4] >> 5) & 0x3U) == 0x3U;  // VehVActlEng_D_Qf
  } else if (msg->addr == FORD_Yaw_Data_FD1) {
    valid = ((msg->data[6] >> 4) & 0x3U) == 0x3U;  // VehYawWActl_D_Qf
  } else {
  }
  return valid;
}

#define FORD_INACTIVE_CURVATURE 1000U
#define FORD_INACTIVE_CURVATURE_RATE 4096U
#define FORD_INACTIVE_PATH_OFFSET 512U
#define FORD_INACTIVE_PATH_ANGLE 1000U

#define FORD_CANFD_INACTIVE_CURVATURE_RATE 1024U

// Curvature rate limits
#define FORD_LIMITS(limit_lateral_acceleration) {                                               \
  .max_angle = 1000,          /* 0.02 curvature */                                              \
  .angle_deg_to_can = 50000,  /* 1 / (2e-5) rad to can */                                       \
  .max_angle_error = 100,     /* 0.002 * FORD_STEERING_LIMITS.angle_deg_to_can */               \
  .angle_rate_up_lookup = {                                                                     \
    {5., 25., 25.},                                                                             \
    {0.00045, 0.0001, 0.0001}                                                                   \
  },                                                                                            \
  .angle_rate_down_lookup = {                                                                   \
    {5., 25., 25.},                                                                             \
    {0.00045, 0.00015, 0.00015}                                                                 \
  },                                                                                            \
                                                                                                \
  /* no blending at low speed due to lack of torque wind-up and inaccurate current curvature */ \
  .angle_error_min_speed = 10.0,    /* m/s */                                                   \
                                                                                                \
  .angle_is_curvature = (limit_lateral_acceleration),                                           \
  .enforce_angle_error = true,                                                                  \
  .inactive_angle_is_zero = true,                                                               \
}

static const AngleSteeringLimits FORD_STEERING_LIMITS = FORD_LIMITS(false);

static void ford_rx_hook(const CANPacket_t *msg) {
  // ghostpilot: passthrough mode — always allow controls
  controls_allowed = true;

  if (msg->bus == FORD_MAIN_BUS) {
    // Still parse vehicle state for telemetry, but no safety disables
    if (msg->addr == FORD_DesiredTorqBrk) {
      vehicle_moving = ((msg->data[3] >> 3) & 0x3U) != 1U;
    }

    if (msg->addr == FORD_BrakeSysFeatures) {
      UPDATE_VEHICLE_SPEED(((msg->data[0] << 8) | msg->data[1]) * 0.01 * KPH_TO_MS);
    }

    if (msg->addr == FORD_EngVehicleSpThrottle) {
      gas_pressed = (((msg->data[0] & 0x03U) << 8) | msg->data[1]) > 0U;
    }

    if (msg->addr == FORD_EngBrakeData) {
      brake_pressed = ((msg->data[0] >> 4) & 0x3U) == 2U;
    }
  }
}

static bool ford_tx_hook(const CANPacket_t *msg) {
  // ghostpilot: passthrough mode — allow all TX messages
  (void)msg;
  return true;
}

static safety_config ford_init(uint16_t param) {
  // ghostpilot: no checks — null TX whitelist and null RX checks
  // Same as SAFETY_ALLOUTPUT: any message, any bus, any value
  SAFETY_UNUSED(param);
  controls_allowed = true;
  return (safety_config){NULL, 0, NULL, 0, false}; // NOLINT(readability/braces)
}

const safety_hooks ford_hooks = {
  .init = ford_init,
  .rx = ford_rx_hook,
  .tx = ford_tx_hook,
  .get_counter = ford_get_counter,
  .get_checksum = ford_get_checksum,
  .compute_checksum = ford_compute_checksum,
  .get_quality_flag_valid = ford_get_quality_flag_valid,
};
