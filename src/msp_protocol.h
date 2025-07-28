#pragma once

#include <vector>
#include <cstdint>

// MSP Commands - Updated to match working VOT_C implementation
#define MSP_AUX_FUNCTIONS     227
#define MSP_SET_AUX_FUNCTIONS 228
#define MSP_RC_TUNING         111
#define MSP_SET_RC_TUNING     204
#define MSP_PID_ADVANCED      94
#define MSP_SET_PID_ADVANCED  95
#define MSP_EEPROM_WRITE      250

// Additional MSP commands that might be needed
#define MSP_IDENT             100
#define MSP_STATUS            101
#define MSP_RAW_IMU           102
#define MSP_SERVO             103
#define MSP_MOTOR             104
#define MSP_RC                105
#define MSP_RAW_GPS           106
#define MSP_COMP_GPS          107
#define MSP_ATTITUDE          108
#define MSP_ALTITUDE          109
#define MSP_ANALOG            110
#define MSP_RC_TUNING         111
#define MSP_PID               112
#define MSP_BOX               113
#define MSP_MISC              114
#define MSP_MOTOR_PINS        115
#define MSP_BOXNAMES          116
#define MSP_PIDNAMES          117
#define MSP_BOXIDS            119
#define MSP_SERVO_CONF        120
#define MSP_SET_RAW_RC        200
#define MSP_SET_RAW_GPS       201
#define MSP_SET_PID           202
#define MSP_SET_BOX           203
#define MSP_SET_RC_TUNING     204
#define MSP_ACC_CALIBRATION   205
#define MSP_MAG_CALIBRATION   206
#define MSP_SET_MISC          207
#define MSP_RESET_CONF        208
#define MSP_SET_WP            209
#define MSP_SELECT_SETTING    210
#define MSP_SET_HEAD           211
#define MSP_SET_SERVO_CONF    212
#define MSP_SET_MOTOR         214
#define MSP_SET_NAV_CONFIG    215
#define MSP_SET_3D            217
#define MSP_SET_RC_EXPO       218
#define MSP_SET_ACC_TRIM      220
#define MSP_ACC_TRIM          221
#define MSP_SERVO_MIX_RULES   222
#define MSP_SET_SERVO_MIX_RULE 223
#define MSP_SET_MOTOR_3D_CONFIG 225
#define MSP_SET_MOTOR_CONFIG  226
#define MSP_SET_GPS_CONFIG    227
#define MSP_SET_COMPASS_CONFIG 228
#define MSP_SET_PID_ADVANCED  229
#define MSP_SET_SENSOR_ALIGNMENT 230
#define MSP_SET_LOOP_TIME     231
#define MSP_SET_SPECIAL_PARAMETERS 232
#define MSP_SET_ARMING_CONFIG 233
#define MSP_SET_RESET_SWITCH_CHANNEL 234
#define MSP_SET_3D_DEADBAND   235
#define MSP_SET_RC_DEADBAND   236
#define MSP_SET_RESET_SWITCH_CHANNEL 237
#define MSP_SET_ESC_SENSOR    238
#define MSP_SET_DEBUG_MODE    239
#define MSP_SET_THROTTLE_SCALE 240
#define MSP_SET_3D_SWITCH_CHANNEL 241
#define MSP_SET_SERVO_CONFIGURATION 242
#define MSP_SET_GPS_NAV_P     243
#define MSP_SET_GPS_NAV_I     244
#define MSP_SET_GPS_NAV_D     245
#define MSP_SET_GPS_WP_APROACH 246
#define MSP_SET_GPS_LOW_VELOCITY_DAMPING 247
#define MSP_SET_GPS_HOLD_P    248
#define MSP_SET_GPS_HOLD_I    249
#define MSP_SET_GPS_HOLD_D    250
#define MSP_SET_GPS_HOLD_RLL_P 251
#define MSP_SET_GPS_HOLD_RLL_I 252
#define MSP_SET_GPS_HOLD_RLL_D 253
#define MSP_SET_GPS_HOLD_YAW_P 254
#define MSP_SET_GPS_HOLD_YAW_I 255
#define MSP_SET_GPS_HOLD_YAW_D 256
#define MSP_SET_GPS_HOLD_VELOCITY_P 257
#define MSP_SET_GPS_HOLD_VELOCITY_I 258
#define MSP_SET_GPS_HOLD_VELOCITY_D 259
#define MSP_SET_GPS_HOLD_VELOCITY_MAX 260
#define MSP_SET_GPS_HOLD_VELOCITY_MIN 261
#define MSP_SET_GPS_HOLD_VELOCITY_ACCEL 262
#define MSP_SET_GPS_HOLD_VELOCITY_DECEL 263
#define MSP_SET_GPS_HOLD_VELOCITY_ACCEL_DECEL 264
#define MSP_SET_GPS_HOLD_VELOCITY_ACCEL_DECEL_ALT 265
#define MSP_SET_GPS_HOLD_VELOCITY_ACCEL_DECEL_ALT_VEL 266
#define MSP_SET_GPS_HOLD_VELOCITY_ACCEL_DECEL_ALT_VEL_ACC 267
#define MSP_SET_GPS_HOLD_VELOCITY_ACCEL_DECEL_ALT_VEL_ACC_DEC 268
#define MSP_SET_GPS_HOLD_VELOCITY_ACCEL_DECEL_ALT_VEL_ACC_DEC_ALT 269
#define MSP_SET_GPS_HOLD_VELOCITY_ACCEL_DECEL_ALT_VEL_ACC_DEC_ALT_VEL 270
#define MSP_SET_GPS_HOLD_VELOCITY_ACCEL_DECEL_ALT_VEL_ACC_DEC_ALT_VEL_ACC 271
#define MSP_SET_GPS_HOLD_VELOCITY_ACCEL_DECEL_ALT_VEL_ACC_DEC_ALT_VEL_ACC_DEC 272
#define MSP_SET_GPS_HOLD_VELOCITY_ACCEL_DECEL_ALT_VEL_ACC_DEC_ALT_VEL_ACC_DEC_ALT 273
#define MSP_SET_GPS_HOLD_VELOCITY_ACCEL_DECEL_ALT_VEL_ACC_DEC_ALT_VEL_ACC_DEC_ALT_VEL 274
#define MSP_SET_GPS_HOLD_VELOCITY_ACCEL_DECEL_ALT_VEL_ACC_DEC_ALT_VEL_ACC_DEC_ALT_VEL_ACC 275
#define MSP_SET_GPS_HOLD_VELOCITY_ACCEL_DECEL_ALT_VEL_ACC_DEC_ALT_VEL_ACC_DEC_ALT_VEL_ACC_DEC 276
#define MSP_SET_GPS_HOLD_VELOCITY_ACCEL_DECEL_ALT_VEL_ACC_DEC_ALT_VEL_ACC_DEC_ALT_VEL_ACC_DEC_ALT 277
#define MSP_SET_GPS_HOLD_VELOCITY_ACCEL_DECEL_ALT_VEL_ACC_DEC_ALT_VEL_ACC_DEC_ALT_VEL_ACC_DEC_ALT_VEL 278
#define MSP_SET_GPS_HOLD_VELOCITY_ACCEL_DECEL_ALT_VEL_ACC_DEC_ALT_VEL_ACC_DEC_ALT_VEL_ACC_DEC_ALT_VEL_ACC 279
#define MSP_SET_GPS_HOLD_VELOCITY_ACCEL_DECEL_ALT_VEL_ACC_DEC_ALT_VEL_ACC_DEC_ALT_VEL_ACC_DEC_ALT_VEL_ACC_DEC 280

// MSP Override mode
#define MSP_OVERRIDE_MODE     50

// Firmware variants
enum FirmwareVariant {
    FIRMWARE_VARIANT_INAV = 0,
    FIRMWARE_VARIANT_BAFL = 1,
    FIRMWARE_VARIANT_BETAFLIGHT = 2
};

struct MSPMessage {
    uint8_t header[3] = {'$', 'M', '<'};  // For INAV, use '<' for requests
    uint8_t size;
    uint8_t cmd;
    std::vector<uint8_t> data;
    uint8_t checksum;
    
    MSPMessage() : size(0), cmd(0), checksum(0) {}
    MSPMessage(uint8_t command, const std::vector<uint8_t>& payload = {});
};

struct AuxFunction {
    uint8_t aux_id;
    uint8_t mode;
    uint8_t aux_channel;
    uint16_t range_start;
    uint16_t range_end;
    uint8_t extra1;
    uint8_t extra2;
};

struct RCTuning {
    uint8_t roll_rc_rate;
    uint8_t pitch_rc_rate;
    uint8_t yaw_rc_rate;
    uint8_t roll_srate;
    uint8_t pitch_srate;
    uint8_t yaw_srate;
    uint8_t thr_mid;
    uint8_t thr_expo;
    uint16_t tpa_rate;
    uint16_t tpa_breakpoint;
    uint8_t expo_8;
    uint8_t roll_rate_limit;
    uint8_t pitch_rate_limit;
    uint8_t yaw_rate_limit;
    uint8_t thrust_linear;
    uint8_t rates_type;
    uint8_t msp_override_channels_mask;
    uint8_t msp_override_failsafe;
};

class MSPProtocol {
public:
    static std::vector<uint8_t> encodeMessage(const MSPMessage& msg);
    static bool decodeMessage(const std::vector<uint8_t>& buffer, MSPMessage& msg);
    static uint8_t calculateChecksum(const MSPMessage& msg);
    static std::vector<AuxFunction> parseAuxFunctions(const std::vector<uint8_t>& data);
    static RCTuning parseRCTuning(const std::vector<uint8_t>& data);
    static std::vector<uint8_t> encodeRCTuning(const RCTuning& tuning);
    static std::vector<uint8_t> encodeAuxFunctions(const std::vector<AuxFunction>& functions);
    
    // New methods for better compatibility
    static MSPMessage createIdentRequest();
    static MSPMessage createStatusRequest();
    static MSPMessage createRcRequest();
    static MSPMessage createAttitudeRequest();
    static MSPMessage createAltitudeRequest();
    static MSPMessage createAnalogRequest();
    static MSPMessage createSetRCTuningRequest(const RCTuning& tuning);
};