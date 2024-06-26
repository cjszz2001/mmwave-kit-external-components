#pragma once
#include "esphome/core/defines.h"
#include "esphome/core/component.h"
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_NUMBER
#include "esphome/components/number/number.h"
#endif
#ifdef USE_SWITCH
#include "esphome/components/switch/switch.h"
#endif
#ifdef USE_BUTTON
#include "esphome/components/button/button.h"
#endif
#ifdef USE_SELECT
#include "esphome/components/select/select.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif
#include "esphome/components/uart/uart.h"
#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"

#include "mmwave_def.h"
#include "trackerproc.h"

#include <map>

namespace esphome {
namespace ti6432 {

static const uint8_t FRAME_BUF_MAX_SIZE = 128;
static const uint8_t PRODUCT_BUF_MAX_SIZE = 32;

static const uint8_t FRAME_IDLE                      = 0;
static const uint8_t FRAME_MAGIC_WORD_LEN            = 8;
static const uint8_t FRAME_HEADER_MAGIC_WORD_VALUE[] = {0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07};
static const uint8_t FRAME_HEADER_LEN                = sizeof(MmwDemo_output_message_header);
static const uint8_t FRAME_MAX_TOTAL_TLV_NUMBER      = 16;  // *** temporary setup
static const uint32_t TLV_MAX_SIZE                   = 1024; // max size for a TL + V
static const uint32_t MESSAGE_MAX_V_SIZE             = TLV_MAX_SIZE - sizeof(MmwDemo_output_message_tl);
static const uint32_t CLASSIFICATION_MAX_FRAMES      = 3; //use 5 frames data to decide human/non-human
static const uint32_t MAX_TARGET_NUMBER              = 5; //track 5 targets at the same time.
static const uint32_t MAX_ZONE_NUMBER                = 5; //
static const uint32_t UNKNOWN_TARGET                 = 0xFFFFFFFF; // preset value to init buffer
static const uint32_t NOT_IN_A_ZONE                  = 0xFFFFFFFF; // 
static const uint32_t TRACKING_TIMEOUT_MS            = 5 * 1000; //
static const uint32_t INVALID_TIMER_ID               = 0xFFFFFFFF;
static const uint32_t MMWDEMO_OUTPUT_MSG_MAX         = 1051; // max value of TLV type
static const float    SENSOR_POS_Z                   = 1.9; 
static const float    MAX_POS_X                      = 4.0; 
static const float    MIN_POS_X                      = -3.0; 
static const float    MAX_POS_Y                      = 7.0; 
static const float    MIN_POS_Y                      = 0.0; 
static const float    MAX_POS_Z                      = 2.0; 
static const float    MIN_POS_Z                      = -2.0; 

enum {
  FRAME_IN_IDLE,
  FRAME_KEEP_IN_IDLE,
  FRAME_IN_WAIT4HEADER,
  FRAME_IN_HEADER,
  FRAME_IN_TL,
  FRAME_IN_WAIT4V,
  FRAME_IN_V,
  FRAME_IN_WAIT4HANDLE,
  FRAME_IN_HANDLE,
  FRAME_TO_RESET, // error happened, reset to idle
};

enum {
  STANDARD_FUNCTION_QUERY_PRODUCT_MODE = 0,
  STANDARD_FUNCTION_QUERY_PRODUCT_ID,
  STANDARD_FUNCTION_QUERY_FIRMWARE_VERSION,
  STANDARD_FUNCTION_QUERY_HARDWARE_MODE,  // Above is the equipment information
  STANDARD_FUNCTION_QUERY_SCENE_MODE,
  STANDARD_FUNCTION_QUERY_SENSITIVITY,
  STANDARD_FUNCTION_QUERY_UNMANNED_TIME,
  STANDARD_FUNCTION_QUERY_HUMAN_STATUS,
  STANDARD_FUNCTION_QUERY_HUMAN_MOTION_INF,
  STANDARD_FUNCTION_QUERY_BODY_MOVE_PARAMETER,
  STANDARD_FUNCTION_QUERY_KEEPAWAY_STATUS,
  STANDARD_QUERY_CUSTOM_MODE,
  STANDARD_FUNCTION_QUERY_HEARTBEAT_STATE,  // Above is the basic function

  CUSTOM_FUNCTION_QUERY_EXISTENCE_BOUNDARY,
  CUSTOM_FUNCTION_QUERY_MOTION_BOUNDARY,
  CUSTOM_FUNCTION_QUERY_EXISTENCE_THRESHOLD,
  CUSTOM_FUNCTION_QUERY_MOTION_THRESHOLD,
  CUSTOM_FUNCTION_QUERY_MOTION_TRIGGER_TIME,
  CUSTOM_FUNCTION_QUERY_MOTION_TO_REST_TIME,
  CUSTOM_FUNCTION_QUERY_TIME_OF_ENTER_UNMANNED,

  UNDERLY_FUNCTION_QUERY_HUMAN_STATUS,
  UNDERLY_FUNCTION_QUERY_SPATIAL_STATIC_VALUE,
  UNDERLY_FUNCTION_QUERY_SPATIAL_MOTION_VALUE,
  UNDERLY_FUNCTION_QUERY_DISTANCE_OF_STATIC_OBJECT,
  UNDERLY_FUNCTION_QUERY_DISTANCE_OF_MOVING_OBJECT,
  UNDERLY_FUNCTION_QUERY_TARGET_MOVEMENT_SPEED,
};

enum {
  OUTPUT_SWITCH_INIT,
  OUTPUT_SWTICH_ON,
  OUTPUT_SWTICH_OFF,
};

static const char *const S_SCENE_STR[5] = {"None", "Living Room", "Bedroom", "Washroom", "Area Detection"};
static const bool S_SOMEONE_EXISTS_STR[2] = {false, true};
static const char *const S_MOTION_STATUS_STR[] = {"None", "Minor", "Major", "Major and Minor"};
static const char *const S_KEEP_AWAY_STR[3] = {"None", "Close", "Away"};
static const char *const S_UNMANNED_TIME_STR[9] = {"None", "10s",   "30s",   "1min", "2min",
                                                   "5min", "10min", "30min", "60min"};
static const char *const S_BOUNDARY_STR[10] = {"0.5m", "1.0m", "1.5m", "2.0m", "2.5m",
                                               "3.0m", "3.5m", "4.0m", "4.5m", "5.0m"};       // uint: m
static const float S_PRESENCE_OF_DETECTION_RANGE_STR[7] = {0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0};  // uint: m

class TI6432Component : public Component,
                          public uart::UARTDevice {  // The class name must be the name defined by text_sensor.py
#ifdef USE_TEXT_SENSOR
//   SUB_TEXT_SENSOR(heartbeat_state)
//   SUB_TEXT_SENSOR(product_model)
//   SUB_TEXT_SENSOR(product_id)
//   SUB_TEXT_SENSOR(hardware_model)
//   SUB_TEXT_SENSOR(firware_version)
//   SUB_TEXT_SENSOR(keep_away)
  SUB_TEXT_SENSOR(motion_status)
//   SUB_TEXT_SENSOR(custom_mode_end)
#endif
#ifdef USE_BINARY_SENSOR
  SUB_BINARY_SENSOR(has_target)
#endif
#ifdef USE_SENSOR
  SUB_SENSOR(custom_presence_of_detection)
  SUB_SENSOR(movement_signs)
  SUB_SENSOR(custom_motion_distance)
  SUB_SENSOR(custom_spatial_static_value)
  SUB_SENSOR(custom_spatial_motion_value)
  SUB_SENSOR(custom_motion_speed)
  SUB_SENSOR(custom_mode_num)
  SUB_SENSOR(human_entered_in_room)
#endif
#ifdef USE_SWITCH
  SUB_SWITCH(underly_open_function)
#endif
#ifdef USE_BUTTON
  SUB_BUTTON(restart)
  SUB_BUTTON(custom_set_end)
#endif
#ifdef USE_SELECT
  SUB_SELECT(scene_mode)
  SUB_SELECT(unman_time)
  SUB_SELECT(existence_boundary)
  SUB_SELECT(motion_boundary)
#endif
#ifdef USE_NUMBER
  SUB_NUMBER(sensitivity)
  SUB_NUMBER(custom_mode)
  SUB_NUMBER(existence_threshold)
  SUB_NUMBER(motion_threshold)
  SUB_NUMBER(motion_trigger)
  SUB_NUMBER(motion_to_rest)
  SUB_NUMBER(custom_unman_time)
#endif

 protected:
  typedef struct 
  {
     MmwDemo_output_message_tl       tl;
     uint8_t                         v[MESSAGE_MAX_V_SIZE];
  } MESSAGE_TLV;
  typedef struct 
  {
     float startX;
     float endX;
     float startY;
     float endY;
     float startZ;
     float endZ;
  } ZONE_BOUNDARY;
  typedef struct 
  {
     // set to 2 float value because NUM_CLASSES_IN_CLASSIFIER defined as 2
     float    humanProb;
     float    nonHumanProb;
  } CLASS_PROBABILITY;

  typedef struct 
  {
     uint32_t           targetId;     // UNKNOWN_TARGET init value
     trackerProc_Target targetTracker;
     uint32_t           targetInZone;  // zone number if this target is in a zone
     uint8_t            validFrameNum; // how many frames are valid in isHuman array. range 0 - CLASSIFICATION_MAX_FRAMES
     bool               reported;      // this target is reported as HUMAN. (non-human will not affect this flag)
     uint8_t            timerIndex;    // index in tracking_timer array
     int8_t             sum;           // sum of isHuman when it's full.
     int8_t             isHuman[CLASSIFICATION_MAX_FRAMES]; // -1 not human, 1 human, 0 init value
  } CLASSIFICATION_DATA;

  typedef struct 
  {
     float     entryY;
  } ENTRY_COUNT_DATA;

  static constexpr ZONE_BOUNDARY zoneBoundary[] = {
     {-4, 4, 0.0, 7.0, 0.0, 3.0}
//     {-1.5, 1.5, 1.0, 3.0, 0.0, 3.0}
//    ,{-1.5, 1.5, 3.0, 5, 0, 3}
  };

  // entry zone used for counting people entering the room
  // z coordinates don't matter
  static constexpr ZONE_BOUNDARY entryZone = {-3, 3, 2.0, 3.5, 0.0, 3.0};

  char c_product_mode_[PRODUCT_BUF_MAX_SIZE + 1];
  char c_product_id_[PRODUCT_BUF_MAX_SIZE + 1];
  char c_hardware_model_[PRODUCT_BUF_MAX_SIZE + 1];
  char c_firmware_version_[PRODUCT_BUF_MAX_SIZE + 1];
  uint8_t s_output_info_switch_flag_;
  uint8_t sg_recv_data_state_;
  uint8_t sg_frame_len_;
  uint8_t sg_data_len_;
  
  uint32_t                        pos_in_frame;
  MmwDemo_output_message_header   frame_header;
  // uint8_t                         sg_frame_header_buf_[FRAME_HEADER_LEN];
  MESSAGE_TLV                     current_message;
  std::vector<MESSAGE_TLV>        message_tlv;

  std::vector<uint8_t>            zone_presence;
  //std::vector<uint8_t>            indexes;
  //std::vector<CLASS_OUTCOME>      class_outcome;
  std::vector<CLASSIFICATION_DATA>  class_outcome;
  int8_t                            reported_human_number[MAX_ZONE_NUMBER];
  int8_t                            human_count_in_room;
  std::map<uint32_t, ENTRY_COUNT_DATA>     entry_targets;

  // bool poll_time_base_func_check_;

  void update_();
  void send_query_(uint8_t *query, size_t string_length);

  void handle_frame(void);
  void handle_tlv(MESSAGE_TLV &tlv);
  void handle_ext_msg_enhanced_presence_indication(uint8_t *data, uint32_t length);
  void handle_ext_msg_target_list(uint8_t *data, uint32_t length);
  void handle_ext_msg_target_index(uint8_t *data, uint32_t length);
  void handle_ext_msg_classifier_info(uint8_t *data, uint32_t length);
  void handle_ext_msg_detected_points(uint8_t *data, uint32_t length);
  void handle_ext_msg_range_profile_major(uint8_t *data, uint32_t length);
  void handle_ext_msg_range_profile_minor(uint8_t *data, uint32_t length);
  void handle_msg_ext_stats(uint8_t *data, uint32_t length);

  void read_big_data_from_uart(uint8_t *data, uint32_t length);
  void read_array_with_delay(uint8_t *data, uint32_t length);
  bool isTargetInZone(trackerProc_Target &tracker, uint32_t *zoneNum);
  void report_human_in_zone(int32_t humanNum, uint32_t zoneNum);
  bool isTargetInEntryZone(trackerProc_Target &tracker);
  bool isTargetEntering(float y);
  bool isTargetReportedAsHuman(uint32_t targetId);

 public:
  float get_setup_priority() const override { return esphome::setup_priority::LATE; }
  void setup() override;
  void dump_config() override;
  void loop() override;

  void get_heartbeat_packet();
  void get_radar_output_information_switch();
  void get_product_mode();
  void get_product_id();
  void get_hardware_model();
  void get_firmware_version();
  void get_human_status();
  void get_human_motion_info();
  void get_body_motion_params();
  void get_keep_away();
  void get_scene_mode();
  void get_sensitivity();
  void get_unmanned_time();
  void get_custom_mode();
  void get_existence_boundary();
  void get_motion_boundary();
  void get_spatial_static_value();
  void get_spatial_motion_value();
  void get_distance_of_static_object();
  void get_distance_of_moving_object();
  void get_target_movement_speed();
  void get_existence_threshold();
  void get_motion_threshold();
  void get_motion_trigger_time();
  void get_motion_to_rest_time();
  void get_custom_unman_time();

  void set_scene_mode(uint8_t value);
  void set_underlying_open_function(bool enable);
  void set_sensitivity(uint8_t value);
  void set_restart();
  void set_unman_time(uint8_t value);
  void set_custom_mode(uint8_t mode);
  void set_custom_end_mode();
  void set_existence_boundary(uint8_t value);
  void set_motion_boundary(uint8_t value);
  void set_existence_threshold(int value);
  void set_motion_threshold(int value);
  void set_motion_trigger_time(int value);
  void set_motion_to_rest_time(int value);
  void set_custom_unman_time(int value);
};

}  // namespace ti6432
}  // namespace esphome
