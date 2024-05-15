#include "esphome/core/log.h"
#include "ti6432.h"

#include <utility>
#ifdef USE_NUMBER
#include "esphome/components/number/number.h"
#endif
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif

namespace esphome {
namespace ti6432 {

static const char *const TAG = "ti6432";

// Prints the component's configuration data. dump_config() prints all of the component's configuration
// items in an easy-to-read format, including the configuration key-value pairs.
void TI6432Component::dump_config() {
  ESP_LOGCONFIG(TAG, "TI6432:");
#ifdef USE_TEXT_SENSOR
//   LOG_TEXT_SENSOR(" ", "Heartbeat Text Sensor", this->heartbeat_state_text_sensor_);
//   LOG_TEXT_SENSOR(" ", "Product Model Text Sensor", this->product_model_text_sensor_);
//   LOG_TEXT_SENSOR(" ", "Product ID Text Sensor", this->product_id_text_sensor_);
//   LOG_TEXT_SENSOR(" ", "Hardware Model Text Sensor", this->hardware_model_text_sensor_);
//   LOG_TEXT_SENSOR(" ", "Firware Verison Text Sensor", this->firware_version_text_sensor_);
//   LOG_TEXT_SENSOR(" ", "Keep Away Text Sensor", this->keep_away_text_sensor_);
  LOG_TEXT_SENSOR(" ", "Motion Status Text Sensor", this->motion_status_text_sensor_);
//   LOG_TEXT_SENSOR(" ", "Custom Mode End Text Sensor", this->custom_mode_end_text_sensor_);
#endif
#ifdef USE_BINARY_SENSOR
  LOG_BINARY_SENSOR(" ", "Has Target Binary Sensor", this->has_target_binary_sensor_);
#endif
#ifdef USE_SENSOR
//zone 1
  LOG_SENSOR(" ", "Custom Presence Of Detection Sensor", this->custom_presence_of_detection_sensor_);
//zone 2
  LOG_SENSOR(" ", "Movement Signs Sensor", this->movement_signs_sensor_);
//zone 3  
  LOG_SENSOR(" ", "Custom Motion Distance Sensor", this->custom_motion_distance_sensor_);
// target ID
  LOG_SENSOR(" ", "Custom Spatial Static Sensor", this->custom_spatial_static_value_sensor_);  
// human or non-human  
  LOG_SENSOR(" ", "Custom Spatial Motion Sensor", this->custom_spatial_motion_value_sensor_);

  LOG_SENSOR(" ", "Custom Motion Speed Sensor", this->custom_motion_speed_sensor_);
  LOG_SENSOR(" ", "Custom Mode Num Sensor", this->custom_mode_num_sensor_);
#endif
#ifdef USE_SWITCH
  LOG_SWITCH(" ", "Underly Open Function Switch", this->underly_open_function_switch_);
#endif
#ifdef USE_BUTTON
  LOG_BUTTON(" ", "Restart Button", this->restart_button_);
  LOG_BUTTON(" ", "Custom Set End Button", this->custom_set_end_button_);
#endif
#ifdef USE_SELECT
  LOG_SELECT(" ", "Scene Mode Select", this->scene_mode_select_);
  LOG_SELECT(" ", "Unman Time Select", this->unman_time_select_);
  LOG_SELECT(" ", "Existence Boundary Select", this->existence_boundary_select_);
  LOG_SELECT(" ", "Motion Boundary Select", this->motion_boundary_select_);
#endif
#ifdef USE_NUMBER
  LOG_NUMBER(" ", "Sensitivity Number", this->sensitivity_number_);
  LOG_NUMBER(" ", "Custom Mode Number", this->custom_mode_number_);
  LOG_NUMBER(" ", "Existence Threshold Number", this->existence_threshold_number_);
  LOG_NUMBER(" ", "Motion Threshold Number", this->motion_threshold_number_);
  LOG_NUMBER(" ", "Motion Trigger Time Number", this->motion_trigger_number_);
  LOG_NUMBER(" ", "Motion To Rest Time Number", this->motion_to_rest_number_);
  LOG_NUMBER(" ", "Custom Unman Time Number", this->custom_unman_time_number_);
#endif
}

// Initialisation functions
void TI6432Component::setup() {
  ESP_LOGCONFIG(TAG, "uart_settings is 115200");
  this->check_uart_settings(115200);
//   ESP_LOGCONFIG(TAG, "uart_settings is 1250000");
//   this->check_uart_settings(1250000);

  if (this->custom_mode_number_ != nullptr) {
    this->custom_mode_number_->publish_state(0);  // Zero out the custom mode
  }
  if (this->custom_mode_num_sensor_ != nullptr) {
    this->custom_mode_num_sensor_->publish_state(0);
  }
//   if (this->custom_mode_end_text_sensor_ != nullptr) {
//     this->custom_mode_end_text_sensor_->publish_state("Not in custom mode");
//   }
  this->set_custom_end_mode();
  this->poll_time_base_func_check_ = true;
  this->check_dev_inf_sign_ = true;
  this->sg_start_query_data_ = STANDARD_FUNCTION_QUERY_PRODUCT_MODE;
  this->sg_data_len_ = 0;
  this->sg_frame_len_ = 0;
  this->sg_recv_data_state_ = FRAME_IDLE;
  this->s_output_info_switch_flag_ = OUTPUT_SWITCH_INIT;

  memset(this->c_product_mode_, 0, PRODUCT_BUF_MAX_SIZE);
  memset(this->c_product_id_, 0, PRODUCT_BUF_MAX_SIZE);
  memset(this->c_firmware_version_, 0, PRODUCT_BUF_MAX_SIZE);
  memset(this->c_hardware_model_, 0, PRODUCT_BUF_MAX_SIZE);
  memset(this->sg_frame_prase_buf_, 0, FRAME_BUF_MAX_SIZE);
  memset(this->sg_frame_buf_, 0, FRAME_BUF_MAX_SIZE);

  this->pos_in_frame = FRAME_IN_IDLE;

  this->set_interval(8000, [this]() { this->update_(); });


  class_outcome.clear();
//   for (auto class : this->class_outcome)
//   {
//      class.targetId = UNKNOWN_TARGET;
//      memset(class.isHuman, 0, CLASSIFICATION_MAX_FRAMES);
//   }
}

// Timed polling of radar data
void TI6432Component::update_() {
  this->get_radar_output_information_switch();  // Query the key status every so often
  this->poll_time_base_func_check_ = true;      // Query the base functionality information at regular intervals
}

// main loop
void TI6432Component::loop() {
  uint8_t byte;
  static uint8_t current_byte_in_sync_word = 0; 
  static uint8_t current_num_tlv = 0;
  const int max_line_length = 80;
  static uint8_t buffer[max_line_length];

  // Is there data on the serial port
  while (this->available()) 
  {
    
   //  this->read_byte(&byte);
   //  ESP_LOGD(TAG, "0x%x", byte);

   //  memset(buffer, 0, max_line_length);
   //  this->read_array(buffer, max_line_length);
   //  for (uint8_t i=0; i<max_line_length; i=i+4)
   //  {
   //    ESP_LOGD(TAG, "0x%x, 0x%x, 0x%x, 0x%x", buffer[i], buffer[i+1], buffer[i+2], buffer[i+3]);
   //  }

    switch (this->pos_in_frame)
    {
    case FRAME_IN_IDLE:
    {
      this->read_byte(&byte);
      if (FRAME_HEADER_MAGIC_WORD_VALUE[current_byte_in_sync_word] == byte) 
      {
         current_byte_in_sync_word += 1;
      }
      else
      {
         // if not matching current byte, see if it matches first byte
         if (FRAME_HEADER_MAGIC_WORD_VALUE[0] == byte) 
         {
            current_byte_in_sync_word = 1;
         }
      }
      if(current_byte_in_sync_word == FRAME_MAGIC_WORD_LEN)
      {
         // whole sync word matched, start whole header
         this->pos_in_frame        = FRAME_IN_HEADER;
         current_byte_in_sync_word = 0;
         ESP_LOGD(TAG, "A new frame found!");
      }
    }
    break;
    case FRAME_IN_HEADER:
    {
      // read in whole frame header
      this->read_array(((uint8_t *)&(this->frame_header.version)), (FRAME_HEADER_LEN - FRAME_MAGIC_WORD_LEN));
      ESP_LOGD(TAG, "Frame header read in, numTLVs==%d", this->frame_header.numTLVs);
      if (this->frame_header.numTLVs > 0)
      {
         // there are TLVs following
         this->pos_in_frame = FRAME_IN_TLV;
      }
    }
    break;
    case FRAME_IN_TLV:
    {
      MESSAGE_TLV current_message;
      // read in TL
      this->read_array(((uint8_t *)&(current_message.tl)), sizeof(MmwDemo_output_message_tl));
      
      if (current_message.tl.length <= MESSAGE_MAX_V_SIZE)
      {
         ESP_LOGD(TAG, "TLV: number=%d, type=%d, length=%d", current_num_tlv, current_message.tl.type, current_message.tl.length);
         // read in V
         this->read_array(((uint8_t *)&(current_message.v)), current_message.tl.length);
         this->message_tlv.push_back(current_message);
      }
      else
      {
         // length is invalid, skip this TLV
         ESP_LOGD(TAG, "skip Invalid TLV: number=%d, type=%d, length=%d", current_num_tlv, current_message.tl.type, current_message.tl.length);
      }
      current_num_tlv += 1;
      if (current_num_tlv >= this->frame_header.numTLVs)
      {
         //this frame is over
         ESP_LOGD(TAG, "Whole frame is read in.");
         this->handle_frame();
         //prepare for next frame
         this->pos_in_frame = FRAME_TO_RESET;
      }
    }
    break;
    default:
    break;
    }
    if (this->pos_in_frame == FRAME_TO_RESET)
    {
      // error happens, or frame is ended
      ESP_LOGD(TAG, "Reset to prepare for next frame.");
      this->pos_in_frame            = FRAME_IN_IDLE;     
      current_num_tlv               = 0;
      current_byte_in_sync_word     = 0;
      memset(&this->frame_header, 0, sizeof(this->frame_header));
      this->message_tlv.clear();

      ///// temp code, to clear out all results for next frame
      this->zone_presence.clear();
      this->targets.clear();
      /////this->indexes.clear();
      /////this->class_outcome.clear();

      // break; // break from while, to avoid blocking other tasks.
    }
  } //end of while
}


void TI6432Component::handle_frame(void)
{
   for (auto &tlv: this->message_tlv)
   {
      switch (tlv.tl.type)
      {
         case MMWDEMO_OUTPUT_EXT_MSG_ENHANCED_PRESENCE_INDICATION: // presence detection TLV, 315, 
         {
            this->handle_ext_msg_enhanced_presence_indication(tlv.v, tlv.tl.length);
         }
         break;
         case MMWDEMO_OUTPUT_EXT_MSG_TARGET_LIST: // group tracker data, 308, 
         {
            this->handle_ext_msg_target_list(tlv.v, tlv.tl.length);
         }
         break;
         case MMWDEMO_OUTPUT_EXT_MSG_TARGET_INDEX: // group tracker data, 309
         {
            this->handle_ext_msg_target_index(tlv.v, tlv.tl.length);
         }
         break;
         case MMWDEMO_OUTPUT_EXT_MSG_CLASSIFIER_INFO: // classifier output, 317
         {
            this->handle_ext_msg_classifier_info(tlv.v, tlv.tl.length);
         }
         break;
         case MMWDEMO_OUTPUT_EXT_MSG_DETECTED_POINTS: // 301
         {
            this->handle_ext_msg_detected_points(tlv.v, tlv.tl.length);
         }
         break;
         case MMWDEMO_OUTPUT_EXT_MSG_RANGE_PROFILE_MAJOR: // 302
         {
            this->handle_ext_msg_range_profile_major(tlv.v, tlv.tl.length);
         }
         break;
         case MMWDEMO_OUTPUT_EXT_MSG_RANGE_PROFILE_MINOR: // 303
         {
            this->handle_ext_msg_range_profile_minor(tlv.v, tlv.tl.length);
         }
         break;
         case MMWDEMO_OUTPUT_MSG_EXT_STATS: // 306
         {
            this->handle_msg_ext_stats(tlv.v, tlv.tl.length);
         }
         break;
         default:
         {
            ESP_LOGD(TAG, "handle_frame: unknown TLV type:%d", tlv.tl.type);
         }
         break;
      }
   }
}

void TI6432Component::handle_ext_msg_enhanced_presence_indication(uint8_t *data, uint32_t length)
{
   // (1 + ceiling(NumberOfZones/4) x 1 Byte
   // This TLV contains the presence information for number of zones defined in the radar scene. 
   // The information for each zone is represented with 2 bits, packed in bytes starting from LSB position towards MSB.
   // 0: no detection, 1: minor detected, 2: major detected, 3: major and minor detected
   uint8_t numZones = data[0];
   ESP_LOGD(TAG, "TLV presence indication: numZones=%d", numZones);
   uint8_t *pDetection = &data[1];
   for (uint8_t i=0; i<numZones; i++)
   {
      uint8_t idx = i/4;
      uint8_t value = pDetection[idx] >> ((i*2) % 8) & 3;
      this->zone_presence.push_back(value); 
      ESP_LOGD(TAG, "TLV presence indication: zone=%d, value=%d", i, value);
      this->motion_status_text_sensor_->publish_state(S_MOTION_STATUS_STR[value]);
      if (i == 0)
      {
         this->custom_presence_of_detection_sensor_->publish_state(value);
      }
      else if (i == 1)
      {
         this->movement_signs_sensor_->publish_state(value);
      }
      else if (i ==2)
      {
      this->custom_motion_distance_sensor_->publish_state(value);
      }
      
   }
}

void TI6432Component::handle_ext_msg_target_list(uint8_t *data, uint32_t length)
{
   // Number of Targets x 112 Bytes
   // Set of values that describe the 3 dimensional position and motion of the target being tracked, 
   // include position, velocity, acceleration, as well as error measurements for the track.

// # Decode 3D People Counting Target List TLV
// # MMWDEMO_OUTPUT_MSG_TRACKERPROC_3D_TARGET_LIST
// #3D Struct format
// #uint32_t     tid;     /*! @brief   tracking ID */
// #float        posX;    /*! @brief   Detected target X coordinate, in m */
// #float        posY;    /*! @brief   Detected target Y coordinate, in m */
// #float        posZ;    /*! @brief   Detected target Z coordinate, in m */
// #float        velX;    /*! @brief   Detected target X velocity, in m/s */
// #float        velY;    /*! @brief   Detected target Y velocity, in m/s */
// #float        velZ;    /*! @brief   Detected target Z velocity, in m/s */
// #float        accX;    /*! @brief   Detected target X acceleration, in m/s2 */
// #float        accY;    /*! @brief   Detected target Y acceleration, in m/s2 */
// #float        accZ;    /*! @brief   Detected target Z acceleration, in m/s2 */
// #float        ec[16];  /*! @brief   Target Error covarience matrix, [4x4 float], in row major order, range, azimuth, elev, doppler */
// #float        g;
// #float        confidenceLevel;    /*! @brief   Tracker confidence metric*/

   uint32_t numDetectedTargets = length/sizeof(trackerProc_Target); 
   ESP_LOGD(TAG, "TLV target list: numDetectedTargets=%d", numDetectedTargets);
   if (numDetectedTargets > MAX_TARGET_NUMBER)
   {
      ESP_LOGE(TAG, "TLV target list: too many targets (%d)", numDetectedTargets);
      return;
   }

   uint8_t buf[MAX_TARGET_NUMBER];
   for (uint32_t i=0; i<numDetectedTargets; i++)
   {
      trackerProc_Target oneTarget;
      memcpy((uint8_t *)&oneTarget, &data[i*sizeof(trackerProc_Target)], sizeof(trackerProc_Target));
      this->targets.push_back(oneTarget);
      /////todo: when to empty targets
      ESP_LOGD(TAG, "TLV target list: targetIndex=%d, targetId=%d", i, oneTarget.tid);
      buf[i] = oneTarget.tid; // save for following handling
   }

   // for each target already existed in previous frames, check if it still exists in the new frame,
   // if not found, remove the target from the list
   for (auto it = this->class_outcome.begin(); it != this->class_outcome.end();)
   {
      bool found = false;
      for (uint32_t i=0; i<numDetectedTargets; i++)
      {
         if (it->targetId == buf[i])
         {
            found = true;
            buf[i] = UNKNOWN_TARGET; //remove this target from buf
            ESP_LOGD(TAG, "TLV target list: found existing targetId=%d", it->targetId);
            break;
         }
      }
      if(!found)
      {
         ESP_LOGD(TAG, "TLV target list: delete targetId=%d", it->targetId);
         // this target not existed any more, remove all its data
         it = this->class_outcome.erase(it);
      }
      else
      {
         ++it;
      }
   }

   // until now, class_outcome removed non-existed targets from previous frames.
   // next, add in new target IDs found in the new frame
   for (uint32_t i=0; i<numDetectedTargets; i++)
   {
      if (buf[i] != UNKNOWN_TARGET)
      {
         // this is a new target
         CLASSIFICATION_DATA newClass;

         newClass.targetId = buf[i];
         newClass.validFrameNum = 0;
         memset(newClass.isHuman, 0, CLASSIFICATION_MAX_FRAMES);
         this->class_outcome.push_back(newClass);
         ESP_LOGD(TAG, "TLV target index: add new targetId=%d", buf[i]);
      }
   }
}

void TI6432Component::handle_ext_msg_target_index(uint8_t *data, uint32_t length)
{
   // // Number of Points x 1 Byte
   // // Contains target ID, allocating every point to a specific target or no target
   // if (length > MAX_TARGET_NUMBER)
   // {
   //    ESP_LOGE(TAG, "TLV target index: too many targets (%d)", length);
   //    return;
   // }
   // uint8_t buf[MAX_TARGET_NUMBER];
   
   // memcpy(buf, data, length); //copy over, so we can change

   // // for each target already existed in previous frames, check if it still exists in the new frame,
   // // if not found, remove the target from the list
   // for (auto it = this->class_outcome.begin(); it != this->class_outcome.end();)
   // {
   //    bool found = false;
   //    for (uint32_t i=0; i<length; i++)
   //    {
   //       if (it->targetId == buf[i])
   //       {
   //          found = true;
   //          buf[i] = UNKNOWN_TARGET; //remove this target from buf
   //          ESP_LOGD(TAG, "TLV target index: found existing targetId=%d", it->targetId);
   //          break;
   //       }
   //    }
   //    if(!found)
   //    {
   //       ESP_LOGD(TAG, "TLV target index: delete targetId=%d", it->targetId);
   //       // this target not existed any more, remove all its data
   //       it = this->class_outcome.erase(it);
   //    }
   //    else
   //    {
   //       ++it;
   //    }
   // }

   // // until now, class_outcome removed non-existed targets from previous frames.
   // // next, add in new target IDs found in the new frame
   // for (uint32_t i=0; i<length; i++)
   // {
   //    if (buf[i] != UNKNOWN_TARGET)
   //    {
   //       // this is a new target
   //       CLASSIFICATION_DATA newClass;

   //       newClass.targetId = buf[i];
   //       newClass.validFrameNum = 0;
   //       memset(newClass.isHuman, 0, CLASSIFICATION_MAX_FRAMES);
   //       this->class_outcome.push_back(newClass);
   //       ESP_LOGD(TAG, "TLV target index: add new targetId=%d", buf[i]);
   //    }
   // }
}

void TI6432Component::handle_ext_msg_classifier_info(uint8_t *data, uint32_t length)
{
   // numTrackedObjects * CLASSIFIER_NUM_CLASSES * 1 Byte
   // Array of classifier outcome in Q7 format, packed as classOutcome[targetIndex][classIndex]
   uint32_t numDetectedTargets = length/NUM_CLASSES_IN_CLASSIFIER;

   if (numDetectedTargets != this->class_outcome.size())
   {
      //by now, class_outcome should already updated according to 308
      //total target number should always match
      ESP_LOGE(TAG, "TLV classifier info: target number not match. in 317 =%d, in class_outcome = %d", this->class_outcome.size());
      return;
   }
   for (uint32_t i=0; i<numDetectedTargets; i++)
   {
      CLASS_PROBABILITY prob;

      prob.humanProb    = (float)data[i*2]   / 128;
      prob.nonHumanProb = (float)data[i*2+1] / 128;
      if (prob.humanProb != 0.5)
      {
         ESP_LOGD(TAG, "TLV classifier info: targetIndex=%d, human=%f, nonHuman=%f", i, prob.humanProb, prob.nonHumanProb);
         uint8_t isHumanIndex = this->class_outcome[i].validFrameNum;
         if (prob.humanProb >= CLASSIFIER_CONFIDENCE_SCORE)
         {
            // human detected
            if (isHumanIndex < CLASSIFICATION_MAX_FRAMES)
            {
               this->class_outcome[i].isHuman[isHumanIndex] = 1;
               this->class_outcome[i].validFrameNum ++;
            }
            else
            {
               //valid frame is full of class_outcome, remove the oldest one.
               for (uint8_t i=0; i<CLASSIFICATION_MAX_FRAMES-1; i++ )
               {
                  this->class_outcome[i] = this->class_outcome[i+1];
               }
               this->class_outcome[i].isHuman[CLASSIFICATION_MAX_FRAMES] = 1;
            }

            if (this->class_outcome[i].validFrameNum >= CLASSIFICATION_MAX_FRAMES)
            {
               int8_t sum = 0;
               for (uint8_t i=0; i<CLASSIFICATION_MAX_FRAMES; i++)
               {
                  sum += this->class_outcome[i].isHuman[i];
               }
               if (sum > 0)
               {
                  //overall, human detected, report
                  this->custom_spatial_static_value_sensor_->publish_state(this->class_outcome[i].targetId);
                  this->custom_spatial_motion_value_sensor_->publish_state(1);
                  ESP_LOGD(TAG, "TLV classifier info: human detected. targetId=%d", this->class_outcome[i].targetId);
               }
               // no need to report non-human, because it should be reported before
               // only possible change is from non-human to human
            }
         }
         else if (prob.nonHumanProb >= CLASSIFIER_CONFIDENCE_SCORE)
         {
            // non-human detected
            if (isHumanIndex < CLASSIFICATION_MAX_FRAMES)
            {
               this->class_outcome[i].isHuman[isHumanIndex] = -1;
               this->class_outcome[i].validFrameNum ++;
            }
            else
            {
               //valid frame is full of class_outcome, remove the oldest one.
               for (uint8_t i=0; i<CLASSIFICATION_MAX_FRAMES-1; i++ )
               {
                  this->class_outcome[i] = this->class_outcome[i+1];
               }
               this->class_outcome[i].isHuman[CLASSIFICATION_MAX_FRAMES] = -1;
            }

            if (this->class_outcome[i].validFrameNum >= CLASSIFICATION_MAX_FRAMES)
            {
               int8_t sum = 0;
               for (uint8_t i=0; i<CLASSIFICATION_MAX_FRAMES; i++)
               {
                  sum += this->class_outcome[i].isHuman[i];
               }
               if (sum < 0)
               {
                  //overall, non-human detected, report
                  this->custom_spatial_static_value_sensor_->publish_state(this->class_outcome[i].targetId);
                  this->custom_spatial_motion_value_sensor_->publish_state(0);
                  ESP_LOGD(TAG, "TLV classifier info: non-human detected. targetId=%d", this->class_outcome[i].targetId);
               }
               // no need to report human, because it should be reported before
               // only possible change is from human to non-human
            }
         }
         else
         {
            // unknown, ignore this frame
         }
      }
   }
}

void TI6432Component::handle_ext_msg_detected_points(uint8_t *data, uint32_t length)
{

}
void TI6432Component::handle_ext_msg_range_profile_major(uint8_t *data, uint32_t length)
{

}
void TI6432Component::handle_ext_msg_range_profile_minor(uint8_t *data, uint32_t length)
{

}
void TI6432Component::handle_msg_ext_stats(uint8_t *data, uint32_t length)
{

}

// Calculate CRC check digit
static uint8_t get_frame_crc_sum(const uint8_t *data, int len) {
  unsigned int crc_sum = 0;
  for (int i = 0; i < len - 3; i++) {
    crc_sum += data[i];
  }
  return crc_sum & 0xff;
}

// Check that the check digit is correct
static int get_frame_check_status(uint8_t *data, int len) {
  uint8_t crc_sum = get_frame_crc_sum(data, len);
  uint8_t verified = data[len - 3];
  return (verified == crc_sum) ? 1 : 0;
}

// Sending data frames
void TI6432Component::send_query_(uint8_t *query, size_t string_length) 
{ 
//   this->write_array(query, string_length); 
}

// Send Heartbeat Packet Command
void TI6432Component::get_heartbeat_packet() {
  uint8_t send_data_len = 10;
  uint8_t send_data[10] = {0x53, 0x59, 0x01, 0x01, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
  send_data[7] = get_frame_crc_sum(send_data, send_data_len);
  this->send_query_(send_data, send_data_len);
}

// Issuance of the underlying open parameter query command
void TI6432Component::get_radar_output_information_switch() {
  uint8_t send_data_len = 10;
  uint8_t send_data[10] = {0x53, 0x59, 0x08, 0x80, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
  send_data[7] = get_frame_crc_sum(send_data, send_data_len);
  this->send_query_(send_data, send_data_len);
}

// Issuance of product model orders
void TI6432Component::get_product_mode() {
  uint8_t send_data_len = 10;
  uint8_t send_data[10] = {0x53, 0x59, 0x02, 0xA1, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
  send_data[7] = get_frame_crc_sum(send_data, send_data_len);
  this->send_query_(send_data, send_data_len);
}

// Issuing the Get Product ID command
void TI6432Component::get_product_id() {
  uint8_t send_data_len = 10;
  uint8_t send_data[10] = {0x53, 0x59, 0x02, 0xA2, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
  send_data[7] = get_frame_crc_sum(send_data, send_data_len);
  this->send_query_(send_data, send_data_len);
}

// Issuing hardware model commands
void TI6432Component::get_hardware_model() {
  uint8_t send_data_len = 10;
  uint8_t send_data[10] = {0x53, 0x59, 0x02, 0xA3, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
  send_data[7] = get_frame_crc_sum(send_data, send_data_len);
  this->send_query_(send_data, send_data_len);
}

// Issuing software version commands
void TI6432Component::get_firmware_version() {
  uint8_t send_data_len = 10;
  uint8_t send_data[10] = {0x53, 0x59, 0x02, 0xA4, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
  send_data[7] = get_frame_crc_sum(send_data, send_data_len);
  this->send_query_(send_data, send_data_len);
}

void TI6432Component::get_human_status() {
  uint8_t send_data_len = 10;
  uint8_t send_data[10] = {0x53, 0x59, 0x80, 0x81, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
  send_data[7] = get_frame_crc_sum(send_data, send_data_len);
  this->send_query_(send_data, send_data_len);
}

void TI6432Component::get_human_motion_info() {
  uint8_t send_data_len = 10;
  uint8_t send_data[10] = {0x53, 0x59, 0x80, 0x82, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
  send_data[7] = get_frame_crc_sum(send_data, send_data_len);
  this->send_query_(send_data, send_data_len);
}

void TI6432Component::get_body_motion_params() {
  uint8_t send_data_len = 10;
  uint8_t send_data[10] = {0x53, 0x59, 0x80, 0x83, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
  send_data[7] = get_frame_crc_sum(send_data, send_data_len);
  this->send_query_(send_data, send_data_len);
}

void TI6432Component::get_keep_away() {
  uint8_t send_data_len = 10;
  uint8_t send_data[10] = {0x53, 0x59, 0x80, 0x8B, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
  send_data[7] = get_frame_crc_sum(send_data, send_data_len);
  this->send_query_(send_data, send_data_len);
}

void TI6432Component::get_scene_mode() {
  uint8_t send_data_len = 10;
  uint8_t send_data[10] = {0x53, 0x59, 0x05, 0x87, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
  send_data[7] = get_frame_crc_sum(send_data, send_data_len);
  this->send_query_(send_data, send_data_len);
}

void TI6432Component::get_sensitivity() {
  uint8_t send_data_len = 10;
  uint8_t send_data[10] = {0x53, 0x59, 0x05, 0x88, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
  send_data[7] = get_frame_crc_sum(send_data, send_data_len);
  this->send_query_(send_data, send_data_len);
}

void TI6432Component::get_unmanned_time() {
  uint8_t send_data_len = 10;
  uint8_t send_data[10] = {0x53, 0x59, 0x80, 0x8a, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
  send_data[7] = get_frame_crc_sum(send_data, send_data_len);
  this->send_query_(send_data, send_data_len);
}

void TI6432Component::get_custom_mode() {
  uint8_t send_data_len = 10;
  uint8_t send_data[10] = {0x53, 0x59, 0x05, 0x89, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
  send_data[7] = get_frame_crc_sum(send_data, send_data_len);
  this->send_query_(send_data, send_data_len);
}

void TI6432Component::get_existence_boundary() {
  uint8_t send_data_len = 10;
  uint8_t send_data[10] = {0x53, 0x59, 0x08, 0x8A, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
  send_data[7] = get_frame_crc_sum(send_data, send_data_len);
  this->send_query_(send_data, send_data_len);
}

void TI6432Component::get_motion_boundary() {
  uint8_t send_data_len = 10;
  uint8_t send_data[10] = {0x53, 0x59, 0x08, 0x8B, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
  send_data[7] = get_frame_crc_sum(send_data, send_data_len);
  this->send_query_(send_data, send_data_len);
}

void TI6432Component::get_spatial_static_value() {
  uint8_t send_data_len = 10;
  uint8_t send_data[10] = {0x53, 0x59, 0x08, 0x81, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
  send_data[7] = get_frame_crc_sum(send_data, send_data_len);
  this->send_query_(send_data, send_data_len);
}

void TI6432Component::get_spatial_motion_value() {
  uint8_t send_data_len = 10;
  uint8_t send_data[10] = {0x53, 0x59, 0x08, 0x82, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
  send_data[7] = get_frame_crc_sum(send_data, send_data_len);
  this->send_query_(send_data, send_data_len);
}

void TI6432Component::get_distance_of_static_object() {
  uint8_t send_data_len = 10;
  uint8_t send_data[10] = {0x53, 0x59, 0x08, 0x83, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
  send_data[7] = get_frame_crc_sum(send_data, send_data_len);
  this->send_query_(send_data, send_data_len);
}

void TI6432Component::get_distance_of_moving_object() {
  uint8_t send_data_len = 10;
  uint8_t send_data[10] = {0x53, 0x59, 0x08, 0x84, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
  send_data[7] = get_frame_crc_sum(send_data, send_data_len);
  this->send_query_(send_data, send_data_len);
}

void TI6432Component::get_target_movement_speed() {
  uint8_t send_data_len = 10;
  uint8_t send_data[10] = {0x53, 0x59, 0x08, 0x85, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
  send_data[7] = get_frame_crc_sum(send_data, send_data_len);
  this->send_query_(send_data, send_data_len);
}

void TI6432Component::get_existence_threshold() {
  uint8_t send_data_len = 10;
  uint8_t send_data[10] = {0x53, 0x59, 0x08, 0x88, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
  send_data[7] = get_frame_crc_sum(send_data, send_data_len);
  this->send_query_(send_data, send_data_len);
}

void TI6432Component::get_motion_threshold() {
  uint8_t send_data_len = 10;
  uint8_t send_data[10] = {0x53, 0x59, 0x08, 0x89, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
  send_data[7] = get_frame_crc_sum(send_data, send_data_len);
  this->send_query_(send_data, send_data_len);
}

void TI6432Component::get_motion_trigger_time() {
  uint8_t send_data_len = 10;
  uint8_t send_data[10] = {0x53, 0x59, 0x08, 0x8C, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
  send_data[7] = get_frame_crc_sum(send_data, send_data_len);
  this->send_query_(send_data, send_data_len);
}

void TI6432Component::get_motion_to_rest_time() {
  uint8_t send_data_len = 10;
  uint8_t send_data[10] = {0x53, 0x59, 0x08, 0x8D, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
  send_data[7] = get_frame_crc_sum(send_data, send_data_len);
  this->send_query_(send_data, send_data_len);
}

void TI6432Component::get_custom_unman_time() {
  uint8_t send_data_len = 10;
  uint8_t send_data[10] = {0x53, 0x59, 0x08, 0x8E, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
  send_data[7] = get_frame_crc_sum(send_data, send_data_len);
  this->send_query_(send_data, send_data_len);
}

// Logic of setting: After setting, query whether the setting is successful or not!

void TI6432Component::set_underlying_open_function(bool enable) {
//   uint8_t underlyswitch_on[] = {0x53, 0x59, 0x08, 0x00, 0x00, 0x01, 0x01, 0xB6, 0x54, 0x43};
//   uint8_t underlyswitch_off[] = {0x53, 0x59, 0x08, 0x00, 0x00, 0x01, 0x00, 0xB5, 0x54, 0x43};
//   if (enable) {
//     send_query_(underlyswitch_on, sizeof(underlyswitch_on));
//   } else {
//     send_query_(underlyswitch_off, sizeof(underlyswitch_off));
//   }
//   if (this->keep_away_text_sensor_ != nullptr) {
//     this->keep_away_text_sensor_->publish_state("");
//   }
//   if (this->motion_status_text_sensor_ != nullptr) {
//     this->motion_status_text_sensor_->publish_state("");
//   }
//   if (this->custom_spatial_static_value_sensor_ != nullptr) {
//     this->custom_spatial_static_value_sensor_->publish_state(NAN);
//   }
//   if (this->custom_spatial_motion_value_sensor_ != nullptr) {
//     this->custom_spatial_motion_value_sensor_->publish_state(NAN);
//   }
//   if (this->custom_motion_distance_sensor_ != nullptr) {
//     this->custom_motion_distance_sensor_->publish_state(NAN);
//   }
//   if (this->custom_presence_of_detection_sensor_ != nullptr) {
//     this->custom_presence_of_detection_sensor_->publish_state(NAN);
//   }
//   if (this->custom_motion_speed_sensor_ != nullptr) {
//     this->custom_motion_speed_sensor_->publish_state(NAN);
//   }
}

void TI6432Component::set_scene_mode(uint8_t value) {
//   uint8_t send_data_len = 10;
//   uint8_t send_data[10] = {0x53, 0x59, 0x05, 0x07, 0x00, 0x01, static_cast<uint8_t>(value), 0x00, 0x54, 0x43};
//   send_data[7] = get_frame_crc_sum(send_data, send_data_len);
//   this->send_query_(send_data, send_data_len);
//   if (this->custom_mode_number_ != nullptr) {
//     this->custom_mode_number_->publish_state(0);
//   }
//   if (this->custom_mode_num_sensor_ != nullptr) {
//     this->custom_mode_num_sensor_->publish_state(0);
//   }
//   this->get_scene_mode();
//   this->get_sensitivity();
//   this->get_custom_mode();
//   this->get_existence_boundary();
//   this->get_motion_boundary();
//   this->get_existence_threshold();
//   this->get_motion_threshold();
//   this->get_motion_trigger_time();
//   this->get_motion_to_rest_time();
//   this->get_custom_unman_time();
}

void TI6432Component::set_sensitivity(uint8_t value) {
//   if (value == 0x00)
//     return;
//   uint8_t send_data_len = 10;
//   uint8_t send_data[10] = {0x53, 0x59, 0x05, 0x08, 0x00, 0x01, value, 0x00, 0x54, 0x43};
//   send_data[7] = get_frame_crc_sum(send_data, send_data_len);
//   this->send_query_(send_data, send_data_len);
//   this->get_scene_mode();
//   this->get_sensitivity();
}

void TI6432Component::set_restart() {
//   uint8_t send_data_len = 10;
//   uint8_t send_data[10] = {0x53, 0x59, 0x01, 0x02, 0x00, 0x01, 0x0F, 0xBF, 0x54, 0x43};
//   this->send_query_(send_data, send_data_len);
//   this->check_dev_inf_sign_ = true;
}

void TI6432Component::set_unman_time(uint8_t value) {
//   uint8_t send_data_len = 10;
//   uint8_t send_data[10] = {0x53, 0x59, 0x80, 0x0a, 0x00, 0x01, static_cast<uint8_t>(value), 0x00, 0x54, 0x43};
//   send_data[7] = get_frame_crc_sum(send_data, send_data_len);
//   this->send_query_(send_data, send_data_len);
//   this->get_unmanned_time();
}

void TI6432Component::set_custom_mode(uint8_t mode) {
//   if (mode == 0) {
//     this->set_custom_end_mode();  // Equivalent to end setting
//     if (this->custom_mode_number_ != nullptr) {
//       this->custom_mode_number_->publish_state(0);
//     }
//     return;
//   }
//   uint8_t send_data_len = 10;
//   uint8_t send_data[10] = {0x53, 0x59, 0x05, 0x09, 0x00, 0x01, mode, 0x00, 0x54, 0x43};
//   send_data[7] = get_frame_crc_sum(send_data, send_data_len);
//   this->send_query_(send_data, send_data_len);
//   this->get_existence_boundary();
//   this->get_motion_boundary();
//   this->get_existence_threshold();
//   this->get_motion_threshold();
//   this->get_motion_trigger_time();
//   this->get_motion_to_rest_time();
//   this->get_custom_unman_time();
//   this->get_custom_mode();
//   this->get_scene_mode();
//   this->get_sensitivity();
}

void TI6432Component::set_custom_end_mode() {
//   uint8_t send_data_len = 10;
//   uint8_t send_data[10] = {0x53, 0x59, 0x05, 0x0a, 0x00, 0x01, 0x0F, 0xCB, 0x54, 0x43};
//   this->send_query_(send_data, send_data_len);
//   if (this->custom_mode_number_ != nullptr) {
//     this->custom_mode_number_->publish_state(0);  // Clear setpoints
//   }
//   this->get_existence_boundary();
//   this->get_motion_boundary();
//   this->get_existence_threshold();
//   this->get_motion_threshold();
//   this->get_motion_trigger_time();
//   this->get_motion_to_rest_time();
//   this->get_custom_unman_time();
//   this->get_custom_mode();
//   this->get_scene_mode();
//   this->get_sensitivity();
}

void TI6432Component::set_existence_boundary(uint8_t value) {
//   if ((this->custom_mode_num_sensor_ != nullptr) && (this->custom_mode_num_sensor_->state == 0))
//     return;  // You'll have to check that you're in custom mode to set it up
//   uint8_t send_data_len = 10;
//   uint8_t send_data[10] = {0x53, 0x59, 0x08, 0x0A, 0x00, 0x01, static_cast<uint8_t>(value + 1), 0x00, 0x54, 0x43};
//   send_data[7] = get_frame_crc_sum(send_data, send_data_len);
//   this->send_query_(send_data, send_data_len);
//   this->get_existence_boundary();
}

void TI6432Component::set_motion_boundary(uint8_t value) {
//   if ((this->custom_mode_num_sensor_ != nullptr) && (this->custom_mode_num_sensor_->state == 0))
//     return;  // You'll have to check that you're in custom mode to set it up
//   uint8_t send_data_len = 10;
//   uint8_t send_data[10] = {0x53, 0x59, 0x08, 0x0B, 0x00, 0x01, static_cast<uint8_t>(value + 1), 0x00, 0x54, 0x43};
//   send_data[7] = get_frame_crc_sum(send_data, send_data_len);
//   this->send_query_(send_data, send_data_len);
//   this->get_motion_boundary();
}

void TI6432Component::set_existence_threshold(int value) {
//   if ((this->custom_mode_num_sensor_ != nullptr) && (this->custom_mode_num_sensor_->state == 0))
//     return;  // You'll have to check that you're in custom mode to set it up
//   uint8_t send_data_len = 10;
//   uint8_t send_data[10] = {0x53, 0x59, 0x08, 0x08, 0x00, 0x01, (uint8_t) value, 0x00, 0x54, 0x43};
//   send_data[7] = get_frame_crc_sum(send_data, send_data_len);
//   this->send_query_(send_data, send_data_len);
//   this->get_existence_threshold();
}

void TI6432Component::set_motion_threshold(int value) {
//   if ((this->custom_mode_num_sensor_ != nullptr) && (this->custom_mode_num_sensor_->state == 0))
//     return;  // You'll have to check that you're in custom mode to set it up
//   uint8_t send_data_len = 10;
//   uint8_t send_data[10] = {0x53, 0x59, 0x08, 0x09, 0x00, 0x01, (uint8_t) value, 0x00, 0x54, 0x43};
//   send_data[7] = get_frame_crc_sum(send_data, send_data_len);
//   this->send_query_(send_data, send_data_len);
//   this->get_motion_threshold();
}

void TI6432Component::set_motion_trigger_time(int value) {
//   if ((this->custom_mode_num_sensor_ != nullptr) && (this->custom_mode_num_sensor_->state == 0))
//     return;  // You'll have to check that you're in custom mode to set it up
//   int h24_num = (value >> 24) & 0xff;
//   int h16_num = (value >> 16) & 0xff;
//   int h8_num = (value >> 8) & 0xff;
//   int l8_num = value & 0xff;
//   uint8_t send_data_len = 13;
//   uint8_t send_data[13] = {
//       0x53, 0x59, 0x08, 0x0C, 0x00, 0x04, (uint8_t) h24_num, (uint8_t) h16_num, (uint8_t) h8_num, (uint8_t) l8_num,
//       0x00, 0x54, 0x43};
//   send_data[10] = get_frame_crc_sum(send_data, send_data_len);
//   this->send_query_(send_data, send_data_len);
//   this->get_motion_trigger_time();
}

void TI6432Component::set_motion_to_rest_time(int value) {
//   if ((this->custom_mode_num_sensor_ != nullptr) && (this->custom_mode_num_sensor_->state == 0))
//     return;  // You'll have to check that you're in custom mode to set it up
//   int h24_num = (value >> 24) & 0xff;
//   int h16_num = (value >> 16) & 0xff;
//   int h8_num = (value >> 8) & 0xff;
//   int l8_num = value & 0xff;
//   uint8_t send_data_len = 13;
//   uint8_t send_data[13] = {
//       0x53, 0x59, 0x08, 0x0D, 0x00, 0x04, (uint8_t) h24_num, (uint8_t) h16_num, (uint8_t) h8_num, (uint8_t) l8_num,
//       0x00, 0x54, 0x43};
//   send_data[10] = get_frame_crc_sum(send_data, send_data_len);
//   this->send_query_(send_data, send_data_len);
//   this->get_motion_to_rest_time();
}

void TI6432Component::set_custom_unman_time(int value) {
//   if ((this->custom_mode_num_sensor_ != nullptr) && (this->custom_mode_num_sensor_->state == 0))
//     return;  // You'll have to check that you're in custom mode to set it up
//   value *= 1000;
//   int h24_num = (value >> 24) & 0xff;
//   int h16_num = (value >> 16) & 0xff;
//   int h8_num = (value >> 8) & 0xff;
//   int l8_num = value & 0xff;
//   uint8_t send_data_len = 13;
//   uint8_t send_data[13] = {
//       0x53, 0x59, 0x08, 0x0E, 0x00, 0x04, (uint8_t) h24_num, (uint8_t) h16_num, (uint8_t) h8_num, (uint8_t) l8_num,
//       0x00, 0x54, 0x43};
//   send_data[10] = get_frame_crc_sum(send_data, send_data_len);
//   this->send_query_(send_data, send_data_len);
//   this->get_custom_unman_time();
}
}  // namespace ti6432
}  // namespace esphome
