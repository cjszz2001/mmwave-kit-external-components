#include "esphome/core/log.h"
#include "ti6432.h"

#include "freertos/timers.h"

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

TimerHandle_t  tracking_timer[MAX_TARGET_NUMBER];
static uint32_t  resetTarget = UNKNOWN_TARGET;

static void timer_call_back(TimerHandle_t xTimer);


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
// how many humans detected in zone 1
  LOG_SENSOR(" ", "Custom Motion Speed Sensor", this->custom_motion_speed_sensor_);
// how many humans detected in zone 2
  LOG_SENSOR(" ", "Custom Mode Num Sensor", this->custom_mode_num_sensor_);
// how many humans entered the room
  LOG_SENSOR(" ", "Human entered in room", this->human_entered_in_room_sensor_);
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
  this->pos_in_frame = FRAME_IN_IDLE;

  this->set_interval(8000, [this]() { this->update_(); });


  //this->reported_human_number = 0;
  memset(this->reported_human_number, 0, sizeof(this->reported_human_number));
  this->custom_motion_speed_sensor_->publish_state(0);
  this->custom_mode_num_sensor_->publish_state(0);
  this->custom_spatial_static_value_sensor_->publish_state(255);
  this->custom_spatial_motion_value_sensor_->publish_state(0);
  this->human_entered_in_room_sensor_->publish_state(0);
  this->class_outcome.clear();
  this->entry_targets.clear();
  this->human_count_in_room = 0;

  for (uint8_t i=0; i<MAX_TARGET_NUMBER; i++)
  {
     tracking_timer[i] = xTimerCreate("tracking_timer", pdMS_TO_TICKS(TRACKING_TIMEOUT_MS), false, NULL, timer_call_back);
     if( tracking_timer[i] == NULL )
     {
         /* The timer was not created. */
         ESP_LOGE(TAG, "Tracking timer create failed. index=%d", i);
     }
     else
     {
        // initiate all timer ID to INVALID.
        vTimerSetTimerID( tracking_timer[i], (void *)INVALID_TIMER_ID );
     }
  }

}

void timer_call_back(TimerHandle_t xTimer)
{
   uint32_t targetId = (uint32_t)pvTimerGetTimerID(xTimer);
   ESP_LOGI(TAG, "Target ID %d disappear...", targetId);
   vTimerSetTimerID( xTimer, (void *)INVALID_TIMER_ID );

   resetTarget = targetId;
}

bool findAvailableTimerIndex(uint8_t *index)
{
   for (uint8_t i=0; i<MAX_TARGET_NUMBER; i++)
   {
      if ((uint32_t)pvTimerGetTimerID(tracking_timer[i]) == INVALID_TIMER_ID )
      {
         *index = i;
         return true;
      }
   }
   return false;
}

// Timed polling of radar data
void TI6432Component::update_() {
  // this->get_radar_output_information_switch();  // Query the key status every so often
}

// main loop
void TI6432Component::loop() {
   uint8_t byte;
   uint8_t current_byte_in_sync_word = 0;
   static uint8_t current_num_tlv = 0;

   uint32_t ms_time = millis();

   // Is there data on the serial port
   while (this->available())
   {
      ESP_LOGD(TAG, "data on UART.");
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
            else
            {
               current_byte_in_sync_word = 0;
            }
         }
         if (current_byte_in_sync_word == FRAME_MAGIC_WORD_LEN)
         {
            // whole sync word matched, start whole header
            this->pos_in_frame = FRAME_IN_WAIT4HEADER;
            // this->pos_in_frame = FRAME_IN_HEADER;
            ESP_LOGD(TAG, "A new frame found!");
         }
         else if (current_byte_in_sync_word == 0)
         {
            this->pos_in_frame = FRAME_KEEP_IN_IDLE;
         }
         ESP_LOGD(TAG, "current_byte_in_sync_word=%d", current_byte_in_sync_word);
      }
      break;
      case FRAME_IN_HEADER:
      {
         // read in whole frame header
         this->read_array_with_delay(((uint8_t *)&(this->frame_header.version)), (FRAME_HEADER_LEN - FRAME_MAGIC_WORD_LEN));
         ESP_LOGD(TAG, "Frame header read in, numTLVs==%d", this->frame_header.numTLVs);
         if (this->frame_header.numTLVs > 0)
         {
            // there are TLVs following
            this->pos_in_frame = FRAME_IN_TL;
         }
      }
      break;
      case FRAME_IN_TL:
      {
         if (current_num_tlv >= this->frame_header.numTLVs)
         {
            // this frame is over
            //this->handle_frame();
            // prepare for next frame
            this->pos_in_frame = FRAME_TO_RESET;
            break;
         }
         memset((void *)(&this->current_message), 0, sizeof(this->current_message));
         // read in TL
         this->read_array_with_delay(((uint8_t *)&(this->current_message.tl)), sizeof(MmwDemo_output_message_tl));

         if (this->current_message.tl.type > MMWDEMO_OUTPUT_MSG_MAX || this->current_message.tl.length > MESSAGE_MAX_V_SIZE)
         {
            ESP_LOGE(TAG, "skip Invalid TLV: number=%d, type=%d, length=%d", current_num_tlv, current_message.tl.type, current_message.tl.length);
            this->pos_in_frame = FRAME_TO_RESET;
         }
         else
         {
            ESP_LOGD(TAG, "TLV: number=%d, type=%d, length=%d", current_num_tlv, this->current_message.tl.type, this->current_message.tl.length);
            this->pos_in_frame = FRAME_IN_WAIT4V;
         }
         ESP_LOGD(TAG, "time in TL=%d", millis() - ms_time);
      }
      break;
      case FRAME_IN_V:
      {
         // read in V
         this->read_array(((uint8_t *)&(this->current_message.v)), this->current_message.tl.length);
         //this->read_big_data_from_uart(((uint8_t *)&(this->current_message.v)), this->current_message.tl.length);
         
         this->pos_in_frame = FRAME_IN_WAIT4HANDLE;
      }
      break;
      case FRAME_IN_HANDLE:
      {

         // this->message_tlv.push_back(this->current_message);
         this->handle_tlv(this->current_message);
         ESP_LOGD(TAG, "time in handle=%d", millis() - ms_time);

         current_num_tlv += 1;
         this->pos_in_frame = FRAME_IN_TL;
      }
      break;
      default:
         break;
      }

      if (this->pos_in_frame == FRAME_IN_WAIT4V)
      {
         if (this->available() >= this->current_message.tl.length)
         {
            // UART data is ready
            this->pos_in_frame = FRAME_IN_V;
         }
         break; // break from while loop
      }
      else if (this->pos_in_frame == FRAME_KEEP_IN_IDLE)
      {
         this->pos_in_frame = FRAME_IN_IDLE;
         break; // break from while loop
      }
      else if (this->pos_in_frame == FRAME_IN_WAIT4HEADER)
      {
         this->pos_in_frame = FRAME_IN_HEADER;
         break; // break from while loop
      }
      else if (this->pos_in_frame == FRAME_IN_WAIT4HANDLE)
      {
         this->pos_in_frame = FRAME_IN_HANDLE;
         break; //break from while loop
      }
      else if(this->pos_in_frame == FRAME_TO_RESET)
      {
         // error happens, or frame is ended
         ESP_LOGD(TAG, "Reset to prepare for next frame.");
         this->pos_in_frame = FRAME_IN_IDLE;
         current_num_tlv = 0;
         memset(&this->frame_header, 0, sizeof(this->frame_header));
         memset(&this->current_message, 0, sizeof(this->current_message));
         ///this->message_tlv.clear();

         ///// temp code, to clear out all results for next frame
         this->zone_presence.clear();
         break; // break from while loop
      }
      else
      {

      }
   } // end of while

   if (resetTarget != UNKNOWN_TARGET)
   {
      // one of the target disappear, clear it out
      for (auto &outcome : this->class_outcome)
      {
         if (outcome.targetId == resetTarget)
         {
            // found the target
            if (outcome.reported)
            {
               // this target should be reported already
               this->custom_spatial_static_value_sensor_->publish_state(outcome.targetId);
               this->custom_spatial_motion_value_sensor_->publish_state(0);
               ESP_LOGD(TAG, "Loop: remove reported status for targetId=%d", outcome.targetId);

               outcome.reported = false;
               outcome.sum = 0;
               memset(outcome.isHuman, 0, sizeof(outcome.isHuman));

               if(outcome.targetInZone != NOT_IN_A_ZONE)
               {
                  this->reported_human_number[outcome.targetInZone]--;
                  this->report_human_in_zone(this->reported_human_number[outcome.targetInZone], outcome.targetInZone);

                  outcome.targetInZone = NOT_IN_A_ZONE;
               }
               break;
            }
         }
      }
      resetTarget = UNKNOWN_TARGET;
   }
   ESP_LOGD(TAG, "Loop:end.");
}

bool TI6432Component::isTargetInEntryZone(trackerProc_Target &tracker)
{
   float x = tracker.posX;
   float y = tracker.posY;
   float z = tracker.posZ;

   if(  (x >= this->entryZone.startX && x <= this->entryZone.endX)
     && (y >= this->entryZone.startY && y <= this->entryZone.endY)
      // && (z >= this->entryZone.startZ && z <= this->entryZone.endZ) 
      )
   {
      return true;
   }
   return false;
}

// return: true -- entering, false -- leaving
bool TI6432Component::isTargetEntering(float y)
{
   if ( this->entryZone.endY - y < y - this->entryZone.startY)
   {
      return true;
   }
   return false;
}

// input : tracker: target coordinates
// output: return: true -- in a zone; *zoneNum is the zone number
//                 false -- not in a zone
bool TI6432Component::isTargetInZone(trackerProc_Target &tracker, uint32_t *zoneNum)
{
   float x = tracker.posX;
   float y = tracker.posY;
   float z = tracker.posZ;

   for (uint32_t i=0; i<sizeof(this->zoneBoundary)/sizeof(ZONE_BOUNDARY); i++)
   {
      if(  (x >= this->zoneBoundary[i].startX && x <= this->zoneBoundary[i].endX)
        && (y >= this->zoneBoundary[i].startY && y <= this->zoneBoundary[i].endY)
        // && (z >= this->zoneBoundary[i].startZ && z <= this->zoneBoundary[i].endZ) 
        )
      {
         *zoneNum = i;
         return true;
      }
   }
   return false;
}

bool TI6432Component::isTargetReportedAsHuman(uint32_t targetId)
{
   for(auto &oneClass : this->class_outcome)
   {
      if (oneClass.targetId == targetId)
      {
         // return oneClass.reported;
         return((oneClass.targetTracker.posZ + SENSOR_POS_Z >= 0.6) ? true : false);
      }
   }
   return false;
}

void TI6432Component::report_human_in_zone(int32_t humanNum, uint32_t zoneNum)
{
   if (zoneNum == 0)
   {
      this->custom_motion_speed_sensor_->publish_state(humanNum);
      ESP_LOGD(TAG, "reported_human_number[0]=%d", humanNum);
   }
   else if (zoneNum == 1)
   {
      this->custom_mode_num_sensor_->publish_state(humanNum);
      ESP_LOGD(TAG, "reported_human_number[1]=%d", humanNum);
   }
   else
   {
      // should not come here
   }
}

void TI6432Component::handle_frame(void)
{
   // for (auto &tlv: this->message_tlv)
   // {
   //    this->handle_tlv(tlv);
   // }
}

void TI6432Component::handle_tlv(MESSAGE_TLV &tlv)
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
         //this->handle_ext_msg_target_index(tlv.v, tlv.tl.length);
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
         ESP_LOGD(TAG, "handle_tlv: unknown TLV type:%d", tlv.tl.type);
      }
      break;
   }
}

void TI6432Component::read_big_data_from_uart(uint8_t *data, uint32_t length)
{
   if (length > 100)
   {
      // if length is >100, read 100 bytes out every time
      uint32_t readCount = 0;
      while (readCount < length)
      {
         if (readCount + 100 >= length)
         {
            this->read_array_with_delay(data+readCount, length-readCount);
            break; // read all data out
         }
         else
         {
            this->read_array_with_delay(data+readCount, 100);
            readCount += 100;
         }
      }
   }
   else
   {
      this->read_array_with_delay(data, length);
   }
}

void TI6432Component::read_array_with_delay(uint8_t *data, uint32_t length)
{
   // uint32_t start_time = millis();
   // while (this->available() < length) 
   // {
   //    if (millis() - start_time > 500) 
   //    {
   //       ESP_LOGE(TAG, "Reading from UART timed out at byte %u!", this->available());
   //       return;
   //    }
   //    yield();
   // }
   this->read_array(data, length);
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

   std::map<uint32_t, trackerProc_Target> targets_in_frame;
   for (uint32_t i=0; i<numDetectedTargets; i++)
   {
      trackerProc_Target oneTarget;
      memcpy((uint8_t *)&oneTarget, &data[i*sizeof(trackerProc_Target)], sizeof(trackerProc_Target));
      if ( (oneTarget.tid >= MAX_TARGET_NUMBER)
        || (oneTarget.posX > MAX_POS_X) || (oneTarget.posX < MIN_POS_X)
        || (oneTarget.posY > MAX_POS_Y) || (oneTarget.posY < MIN_POS_Y)
        || (oneTarget.posZ > MAX_POS_Z) || (oneTarget.posY < MIN_POS_Z)
        )
      {
         // out of range Target ID, assume wrong data, to reduce handling time.
         ESP_LOGD(TAG, "TLV target list: wrong data, targetIndex=%d, targetId=%d", i, oneTarget.tid);
         return;   
      }
      targets_in_frame.insert({oneTarget.tid, oneTarget});
   }


   std::vector<CLASSIFICATION_DATA> prev_class_outcome = this->class_outcome; //copy over the whole old class_outcome

   this->class_outcome.clear();
   for (uint32_t i=0; i<numDetectedTargets; i++)
   {
      trackerProc_Target oneTarget;
      memcpy((uint8_t *)&oneTarget, &data[i*sizeof(trackerProc_Target)], sizeof(trackerProc_Target));

      bool found = false;
      ESP_LOGD(TAG, "TLV target list: targetIndex=%d, targetId=%d", i, oneTarget.tid);      
      for (auto &outcome : prev_class_outcome)
      {
         if (outcome.targetId == oneTarget.tid)
         {
            // found existing target in class_outcome
            found = true;
            outcome.targetTracker = oneTarget;
            this->class_outcome.push_back(outcome);
            ESP_LOGD(TAG, "TLV target list: found existing targetId=%d", outcome.targetId);

            if (outcome.reported)
            {
               // this target is reported, refresh its timer
               xTimerReset(tracking_timer[outcome.timerIndex], 0);
            }
            
            // since this targetId is already found, clear it out from prev_class_outcome
            // this is for easy handle the removed targets later
            outcome.targetId = UNKNOWN_TARGET; 
            break;
         }
      }
      if (!found)
      {
         // this is a new target
         CLASSIFICATION_DATA newClass;

         newClass.targetId      = oneTarget.tid;
         newClass.targetTracker = oneTarget;
         newClass.targetInZone  = NOT_IN_A_ZONE;
         newClass.validFrameNum = 0;
         newClass.reported      = false;
         newClass.timerIndex    = 0xff;
         newClass.sum           = 0;
         memset(newClass.isHuman, 0, CLASSIFICATION_MAX_FRAMES);
         this->class_outcome.push_back(newClass);
         ESP_LOGD(TAG, "TLV target list: add new targetId=%d", newClass.targetId);
      }
   }

   for (auto &outcome : prev_class_outcome)
   {
      // if target removed in the new frame, 
      // need to check if we need to stop timer in case it was reported
      if (outcome.targetId != UNKNOWN_TARGET)
      {
         // all found targets were set to UNKNOWN_TARGET
         // so in this case, this target was not found in the new frame
         if (outcome.reported)
         {
            // this target was reported, clear out reported status
            this->custom_spatial_static_value_sensor_->publish_state(outcome.targetId);
            this->custom_spatial_motion_value_sensor_->publish_state(0);
            ESP_LOGD(TAG, "TLV target list: remove reported status for targetId=%d", outcome.targetId);  

            xTimerStop(tracking_timer[outcome.timerIndex], 0);  
            vTimerSetTimerID( tracking_timer[outcome.timerIndex], (void *)INVALID_TIMER_ID );

            if (outcome.targetInZone != NOT_IN_A_ZONE)
            {
               // target was in a zone
               this->reported_human_number[outcome.targetInZone] --;
               this->report_human_in_zone(this->reported_human_number[outcome.targetInZone], outcome.targetInZone);
            }
         }
      }
   }

   // handle entry zone counting
   // this is done after class_outcome is updated
   for (auto &one_in_frame : targets_in_frame)
   {
      if(this->entry_targets.count(one_in_frame.first) != 0)
      {
         // this target already in entry zone
         if (this->isTargetInEntryZone(one_in_frame.second))
         {
            //target remains in entry zone
            continue;
         }
         else
         {
            // target just left entry zone
            if (one_in_frame.second.posY < this->entryZone.startY)
            {
               // entered into the room
               if (this->isTargetEntering(this->entry_targets[one_in_frame.first].entryY))
               {
                  //human coming from outside, passing entry zone, now inside the room
                  if (isTargetReportedAsHuman(one_in_frame.first))
                  {
                     this->human_count_in_room ++;
                     this->human_entered_in_room_sensor_->publish_state(this->human_count_in_room);
                  }
               }
               else
               {
                  //human coming from inside room, now back to the room again
               }
            }
            else if (one_in_frame.second.posY > this->entryZone.endY)
            {
               // went out the room
               if (!this->isTargetEntering(this->entry_targets[one_in_frame.first].entryY))
               {
                  //human coming from inside room, passing entry zone, now outside the room
                  if (isTargetReportedAsHuman(one_in_frame.first))
                  {
                     this->human_count_in_room --;
                     this->human_entered_in_room_sensor_->publish_state(this->human_count_in_room);
                  }
               }
               else
               {
                  //human coming from outside room, now back to outside again
               }
            }
            this->entry_targets.erase(one_in_frame.first);
         }
      }
      else
      {
         // target was not in entry zone
         if (this->isTargetInEntryZone(one_in_frame.second))
         {
            // add new target in entry zone.
            this->entry_targets.insert({one_in_frame.first, {one_in_frame.second.posY}});
         }
      }
   }

}

void TI6432Component::handle_ext_msg_target_index(uint8_t *data, uint32_t length)
{
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
      CLASS_PROBABILITY     prob;
      CLASSIFICATION_DATA   * pClassData = &(this->class_outcome[i]);

      prob.humanProb    = (float)data[i*2+1] / 128;
      prob.nonHumanProb = (float)data[i*2]   / 128;
      
      // when target is higher than 1.4 meter, treat it as a human

      ESP_LOGD(TAG, "TLV classifier info: target pos X=%f", pClassData->targetTracker.posX);
      ESP_LOGD(TAG, "TLV classifier info: target pos Y=%f", pClassData->targetTracker.posY);
      ESP_LOGD(TAG, "TLV classifier info: target pos Z=%f", pClassData->targetTracker.posZ);
      // if (prob.humanProb == 0.5 && (pClassData->targetTracker.posZ+SENSOR_POS_Z) >= 0.6)
      // {
      //    prob.humanProb    = CLASSIFIER_CONFIDENCE_SCORE;
      //    prob.nonHumanProb = 1 - CLASSIFIER_CONFIDENCE_SCORE;
      // }

      if (prob.humanProb != 0.5)
      {
         uint8_t isHumanIndex = pClassData->validFrameNum;
         if (prob.humanProb >= CLASSIFIER_CONFIDENCE_SCORE)
         {
            // human detected
            ESP_LOGD(TAG, "TLV classifier info: targetIndex=%d, human=%f, nonHuman=%f", i, prob.humanProb, prob.nonHumanProb);
            if (isHumanIndex < CLASSIFICATION_MAX_FRAMES)
            {
               pClassData->isHuman[isHumanIndex] = 1;
               pClassData->validFrameNum ++;
            }
            else
            {
               //valid frame is full of class_outcome, remove the oldest one.
               for (uint8_t frameNum=0; frameNum<CLASSIFICATION_MAX_FRAMES-1; frameNum++ )
               {
                  pClassData->isHuman[frameNum] = pClassData->isHuman[frameNum+1];
               }
               pClassData->isHuman[CLASSIFICATION_MAX_FRAMES-1] = 1;
            }

            if (pClassData->validFrameNum >= CLASSIFICATION_MAX_FRAMES)
            {
               int8_t sum = 0;
               for (uint8_t frameNum=0; frameNum<CLASSIFICATION_MAX_FRAMES; frameNum++)
               {
                  sum += pClassData->isHuman[frameNum];
               }
               if (sum > 0)
               {
                  uint32_t inZone;

                  //overall, human detected, report
                  this->custom_spatial_static_value_sensor_->publish_state(pClassData->targetId);
                  this->custom_spatial_motion_value_sensor_->publish_state(sum);
                  ESP_LOGD(TAG, "TLV classifier info: human detected. targetId=%d, sum=%d", pClassData->targetId, sum);
                  pClassData->sum = sum;

                  if (!pClassData->reported)
                  {
                     // first time to report this target
                     // start timer to monitor disappear
                     uint8_t timerIndex;
                     if(findAvailableTimerIndex(&timerIndex))
                     {
                        xTimerStart(tracking_timer[timerIndex], 0);
                        pClassData->timerIndex = timerIndex;
                        vTimerSetTimerID( tracking_timer[timerIndex], (void *)(pClassData->targetId)); 
                     }
                     pClassData->reported = true;

                     if(this->isTargetInZone(pClassData->targetTracker, &inZone))
                     {
                        pClassData->targetInZone = inZone;
                        this->reported_human_number[inZone] ++;
                        this->report_human_in_zone(this->reported_human_number[inZone], inZone);
                     }
                     else
                     {
                        pClassData->targetInZone = NOT_IN_A_ZONE;
                     }
                  }
                  else
                  {
                     // target was reported as human
                     // check if inZone changed
                     bool isInZone = this->isTargetInZone(pClassData->targetTracker, &inZone);
                     if (pClassData->targetInZone == NOT_IN_A_ZONE)
                     {
                        // was not in a zone, 
                        if (!isInZone)
                        {
                           // and is not in a zone, NO change
                           continue;
                        }
                        else
                        {
                           // newly come into a zone
                           this->reported_human_number[inZone] ++;
                           this->report_human_in_zone(this->reported_human_number[inZone], inZone);
                           pClassData->targetInZone = inZone;
                        }
                     }
                     else
                     {
                        // was in a zone
                        if (!isInZone)
                        {
                           // not in a zone now
                           this->reported_human_number[pClassData->targetInZone] --;
                           this->report_human_in_zone(this->reported_human_number[pClassData->targetInZone], pClassData->targetInZone);
                           pClassData->targetInZone = NOT_IN_A_ZONE;
                        }
                        else
                        {
                           if (pClassData->targetInZone == inZone)
                           {
                              // zone number no change
                              continue;
                           }
                           else
                           {
                              this->reported_human_number[pClassData->targetInZone] --;
                              this->report_human_in_zone(this->reported_human_number[pClassData->targetInZone], pClassData->targetInZone);

                              this->reported_human_number[inZone] ++;
                              this->report_human_in_zone(this->reported_human_number[inZone], inZone);
                              pClassData->targetInZone = inZone;
                           }
                        }
                     }
                  }
               }
            }
         }
         else if (prob.nonHumanProb >= CLASSIFIER_CONFIDENCE_SCORE)
         {
            // non-human detected
            if (isHumanIndex < CLASSIFICATION_MAX_FRAMES)
            {
               pClassData->isHuman[isHumanIndex] = -1;
               pClassData->validFrameNum ++;
            }
            else
            {
               //valid frame is full of class_outcome, remove the oldest one.
               for (uint8_t frameNum=0; frameNum<CLASSIFICATION_MAX_FRAMES-1; frameNum++ )
               {
                  pClassData->isHuman[frameNum] = pClassData->isHuman[frameNum+1];
               }
               pClassData->isHuman[CLASSIFICATION_MAX_FRAMES-1] = -1;
            }

            if (pClassData->validFrameNum >= CLASSIFICATION_MAX_FRAMES)
            {
               int8_t sum = 0;
               for (uint8_t frameNum=0; frameNum<CLASSIFICATION_MAX_FRAMES; frameNum++)
               {
                  sum += pClassData->isHuman[frameNum];
               }
               if (sum < 0)
               {
                  // non-human detected
                  if (pClassData->reported)
                  {
                     // this target WAS reported as human, but now it changes to non-human
                     this->custom_spatial_static_value_sensor_->publish_state(pClassData->targetId);
                     this->custom_spatial_motion_value_sensor_->publish_state(sum);
                     ESP_LOGD(TAG, "TLV classifier info: targetId=%d change from human to non-human, sum=%d", pClassData->targetId, sum);  
                     pClassData->sum = sum;

                     xTimerStop(tracking_timer[pClassData->timerIndex], 0);  
                     vTimerSetTimerID( tracking_timer[pClassData->timerIndex], (void *)INVALID_TIMER_ID );     
                     pClassData->reported = false;
                  
                     if (pClassData->targetInZone != NOT_IN_A_ZONE)
                     {
                        this->reported_human_number[pClassData->targetInZone] --;
                        this->report_human_in_zone(this->reported_human_number[pClassData->targetInZone], pClassData->targetInZone);
                        pClassData->targetInZone = NOT_IN_A_ZONE;
                     }
                  }
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

// Sending data frames
void TI6432Component::send_query_(uint8_t *query, size_t string_length) 
{ 
//   this->write_array(query, string_length); 
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
}

void TI6432Component::set_scene_mode(uint8_t value) {
}

void TI6432Component::set_sensitivity(uint8_t value) {
}

void TI6432Component::set_restart() {
}

void TI6432Component::set_unman_time(uint8_t value) {
}

void TI6432Component::set_custom_mode(uint8_t mode) {
}

void TI6432Component::set_custom_end_mode() {
}

void TI6432Component::set_existence_boundary(uint8_t value) {
}

void TI6432Component::set_motion_boundary(uint8_t value) {
}

void TI6432Component::set_existence_threshold(int value) {
}

void TI6432Component::set_motion_threshold(int value) {
}

void TI6432Component::set_motion_trigger_time(int value) {
}

void TI6432Component::set_motion_to_rest_time(int value) {
}

void TI6432Component::set_custom_unman_time(int value) {
}
}  // namespace ti6432
}  // namespace esphome
