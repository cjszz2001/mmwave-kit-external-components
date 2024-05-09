// this file is copied from TI radar toolbox, industrial virualizer, gui_common.py

#ifndef GUI_COMMON_H
#define GUI_COMMON_H


// # x843 Demo Names
// DEMO_NAME_OOB = 'SDK Out of Box Demo'
// DEMO_NAME_3DPC = '3D People Tracking'
// DEMO_NAME_VITALS = 'Vital Signs with People Tracking'
// DEMO_NAME_LRPD = 'Long Range People Detection'
// DEMO_NAME_MT = 'Mobile Tracker'
// DEMO_NAME_SOD = 'Small Obstacle Detection'
// DEMO_NAME_GESTURE = 'Gesture with Machine Learning'
// DEMO_NAME_SURFACE = 'Surface Classification with Machine Learning'

// #x432 demo names
// DEMO_NAME_x432_OOB = 'IWRL6432 Out of Box Demo'
// DEMO_NAME_x432_GESTURE = 'IWRLx432 Gesture Recognition'
// DEMO_NAME_x432_SURFACE = 'IWRL6432 Surface Classification Machine Learning'
// DEMO_NAME_x432_LEVEL_SENSING = 'IWRL6432 Level Sensing'
// DEMO_NAME_x432_SOD = 'IWRL6432 Small Obstacle Detection'
// DEMO_NAME_x432_GROUND_SPEED = 'IWRLx432 True Ground Speed'
// DEMO_NAME_x432_VITAL_SIGNS = 'IWRL6432 Vital Signs'

// #1432 demo names
// DEMO_NAME_xWRL1432_OOB = 'IWRL1432 Out of Box Demo'
// DEMO_NAME_xWRL1432_LEVEL_SENSING = 'IWRL1432 Level Sensing'

// # Hold array of all the demos to check against in previous file
// x843_DEMO_TYPES = [DEMO_NAME_OOB, DEMO_NAME_3DPC, DEMO_NAME_VITALS, DEMO_NAME_MT, DEMO_NAME_LRPD, DEMO_NAME_SOD, DEMO_NAME_GESTURE, DEMO_NAME_SURFACE]
// x432_DEMO_TYPES = [DEMO_NAME_x432_OOB, DEMO_NAME_x432_GESTURE, DEMO_NAME_x432_SURFACE, DEMO_NAME_x432_LEVEL_SENSING, DEMO_NAME_x432_SOD, DEMO_NAME_x432_GROUND_SPEED, DEMO_NAME_x432_VITAL_SIGNS]
// xWRL1432_DEMO_TYPES = [DEMO_NAME_xWRL1432_OOB, DEMO_NAME_xWRL1432_LEVEL_SENSING, DEMO_NAME_x432_GESTURE, DEMO_NAME_x432_GROUND_SPEED]

// DEVICE_LIST = ["IWR6843", "IWR1843", "IWRL6432", "IWRL1432"]

// # Different methods to color the points 
// COLOR_MODE_SNR = 'SNR'
// COLOR_MODE_HEIGHT = 'Height'
// COLOR_MODE_DOPPLER = 'Doppler'
// COLOR_MODE_TRACK = 'Associated Track'


// # Com Port names
// CLI_XDS_SERIAL_PORT_NAME = 'XDS110 Class Application/User UART'
// DATA_XDS_SERIAL_PORT_NAME = 'XDS110 Class Auxiliary Data Port'
// CLI_SIL_SERIAL_PORT_NAME = 'Enhanced COM Port'
// DATA_SIL_SERIAL_PORT_NAME = 'Standard COM Port'


// UART_MAGIC_WORD = bytearray(b'\x02\x01\x04\x03\x06\x05\x08\x07')

// # Configurables
// MAX_POINTS = 1000
// MAX_PERSISTENT_FRAMES = 30

// # Classifier Configurables
// MAX_NUM_TRACKS = 20 # This could vary depending on the configuration file. Use 20 here as a safe likely maximum to ensure there's enough memory for the classifier

// # Vitals Configurables
// MAX_VITALS_PATIENTS = 2
// NUM_FRAMES_PER_VITALS_PACKET = 15
// NUM_VITALS_FRAMES_IN_PLOT = 150
// NUM_HEART_RATES_FOR_MEDIAN = 10
// NUM_VITALS_FRAMES_IN_PLOT_IWRL6432 = 15

// # Classifier Configurables
static const uint8_t NUM_CLASSES_IN_CLASSIFIER = 2;
static const uint8_t CLASSIFIER_BUFFER_LEN = 30;
static const float CLASSIFIER_CONFIDENCE_SCORE = 0.6;
static const float MIN_CLASSIFICATION_VELOCITY = 0.3;
static const uint8_t TAG_HISTORY_LEN = 5;;
static const uint8_t MAX_NUM_UNKNOWN_TAGS_FOR_HUMAN_DETECTION = 1;

// GESTURE_FEATURE_LENGTH = 15

// # 6843 9 Gesture Demo - Gestures
// GESTURE_NO_GESTURE_6843     = 0
// GESTURE_L2R_6843            = 1  
// GESTURE_R2L_6843            = 2  
// GESTURE_U2D_6843            = 3   
// GESTURE_D2U_6843            = 4 
// GESTURE_CW_TWIRL_6843       = 5   
// GESTURE_CCW_TWIRL_6843      = 6   
// GESTURE_OFF_6843            = 7     
// GESTURE_ON_6843             = 8    
// GESTURE_SHINE_6843          = 9  

// # 6432 6 Gesture Demo - Gestures
// GESTURE_NO_GESTURE_6432     = 0
// GESTURE_L2R_6432            = 1  
// GESTURE_R2L_6432            = 2  
// GESTURE_U2D_6432            = 3   
// GESTURE_D2U_6432            = 4
// GESTURE_PUSH_6432           = 5     
// GESTURE_PULL_6432           = 6

// # x6432 KTO/Gesture - Presence/Gesture Mode
// # GESTURE_PRESENCE_MODE_x432  = 0
// # GESTURE_GESTURE_MODE_x432   = 1

// # Magic Numbers for Target Index TLV
// TRACK_INDEX_WEAK_SNR = 253 # Point not associated, SNR too weak
// TRACK_INDEX_BOUNDS = 254 # Point not associated, located outside boundary of interest
// TRACK_INDEX_NOISE = 255 # Point not associated, considered as noise

// # Defined TLV's
static const uint32_t MMWDEMO_OUTPUT_MSG_DETECTED_POINTS                      = 1;
static const uint32_t MMWDEMO_OUTPUT_MSG_RANGE_PROFILE                        = 2;
static const uint32_t MMWDEMO_OUTPUT_MSG_NOISE_PROFILE                        = 3;
static const uint32_t MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP               = 4;
static const uint32_t MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP               = 5;
static const uint32_t MMWDEMO_OUTPUT_MSG_STATS                                = 6;
static const uint32_t MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO            = 7;
static const uint32_t MMWDEMO_OUTPUT_MSG_AZIMUT_ELEVATION_STATIC_HEAT_MAP     = 8;
static const uint32_t MMWDEMO_OUTPUT_MSG_TEMPERATURE_STATS                    = 9;

// # IWRL6432 Out-of-box demo packets
static const uint32_t MMWDEMO_OUTPUT_EXT_MSG_DETECTED_POINTS                  = 301;
static const uint32_t MMWDEMO_OUTPUT_EXT_MSG_RANGE_PROFILE_MAJOR              = 302;
static const uint32_t MMWDEMO_OUTPUT_EXT_MSG_RANGE_PROFILE_MINOR              = 303;
static const uint32_t MMWDEMO_OUTPUT_EXT_MSG_RANGE_AZIMUT_HEAT_MAP_MAJOR      = 304;
static const uint32_t MMWDEMO_OUTPUT_EXT_MSG_RANGE_AZIMUT_HEAT_MAP_MINOR      = 305;
static const uint32_t MMWDEMO_OUTPUT_MSG_EXT_STATS                            = 306;
static const uint32_t MMWDEMO_OUTPUT_EXT_MSG_PRESENCE_INFO                    = 307;
static const uint32_t MMWDEMO_OUTPUT_EXT_MSG_TARGET_LIST                      = 308;
static const uint32_t MMWDEMO_OUTPUT_EXT_MSG_TARGET_INDEX                     = 309;
static const uint32_t MMWDEMO_OUTPUT_EXT_MSG_MICRO_DOPPLER_RAW_DATA           = 310;
static const uint32_t MMWDEMO_OUTPUT_EXT_MSG_MICRO_DOPPLER_FEATURES           = 311;
static const uint32_t MMWDEMO_OUTPUT_EXT_MSG_RADAR_CUBE_MAJOR                 = 312;
static const uint32_t MMWDEMO_OUTPUT_EXT_MSG_RADAR_CUBE_MINOR                 = 313;
static const uint32_t MMWDEMO_OUTPUT_EXT_MSG_POINT_CLOUD_INDICES              = 314;
static const uint32_t MMWDEMO_OUTPUT_EXT_MSG_ENHANCED_PRESENCE_INDICATION     = 315;
static const uint32_t MMWDEMO_OUTPUT_EXT_MSG_ADC_SAMPLES                      = 316;
static const uint32_t MMWDEMO_OUTPUT_EXT_MSG_CLASSIFIER_INFO                  = 317;
static const uint32_t MMWDEMO_OUTPUT_EXT_MSG_RX_CHAN_COMPENSATION_INFO        = 318;


static const uint32_t MMWDEMO_OUTPUT_MSG_GESTURE_FEATURES_6432               = 350;
static const uint32_t MMWDEMO_OUTPUT_MSG_GESTURE_CLASSIFIER_6432             = 351;
static const uint32_t MMWDEMO_OUTPUT_MSG_GESTURE_PRESENCE_x432               = 352;
static const uint32_t MMWDEMO_OUTPUT_MSG_GESTURE_PRESENCE_THRESH_x432        = 353;
static const uint32_t MMWDEMO_OUTPUT_MSG_GESTURE_CLASSIFIER_PROB_6432        = 354;

static const uint32_t MMWDEMO_OUTPUT_MSG_SPHERICAL_POINTS                     = 1000;
static const uint32_t MMWDEMO_OUTPUT_MSG_TRACKERPROC_3D_TARGET_LIST           = 1010;
static const uint32_t MMWDEMO_OUTPUT_MSG_TRACKERPROC_TARGET_INDEX             = 1011;
static const uint32_t MMWDEMO_OUTPUT_MSG_TRACKERPROC_TARGET_HEIGHT            = 1012;
static const uint32_t MMWDEMO_OUTPUT_MSG_COMPRESSED_POINTS                    = 1020;
static const uint32_t MMWDEMO_OUTPUT_MSG_PRESCENCE_INDICATION                 = 1021;
static const uint32_t MMWDEMO_OUTPUT_MSG_OCCUPANCY_STATE_MACHINE              = 1030;
static const uint32_t MMWDEMO_OUTPUT_MSG_SURFACE_CLASSIFICATION               = 1031;
static const uint32_t MMWDEMO_OUTPUT_EXT_MSG_VELOCITY                         = 1033;

static const uint32_t MMWDEMO_OUTPUT_MSG_VITALSIGNS                           = 1040;

static const uint32_t MMWDEMO_OUTPUT_MSG_GESTURE_FEATURES_6843                = 1050;
static const uint32_t MMWDEMO_OUTPUT_MSG_GESTURE_OUTPUT_PROB_6843             = 1051;

/*
# Expected minimums and maximums to bound the range of colors used for coloring points
SNR_EXPECTED_MIN = 5
SNR_EXPECTED_MAX = 40
SNR_EXPECTED_RANGE = SNR_EXPECTED_MAX - SNR_EXPECTED_MIN
DOPPLER_EXPECTED_MIN = -30
DOPPLER_EXPECTED_MAX = 30
DOPPLER_EXPECTED_RANGE = DOPPLER_EXPECTED_MAX - DOPPLER_EXPECTED_MIN
*/

#endif // GUI_COMMON_H