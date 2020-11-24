//
// Created by jvaccaro on 4/11/19.
//
/**
* Copyright 2019 Woods Hole Oceanographic Institution
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

/* Defined by J Vaccaro */
#define EM_DGM_I_PU                        "#IPU"
#define EM_DGM_I_PI                        "#IPI"
#define EM_DGM_I_BIST_ERROR                "#IBE" // (BIST Error report),
#define EM_DGM_I_BIST_REPLY                "#IBR" // (BIST reply) or
#define EM_DGM_I_BIST_SHORT                "#IBS" // (BIST short reply).

#include <string>
#include <vector>
#include <fstream>

namespace kongsberg_em{

// 813, 812, 455, 12, 1, 616,

enum RET_CODE{
  SUCCESS = 0,
  WRONG_SOUNDER_NAME,
  BAD_CODE
};

enum K_TO_SIS{
  STATUS_IPI = 12,
  READY = 39,
  // PING AND SETTINGS
  STATUS_IPU = 616,
  // PU PARAMS
  FILEIMPORTED = 476, // 451,476
  PARAMSREAD = 459,
  // VALUES
  VALUES = 401,
  // Other, not documented so these are guesses/unknown?
  KCTRL_VERSION=1,
  STATUS_IPI_PU = 812, // #IPI
  STATUS_INPUTS = 813, // # IPU
  XML = 814,
  UNKNOWN_455 = 455, // DATA STREAM PARAMS
  BIST_RESULT = 817,
  PU_DISCONNECTED = 998,
} ;

// IPI LIST

// IPU LIST

// PARAMS LIST

// VALUES LIST

enum SIS_TO_K{
  START = 13,
  SET_READY = 451,
  // PING
  START_PING = 458,
  STOP_PING = 457,
  LOG_IOP_SVP = 454,
  // SETTINGS
  WC_OFF = 460,
  WC_ON = 461,
  STAVEGEN_OFF = 462,
  STAVEGEN_ON = 463,
  SCOPEDATA_OFF = 464,
  SCOPEDATA_ON = 465,
  PIPETRACK_OFF = 466,
  PIPETRACK_ON = 467,
  // PU PARAMS
  IMPORTFILE = 473, // 450,473
  READPARAMS = 476, // 451,476
  EXPORTPARAM = 472, // 450,472
  // RUNTIME VALUES
  GETVALUES = 261,
  SETVALUES = 262,
  TERMINATE = 24
} ;


// Source: Interface specification K-controller and SIS
// DETECTION OF SOUNDERS
// --> $KSSIS,12,NAME=xyz~~IP=xyz....$$$NAME=xyz... // Each sounder detected will be separated with «$$$», and each
                                                    // token/value pair is separated with «~~»
// <-- $KSSIS,13,SOUNDER_NAME  // A sounder is selected for start.
// --> $KSSIS,39,SOUNDER_NAME  // Sounder is ready for use, see also <$KSSIS,616,...> for more information on status
// <-- $KSSIS,451,SOUNDER_NAME // Operator selects a sounder, as active in case several has been detected.

// CHECK THE RESULTS OF ANY CHANGES
// --> $KSSIS,616,SOUNDER_NAME,$IPU:...$$$) // #IPU is received from PU every sec. and contains pinging status etc.

// PINGING
// <-- $KSSIS,458,SOUNDER_NAME // Start pinging
// <-- $KSSIS,457,SOUNDER_NAME // Stop pinging
// <-- $KSSIS,457,SOUNDER_NAME  // Stop pinging
// LOGGING
// <-- $KSSIS,454,SOUNDER_NAME  // When starting new logged files (i.e. lines) the KM standard is to begin each file
                                // (.kmall) with the #IIP #IOP #SVP datagrams. To trigger the PU to send these datagrams
                                // in the datastream, KSSIS_454 is sent to the K-Controller
// WATERCOLUMN
// <-- $KSSIS,460,SOUNDER_NAME // Turn generation of watercolumn data for logging and/or display off in PU.
// <-- $KSSIS,461,SOUNDER_NAME // Turn generation of watercolumn data for logging and/or display on in PU.
// STAVE DATA
// <-- $KSSIS,462,SOUNDER_NAME // Stave data generation off
// <-- $KSSIS,463,SOUNDER_NAME // Stave data generation on
// SCOPE DATA
// <-- $KSSIS,464,SOUNDER_NAME // Scope data off, for all beams
// <-- $KSSIS,465,SOUNDER_NAME,BEAM_NO=### // Scope data on, for specific Beam
// PIPE TRACKER
// <-- $KSSIS,466,SOUNDER_NAME // Pipe tracker, off
// <-- $KSSIS,467,SOUNDER_NAME,PIPE_TRACKER_DATA // Pipe tracker, on, with optional data

// PARAMETERS
// Import PU param from file : Changes to apply
// <-- $KSSIS,450,473,file name, // Specify file to import
// --> $KSSIS,451,476,SOUNDER_NAME // Open file and retrieve contained sounder ID
// <-- $KSSIS,450,476,SOUNDER_NAME // Use retrieved ID or set new (existing or dumm, -operator selection in SIS)
// --> $KSSIS,459,SOUNDER_NAME,<#IPI datagram>$$$, ...) // Read parameters from file and update for selected sounder id.
                                                        // If this is a dummy sounder, instantiate it first (see comment on SIS side).
// This may be a dummy sounder which must be instantiated. (Dummy sounders are sounders not detected on the network,
// but only used for pre-config and test purposes. As such they can not be started.)
// <-- $KSSIS,450,472,file name,Comments) // PU parameters for active selected sounder is stored together with set
                                          // comments in file with set name.

// RUNTIME VALUES
// <-- $KSSIS,261,SOUNDER_NAME // Request values
// Previous set values are gathered from DB
// --> $KSSIS,401,SOUNDER_NAME,JSON_GUI_DATA // Values returned
// <-- $KSSIS,262,SOUNDER_NAME,TOKEN=NEW_VAL, ... // Set one or more particular values
// K-Controller sets value in PU
// --> $KSSIS,401,SOUNDER_NAME,JSON_GUI_DATA // K-Controller confirms new value

// TERMINATE K CONTROLLER
// <-- $KSSIS,24  // Send to Terminate K-Controller (end process)

// PING INFO DATAGRAM ENUMS
enum beamSpacing {
  EQUIDISTANCE = 0,
  EQUIANGLE,
  HIGH_DENSITY
};

enum depthMode {
  VERY_SHALLOW = 0,
  SHALLOW,
  MEDIUM,
  DEEP,
  DEEPER,
  VERY_DEEP,
  EXTRA_DEEP,
  EXTREME_DEEP,
  VERY_SHALLOW_MAN = 100,
  SHALLOW_MAN,
  MEDIUM_MAN,
  DEEP_MAN,
  DEEPER_MAN,
  VERY_DEEP_MAN,
  EXTRA_DEEP_MAN,
  EXTREME_DEEP_MAN
};

enum detectionMode {
  NORMAL = 0,
  WATERWAY,
  TRACKING,
  MIN_DEPTH,
  NORMAL_SIM = 100,
  WATERWAY_SIM,
  TRACKING_SIM,
  MIN_DEPTH_SIM
};

enum modeAndStabilisation {
  PITCH_STAB = 1,
  YAW_STAB = 2,
  SONAR_MODE = 4,
  ANGULAR_COVERAGE_MODE = 8,
  SECTOR_MODE = 16,
  SWATH_DYNAMIC_POS = 32
};

enum runtimeFilter1 {
  SLOPE = 1,
  AERATION = 2,
  SECTOR = 4,
  INTERFERENCE = 8,
  SPECIAL_AMPLITUDE = 16
};

enum runtimeFilter2 {
  RANGE_GATE_SMALL = 1,
  RANGE_GATE_NORMAL = 2,
  RANGE_GATE_LARGE = 4,
  SPIKE_FILTER_OFF = 16,
  SPIKE_FILTER_WEAK = 32,
  SPIKE_FILTER_MEDIUM = 64,
  SPIKE_FILTER_STRONG = 128,
  PEN_FILTER_OFF = 256,
  PEN_FILTER_WEAK = 512,
  PEN_FILTER_MEDIUM = 1024,
  PEN_FILTER_STRONG = 2048,
  PHASE_RAMP_SHORT = 4096,
  PHASE_RAMP_NORMAL = 8192,
  PHASE_RAMP_LONG = 16384
};

enum latLongInfo {
  LAST_POSITION = 0,
  INTERPOLATED,
  PROCESSED
};

enum generalStatus {
  VALID = 0,
  INVALID,
  REDUCED_PERFORMANCE
};

// SOUNDING STRUCT ENUMS
enum detectionMethod{
  NO_VALID_DETECT = 0,
  AMPLITUDE_DETECT,
  PHASE_DETECT
};

struct runtime_params{
//  std::string SIM_ON = "$KSSIS,262,EM2040_40,RUNT_PARAM_SIM_ON_OFF=OK";
//  std::string SIM_OFF = "$KSSIS,262,EM2040_40,RUNT_PARAM_SIM_ON_OFF=";
//  std::string MCAST_ADDR = "$KSSIS,262,EM2040_40,INST_PARAM_PU_OUTPUT_MCAST_A=240.237.20.40";
//  std::string MCAST_ADDR_AGAIN = "$KSSIS,455,EM2040_40,234.255.255.255,6020";
//  std::string SIM_SLANT_ACROSS = "RUNT_PARAM_SIM_SLANT_ACROSS=0.00";
//  std::string SIM_STEP_ALONG = "RUNT_PARAM_SIM_STEP_ALONG=50";
//  std::string SIM_MIN_DEPTH = "RUNT_PARAM_SIM_MIN_DEPTH=50";
//  std::string SIM_ON_OFF = "RUNT_PARAM_SIM_ON_OFF=1";
//  std::string PU_BEAMNO = "RUNT_PARAM_PU_BEAMNO=0";
//  std::string PU_WATERCOL = "RUNT_PARAM_PU_WATERCOL=1";
  // SECTOR COVERAGE
  std::string PU_ANGLE_PORT_RX1 = "RUNT_PARAM_PU_ANGLE_PORT_RX1=70"; //int
  std::string PU_ANGLE_STARB_RX1 = "RUNT_PARAM_PU_ANGLE_STARB_RX1=70"; //int
  std::string PU_COVERAGE_PORT = "RUNT_PARAM_PU_COVERAGE_PORT=200"; //int
  std::string PU_COVERAGE_STARB = "RUNT_PARAM_PU_COVERAGE_STARB=200"; //int
  std::string ANGULAR_COVERAGE = "RUNT_PARAM_PU_ANGULAR_COVERAGE_MODE=Manual"; // "RUNT_PARAM_PU_ANGULAR_COVERAGE_MODE=Auto"
  std::string SECTOR_MODE = "RUNT_PARAM_PU_SECTOR_MODE=Single sector centre"; // "RUNT_PARAM_PU_SECTOR_MODE=Normal" "RUNT_PARAM_PU_SECTOR_MODE=Single sector starboard" "RUNT_PARAM_PU_SECTOR_MODE=Single sector port"
  std::string BEAM_SPACING = "RUNT_PARAM_PU_BEAM_SPACING=Equidistance"; // "Equiangle" "High density"
  // DEPTH SETTINGS
  std::string FORCE_DEPTH = "RUNT_PARAM_PU_FORCE_DEPTH=1.0"; //float
  std::string MIN_DEPTH = "RUNT_PARAM_PU_MIN_DEPTH=1"; //int
  std::string MAX_DEPTH = "RUNT_PARAM_PU_MAX_DEPTH=150"; //int
  std::string FREQUENCY = "RUNT_PARAM_PU_MAX_PING_FREQ=200kHz"; // 300kHz 400kHz
  std::string DEPTH_MODE = "RUNT_PARAM_PU_PULSE_TYPE1=Auto"; // Shallow Medium Deep "Very deep"
  std::string DETECTOR_MODE = "RUNT_PARAM_PU_DETECTOR_MODE=Normal"; // Waterway Tracking "Min. depth"
  std::string FM_DISABLE = "RUNT_PARAM_PU_FM_PULSE=1"; //bool
  std::string WATERCOL = "RUNT_PARAM_PU_WATERCOL=1"; //bool
  std::string EXTRA_DET = "RUNT_PARAM_PU_EXTRA_DET=1"; //bool
  // TRANSMIT CONTROL
  std::string PITCH_STAB = "RUNT_PARAM_PU_PITCH_STAB=1"; //bool
  std::string TXMIT_ANGLE_ALONG = "RUNT_PARAM_PU_TX_ANGLE=1.00"; //float
  std::string YAW_STAB = "RUNT_PARAM_PU_SEL_STAB_MODE=Rel. mean heading"; // Off Manual
  std::string HEADING_FILTER = "RUNT_PARAM_PU_HEAD_FILTER=Weak"; //Medium Hard
  std::string MAX_PING_RATE = "RUNT_PARAM_PU_PING_MODE=55.00"; //float
  std::string MIN_SWATH_DISTANCE = "RUNT_PARAM_PU_MIN_SWATH_DIST=1"; //int
  std::string EXTERNAL_TRIGGER = "RUNT_PARAM_PU_EXT_TRIG=1"; //bool
  // FILTERS
  std::string SPIKE_FILTER = "RUNT_PARAM_PU_SPIKE_FILTER_STRENGTH=Off"; // Weak Medium Strong
  std::string RANGE_GATE = "RUNT_PARAM_PU_RANGE_GATE=Small"; // Normal Large
  std::string PHASE_RAMP = "RUNT_PARAM_PU_PHASE_RAMP=Short"; // Normal Long
  std::string PEN_FILTER_STRENGTH = "RUNT_PARAM_PU_PENETRATION_FILTER_STRENGTH=Weak"; // Off Medium Strong
  std::string SPECIAL_TVG = "RUNT_PARAM_PU_SPECIAL_TVG=1"; //bool
  std::string SLOPE_FILTER = "RUNT_PARAM_PU_SLOPE=1"; //bool
  std::string AERATION_FILTER = "RUNT_PARAM_PU_AERATION=1";//bool
  std::string INTERFERENCE_FILTER = "RUNT_PARAM_PU_INTERFERENCE=1"; //bool
  std::string SPECIAL_AMP_DETECT = "RUNT_PARAM_PU_SPECIAL_AMP_DETECT=0"; //bool
  std::string NORMAL_INCIDENCE_CORR = "RUNT_PARAM_PU_NORM_INCIDENCE=11.0";//float
  std::string USE_LAMBERTS_LAW = "RUNT_PARAM_PU_USE_LAMBERTS=0"; //bool
  std::string TXMIT_POWER_LEVEL = "RUNT_PARAM_PU_TXP=Normal"; //"-10 dB" "-20 dB"
  std::string WATERCOL_X_LOG = "RUNT_PARAM_PU_WC_XRLOGR=40.0";//float
  std::string WATERCOL_TVG_OFFSET_DB = "RUNT_PARAM_PU_WC_DBOFFSET=20";//int
  std::string ADD_PHASE_DATA = "RUNT_PARAM_PU_WC_PHASE=Low resolution"; // Off "High resolution"
  std::string SONAR_MODE = "RUNT_PARAM_PU_SPECIAL_MODE_SONAR=Sonar active"; //Off "Sonar passive"
  std::string ENABLE_SCOPE = "RUNT_PARAM_PU_SCOPE=1"; //bool
  std::string SWATH_NO = "RUNT_PARAM_PU_SWATHNO=1"; //int
  std::string SCOPE_BEAM_NO = "RUNT_PARAM_PU_SCOPE=1"; //int
  // SIMULATOR
  std::string ENABLE_SIM = "RUNT_PARAM_SIM_ON_OFF=1"; //bool
  std::string SIM_MIN_DEPTH = "RUNT_PARAM_SIM_MIN_DEPTH=50"; //int
  std::string SIM_MAX_DEPTH = "RUNT_PARAM_SIM_MAX_DEPTH=60"; //int
  std::string SIM_SLANT_ACROSS = "RUNT_PARAM_SIM_STEP_ALONG=70.00"; //float MISLABELED IN KCTRL WEBBROWSER
  std::string SIM_STEP_ALONG = "RUNT_PARAM_SIM_SLANT_ACROSS=8.00"; //float MISLABED IN KCTRL WEBBROWSER
  //SOUND VELOCITY
  std::string SOUNDVEL_SOURCE = "RUNT_PARAM_PU_SV_SOURCE=Manual"; // Profile "Probe at HWS" "Probe at EM"
  std::string SOUNDVEL_OFFSET = "RUNT_PARAM_PU_SV_OFFSET=0.00";//float
  std::string SOUNDVEL_FILTER = "RUNT_PARAM_PU_SV_FILTER=55.00"; //float
};

struct install_params{
  // --------SENSORS--------
  // POS1
  std::string POS1_NAME = "INST_PARAM_CTRL_POS1_NAME=System Name"; //string
  std::string POS1_X = "INST_PARAM_PU_POS1_X=1.000"; //float FORWARD
  std::string POS1_Y = "INST_PARAM_PU_POS1_Y=2.000"; //float STARBOARD
  std::string POS1_Z = "INST_PARAM_PU_POS1_Z=3.000"; //float DOWNWARD
  std::string POS1_CORR = "INST_PARAM_PU_POS1_CORR=1"; //bool motion correction
  std::string POS1_DELAY = "INST_PARAM_PU_POS1_DELAY=1.000"; //float seconds
  std::string POS1_QUAL = "INST_PARAM_PU_POS1_QAUL=On"; //enum Off On MISSPELLED
  std::string POS1_TIME = "INST_PARAM_PU_POS1_TIME=System"; // Datagram
  std::string POS1_FORMAT = "INST_PARAM_PU_POS1_FORMAT=GGK"; // GGA
  std::string POS1_INPUT = "INST_PARAM_PU_POS1_INPUT=Serial port 1"; // "No" "Net port 1"
  // POS2 "INST_PARAM_CTRL_POS2_NAME=POS_SYS_2"
  // POS3 "INST_PARAM_CTRL_POS3_NAME=blah blah"
  // ATT1
  std::string ATT1_NAME = "INST_PARAM_CTRL_ATT1_NAME=Attitude Name";
  std::string ATT1_X = "INST_PARAM_PU_ATT1_X=0.00"; //float meters Forward
  std::string ATT1_Y = "INST_PARAM_PU_ATT1_Y=0.00"; //float meters Starboard
  std::string ATT1_Z = "INST_PARAM_PU_ATT1_Z=0.00"; //float meters Down
  std::string ATT1_ROLL = "INST_PARAM_PU_ATT1_R=1.00";//float roll
  std::string ATT1_PITCH = "INST_PARAM_PU_ATT1_P=2.000"; //float pitch
  std::string ATT1_HEADING = "INST_PARAM_PU_ATT1_H=3.00"; //float heading
  std::string ATT1_DELAY = "INST_PARAM_PU_ATT1_DELAY=1.00";//
  std::string ATT1_ROLL_REF = "INST_PARAM_PU_ATT1_ROLL=Horizontal"; // Rotation
  std::string ATT1_FORMAT = "INST_PARAM_PU_ATT1_FORMAT=EM Attitude"; // "KM Binary" "Seapath Binary 11" "Seapath Binary 23" "Seapath Binary 26" "POS MV GRP 102/103"
  std::string ATT1_INPUT = "INST_PARAM_PU_ATT1_INPUT=No"; // Same options as above
  // ATT2
  // SOUNDVEL
  std::string SVP_NAME = "INST_PARAM_CTRL_SVP_NAME=Sound velocity !!!";
  std::string SVP_INPUT = "INST_PARAM_PU_SVP_INPUT=Net port 5";
  std::string SVP_FORMAT = "INST_PARAM_PU_SVP_FORMAT=AML SV";
  // TIME SYSTEM
  // COM1
  std::string COM1_INTERFACE = "INST_PARAM_PU_COM1_INT=RS422";
  std::string COM1_BAUD = "INST_PARAM_PU_COM1_BAUD=1200"; //2400 4800 9600 19200 38400 115200
  std::string COM1_DATA_BIT = "INST_PARAM_PU_COM1_DATA=8"; // 7
  std::string COM1_STOP_BIT = "INST_PARAM_PU_COM1_STOP=1"; // 2
  std::string COM1_PARITY = "INST_PARAM_PU_COM1_PAR=None"; // Even Odd
  // UDP1
  std::string UDP1_NET = "INST_PARAM_PU_UDP1_NET=Second net"; // "Main net"
  std::string UDP1_PORT = "INST_PARAM_PU_UDP1_PORT=20000"; //int
  // ACTIVE INPUTS
  std::string ACTIVE_POSITION = "INST_PARAM_PU_ACTIVE_POS=Position system 1"; // "Position system 2" "Position system 3"
  std::string ACTIVE_ATTITUDE = "INST_PARAM_PU_ACTIVE_ATT=Attitude system 1"; // "Attitude system 2" "Attitude FM corr."
  // -------NETWORKING------
  std::string MULTICAST_IP = "INST_PARAM_PU_OUTPUT_MCAST_A=224.1.20.40"; //224-239.x.x.x
  std::string MULTICAST_PORT = "INST_PARAM_PU_OUTPUT_MCAST_P=6020";
  std::string SECONDARY_NET_IP = "INST_PARAM_PU_NET2_ADDR=105.105.105.100";
  std::string SECONDARY_NET_SUBNET = "INST_PARAM_PU_NET2_SUBNET=255.255.0.0";
  // -------TRANSDUCERS------
  // TX1
  std::string TX1_X = "INST_PARAM_PU_TX1_X=1.00"; // meters forward
  std::string TX1_Y = "INST_PARAM_PU_TX1_Y=2.00"; // m stbd
  std::string TX1_Z = "INST_PARAM_PU_TX1_Z=3.00"; // m downward
  std::string TX1_ROLL = "INST_PARAM_PU_TX1_R=9.00"; // deg
  std::string TX1_PITCH = "INST_PARAM_PU_TX1_P=8.00"; // deg
  std::string TX1_HEADING = "INST_PARAM_PU_TX1_H=7.00"; // deg
  std::string TX1_ROT_180 = "INST_PARAM_PU_TX1_TURN=1"; // bool
  // RX1
  std::string RX1_X = "INST_PARAM_PU_RX1_X=1.00"; // meters forward
  std::string RX1_Y = "INST_PARAM_PU_RX1_Y=2.00"; // m stbd
  std::string RX1_Z = "INST_PARAM_PU_RX1_Z=3.00"; // m downward
  std::string RX1_ROLL = "INST_PARAM_PU_RX1_R=9.00"; // deg
  std::string RX1_PITCH = "INST_PARAM_PU_RX1_P=8.00"; // deg
  std::string RX1_HEADING = "INST_PARAM_PU_RX1_H=7.00"; // deg
  std::string RX1_ROT_180 = "INST_PARAM_PU_RX1_TURN=1"; // bool
  // Other
  std::string RX1_BS_OFFSET = "INST_PARAM_PU_RX1_GAIN=3.00"; // MAYBE MISLABELED
  std::string WATERLINE_Z = "INST_PARAM_PU_WATER_Z=2.00"; // m
  // --------------BIST-----------
  std::string BIST_DO_CPU = "INST_PARAM_BIST_DO=CPU test";
  std::string BIST_DO_TX_UNIT = "INST_PARAM_BIST_DO=TX unit test";
  std::string BIST_DO_RX_CHANNELS = "INST_PARAM_BIST_DO=RX channels";
  std::string BIST_DO_RX_NOISE_SPECTRUM = "INST_PARAM_BIST_DO=RX noise spectrum";
  std::string BIST_DO_CBMF = "INST_PARAM_BIST_DO=CBMF test";
  std::string BIST_DO_CBMF_CPU = "INST_PARAM_BIST_DO=CBMF-CPU link";
  std::string BIST_DO_TX_VIA_RX = "INST_PARAM_BIST_DO=TX channels via RX";
  std::string BIST_DO_SOFTWARE_VERSIONS = "INST_PARAM_BIST_DO=Software date and versions";
  std::string BIST_DO_RX_UNIT = "INST_PARAM_BIST_DO=RX unit test";
  std::string BIST_DO_RX_CBMF = "INST_PARAM_BIST_DO=RX-CBMF link";
  std::string BIST_DO_RX_NOISE_LEVEL = "INST_PARAM_BIST_DO=RX noise level";
  std::string BIST_DO_SYSTEM_INFO = "INST_PARAM_BIST_DO=System information";
  std::string BIST_MULTI = "INST_PARAM_BIST_MULTI=1"; //bool
};

struct bist_strings_def{
  enum {
    BIST_CPU=0,
    BIST_CBMF=2,
    BIST_RX_UNIT=3,
    BIST_TX_UNIT=4,
    BIST_CBMF_CPU=5,
    BIST_RX_CBMF=6,
    BIST_RX_CHANNELS=7,
    BIST_TX_VIA_RX=8,
    BIST_RX_NOISE_LEVEL=9,
    BIST_RX_NOISE_SPECTRUM=10,
    BIST_SOFTWARE_VERSIONS=15,
    BIST_SYSTEM_INFO=16
  };
  std::string CPU = "CPU test"; //0
  std::string CBMF = "CBMF test"; //2
  std::string RX_UNIT = "RX unit test"; //3
  std::string TX_UNIT = "TX unit test"; //4
  std::string CBMF_CPU = "CBMF-CPU link"; //5
  std::string RX_CBMF = "RX-CBMF link"; //6
  std::string RX_CHANNELS = "RX channels"; //7
  std::string TX_VIA_RX = "TX channels via RX"; //8 DON'T RUN ON-DECK
  std::string RX_NOISE_LEVEL = "RX noise level"; //9
  std::string RX_NOISE_SPECTRUM = "RX noise spectrum"; //10
  std::string SOFTWARE_VERSIONS = "Software date and versions"; //15
  std::string SYSTEM_INFO = "System information"; //16
  std::vector<std::string> get_ondeck(){
    return {CPU,
            CBMF,
            RX_UNIT,
            TX_UNIT,
            CBMF_CPU,
            RX_CBMF,
            RX_CHANNELS,
            RX_NOISE_LEVEL,
            RX_NOISE_SPECTRUM,
            SOFTWARE_VERSIONS,
            SYSTEM_INFO};
  }
  std::vector<std::string> get_inwater() {
    return {CPU,
            CBMF,
            RX_UNIT,
            TX_UNIT,
            CBMF_CPU,
            RX_CBMF,
            RX_CHANNELS,
            TX_VIA_RX,
            RX_NOISE_LEVEL,
            RX_NOISE_SPECTRUM,
            SOFTWARE_VERSIONS,
            SYSTEM_INFO};
  }
  std::string get_name_from_code(int code){
    switch (code){
      case BIST_CPU : return CPU;
      case BIST_CBMF : return CBMF;
      case BIST_RX_UNIT : return RX_UNIT;
      case BIST_TX_UNIT : return TX_UNIT;
      case BIST_CBMF_CPU : return CBMF_CPU;
      case BIST_RX_CBMF : return RX_CBMF;
      case BIST_RX_CHANNELS : return RX_CHANNELS;
      case BIST_TX_VIA_RX : return TX_VIA_RX;
      case BIST_RX_NOISE_LEVEL : return RX_NOISE_LEVEL;
      case BIST_RX_NOISE_SPECTRUM : return RX_NOISE_SPECTRUM;
      case BIST_SOFTWARE_VERSIONS : return SOFTWARE_VERSIONS;
      case BIST_SYSTEM_INFO : return SYSTEM_INFO;
      default:
        return "Unknown";
    }
  }
};
typedef struct bist_strings_def bist_strings;


struct bist_result_strings {

  std::string CBMF_P = "$KSSIS,817,EM2040_130,p\\x02\\x00\\x00#IBR\\x00\\x82\\xf8\\x07\\xd4\\xd8\\x07]\\xf2y0\\x10W\\x02\\x00\\x00\\x02\\x00CBMF test -  EM 2040\\nNumber of CBMF boards detected: 1 (1 expected)\\n\\nCBMF status:\\n  - Board ID: 0\\n  - Parts list: EM2040_BMF 373006 FW 1.11 SW 15.01.22 \\n  - Product: CBMF 381169 B 720871 1646\\n  - Board reg. no.: 381169 B\\n  - Board serial no.: 720871\\n  - Firmware/software rev.: 1.11 15.01.22 \\n\\n  Temperature and DC power status:\\n  - FPGA die temperature [Celsius]:   56.0\\n  - FPGA voltages [Volt]: 0.99 (1.0), 1.79 (1.8), 1.00 (1.0)\\n  - External Regulated supply [";
  std::string CBMF_F = "$KSSIS,817,EM2040_130,\\x94\\x00\\x00\\x00#IBR\\x00\\x82\\xf8\\x07;\\xd9\\x07]\\xac\\x11\\x14\\x1cx\\x00\\x00\\x00\\x02\\xffCBMF test -  EM 2040\\nNumber of CBMF boards detected: 1 (1 expected)\\n\\nCBMF status:\\n\\n  ERROR: Module not responding\\n\\x00\\x00\\x00\\x00\\x94\\x00\\x00\\x00\n";
  unsigned char CBMF_PASS[4] = {'p', 0x02, 0x00, 0x00};
  unsigned char CBMF_FAIL[4] = {0x94, 0x00, 0x00, 0x00};
  std::string CPU_P = "$KSSIS,817,EM2040_130,t\\x01\\x00\\x00#IBR\\x00\\x82\\xf8\\x07\\xd6\\xe2\\x07]1\\xee$\\x11Z\\x01\\x00\\x00\\x00\\x00CPU test -  EM 2040\\nCPU: PP 833/x9x (SMP)\\nClock 14 MHz\\nDie   38 oC (peak: 41 oC @ 2019-06-17 - 17:59:23)\\nBoard 38 oC (peak: 40 oC @ 2019-06-17 - 17:59:23)\\nCore  0.80 V\\n3V3   3.04 V\\n12V   14.50 V\\n-12V  -14.50 V\\nPrimary network: 192.168.100.130:0xffffff00\\nSecondary network: 105.105.105.100:0xffff0000\\nSoftware version: 5.0.D 2019-05-21 BETA\\n\\x00\\x00t\\x01\\x00\\x00";
  unsigned char CPU_PASS[4] = {'t', 0x01, 0x00, 0x00};
  std::string TX_UNIT_P =  "$KSSIS,817,EM2040_130,P\\x02\\x00\\x00#IBR\\x00\\x82\\xf8\\x07F\\xe4\\x07]\\xd2\\xb6\\x8a\\x044\\x02\\x00\\x00\\x04\\x00TX unit test -  EM 2040\\nNumber of TX modules detected: 1 (1 expected)\\n\\nTX status:\\n  - Transducer serial number: 316\\n  - Board ID: 0\\n  - Parts list: EM2040_TX 337748 \\n  - Product: LPFPGA 315032 G 3818 \\n  - Board reg. no.: 337748 \\n  - Board serial no.: 759910\\n  - Firmware/software rev.: 1.07   Mar  8 2018 \\n  Temperature and DC power status:\\n  - FPGA die temperature [Celsius]:   38.3\\n  - FPGA voltages [Volt]: 0.98 (1.0), 2.49 (2.5), 3.30 (3.3)\\n  - Power supply [V\n";
  unsigned char TX_UNIT_PASS[4] = {'P', 0x02, 0x00, 0x00};
  std::string RX_CHANNELS_P = "$KSSIS,817,EM2040_130,\\x8cF\\x00\\x00#IBR\\x00\\x82\\xf8\\x07\\x91\\xe5\\x07]\\xb1Z\\x8f5rF\\x00\\x00\\x07\\x00RX channels -  EM 2040\\n\\nTest of RX channels.\\nSignal Amplitude in dB\\n\\n              200kHz              300kHz              380kHz\\nChannel       Low      High       Low      High       Low      High\\n  0           -3.2     -2.4       -2.1     -2.6        2.2     -0.3 \\n  1           -2.8     -2.2       -1.1     -2.1        2.7      0.1 \\n  2           -2.9     -2.3       -1.7     -2.1        2.1     -0.8 \\n  3           -2.6     -2.1       -2.0     -2.0        1.3 \n";
  unsigned char RX_CHANNELS_PASS[4] = {0x8c, 'F', 0x00, 0x00};
  std::string RX_NOISE_SPECTRUM_P = "$KSSIS,817,EM2040_130,\\xb0\\x07\\x00\\x00#IBR\\x00\\x82\\xf8\\x07\\xa4\\xe5\\x07]\\xe0e\\x8e\\x00\\x94\\x07\\x00\\x00\\n\\x00RX noise spectrum -  EM 2040\\nMax noise level per band:\\n   180 to 219 kHz:  52.7 dB at 186.6 kHz\\n   280 to 319 kHz:  47.2 dB at 311.6 kHz\\n   360 to 399 kHz:  46.8 dB at 382.4 kHz\\n\\nSpectral noise test:\\n\\n                       Levels in dB\\nkHz        0 to 31  32 to 63  64 to 95  96 to 127  Average\\n------------------------------------------------------------\\n   182.9 |    49.4 |    45.5 |    45.4 |    46.3 |    47.0 |\\n   188.4 |    53.5 |    45.7 |    45.8 |    47\n";
  unsigned char RX_NOISE_SPECTRUM_PASS[4] = {0xb0, 0x07, 0x00, 0x00};
  std::string CBMF_CPU_LINK_P = "$KSSIS,817,EM2040_130,\\xe4\\x02\\x00\\x00#IBR\\x00\\x82\\xf8\\x07\\xe4\\xe5\\x07]\\xea+\\x839\\xca\\x02\\x00\\x00\\x05\\x00CBMF-CPU link -  EM 2040\\n\\nNumber of samples requested            : 10000\\nTotal number of samples delivered      : 20000     \\n\\nWaiting for samples:\\nMaximum                                : 197    ms\\nMinimum                                : 197    ms\\nAverage last hundred                   : 197    ms\\n\\nChecking samples:\\nMaximum                                : 4      ms\\nMinimum                                : 4      ms\\nAverage last hundred                   : 4 \n";
  unsigned char CBMF_CPU_LINK_PASS[4] = {0xe4, 0x02, 0x00, 0x00};
  std::string TX_CHANNELS_VIA_RX_P = "$KSSIS,817,EM2040_130,\\xc0\\x01\\x00\\x00#IBR\\x00\\x82\\xf8\\x07\\x14\\xe6\\x07]\\xcb9\\xc6\\t\\xa4\\x01\\x00\\x00\\x08\\x00TX channels via RX -  EM 2040\\nTest of TX Channels (via RX):\\n\\nWARNING: \\nTX array is probably OK, but analysis of test data could not \\nbe completed due to the following cause:\\n\\n  - Return echo level too low.\\n\\nIn order to get reliable test results, the following conditions should be met:\\n- The vessel should not be moving.\\n- The seafloor should be flat and uniform.\\n- The water depth should be in the range 5 - 20 m.\\x00\\x00\\x00\\x00\\xc0\\x01\\x00\\x00\n";
  std::string TX_CHANNELS_VIA_RX_F = "$KSSIS,817,EM2040_130,|\\x00\\x00\\x00#IBR\\x00\\x82\\xf8\\x07\\xc7\\xf7\\x07]\\xe76\\x16\\x1b`\\x00\\x00\\x00\\x08\\xffTX channels via RX -  EM 2040\\nTest of TX Channels (via RX):\\n\\nERROR: Missing data from RX1\\n\\x00\\x00\\x00\\x00|\\x00\\x00\\x00";
  unsigned char TX_CHANNELS_VIA_RX_PASS[4] = {0xc0, 0x01, 0x00, 0x00};
  unsigned char TX_CHANNELS_VIA_RX_FAIL[4] = {'|', 0x00, 0x00, 0x00};
  std::string SW_VERSIONS_P = "$KSSIS,817,EM2040_130,\\x0c\\x01\\x00\\x00#IBR\\x00\\x82\\xf8\\x07g\\xe6\\x07]Eml!\\xf1\\x00\\x00\\x00\\x0f\\x00Software date and versions -  EM 2040\\nPU - serial 30018:\\n- CPU: 5.0.D 2019-05-21 BETA\\n- DCL: 1.0.0 2017-11-27\\n- VxWorks 6.9 SMP Build 1.09.01 Jun 29 2018 11:18:01\\n- CBMF: 1.11 15.01.22 \\n\\nRX: 1.02   Nov 12 2012 \\n\\nTX: 1.07   Mar  8 2018 \\x00\\x00\\x00\\x0c\\x01\\x00\\x00\n";
  unsigned char SW_VERSIONS_PASS[4] = {0x0c, 0x01, 0x00, 0x00};
  std::string RX_UNIT_P = "$KSSIS,817,EM2040_130,D\\x02\\x00\\x00#IBR\\x00\\x82\\xf8\\x07\\x84\\xe6\\x07]\\xa3D\\x81\\x1e(\\x02\\x00\\x00\\x03\\x00RX unit test -  EM 2040\\nNumber of RX units detected: 1 (1 expected)\\n\\nRX status:\\n  - Transducer serial number: 2017\\n  - Board ID: 0\\n  - Parts list: EM2040_RX 337744 \\n  - Product: LPFPGA 315032 G 3818 \\n  - Board reg. no.: 337744 \\n  - Board serial no.: 759866\\n  - Firmware/software rev.: 1.02   Nov 12 2012 \\n\\n  Temperature and DC power status:\\n  - FPGA die temperature [Celsius]:   45.9\\n  - FPGA voltages [Volt]: 0.97 (1.0), 2.48 (2.5), 3.30 (3.3)\\n  - Power supply [V\n";
  unsigned char RX_UNIT_PASS[4] = {'D', 0x02, 0x00, 0x00};
  std::string RX_CBMF_LINK_P = "$KSSIS,817,EM2040_130,\\xe4\\x02\\x00\\x00#IBR\\x00\\x82\\xf8\\x07\\x95\\xe6\\x07]\\xb5\\xf7\\xca\\x06\\xc9\\x02\\x00\\x00\\x06\\x00RX-CBMF link -  EM 2040\\n\\nNumber of samples requested            : 10000\\nTotal number of samples delivered      : 20000     \\n\\nWaiting for samples:\\nMaximum                                : 170    ms\\nMinimum                                : 170    ms\\nAverage last hundred                   : 170    ms\\n\\nChecking samples:\\nMaximum                                : 4      ms\\nMinimum                                : 4      ms\\nAverage last hundred                   : 4  \n";
  unsigned char RX_CBMF_LINK_PASS[4] = {0xe4, 0x02, 0x00, 0x00};
  std::string RX_NOISE_LEVEL_P = "$KSSIS,817,EM2040_130,\\xf8\\x19\\x00\\x00#IBR\\x00\\x82\\xf8\\x07\\xa3\\xe6\\x07]\\xedp\\x1b\\x1e\\xdf\\x19\\x00\\x00\\t\\x00RX noise level -  EM 2040\\n\\nNoise Test.\\nSignal Amplitude in dB\\n\\n\\nChannel          200kHz       300kHz       380kHz\\n  0              58.0         53.1         52.2 \\n  1              57.0         54.0         52.8 \\n  2              57.6         53.3         52.2 \\n  3              57.3         53.1         52.1 \\n  4              58.5         53.2         52.7 \\n  5              57.7         51.9         53.1 \\n  6              55.0         49.9         50.4 \\n  7    \n";
  unsigned char RX_NOISE_LEVEL_PASS[4] = {0xf8, 0x19, 0x00, 0x00};
  std::string SYSTEM_INFO_P = "$KSSIS,817,EM2040_130,T\\x08\\x00\\x00#IBR\\x00\\x82\\xf8\\x07\\xcc\\xe6\\x07]\\xa5\\xd1z\\x068\\x08\\x00\\x00\\x10\\x00System information -  EM 2040\\n<filename>PU30018_RX2017_TX316</filename>\\n\\n\\nSystem type: EM 2040 0.75x0.75\\n\\n***  PU  ***\\nPU serial: 30018\\nCPU Manufacturer: Concurrent Technologies\\nCPU BoardName: PP 833/09x\\nCPU PartNumber: 761-6037-27\\nCPU SerialNumber: M24622/061\\nClock 14 MHz\\nPrimary network: 192.168.100.130:0xffffff00\\nSecondary network: 105.105.105.100:0xffff0000\\nSoftware version: 5.0.D 2019-05-21 BETA\\nDCL version: 1.0.0 2017-11-27\\nVxWorks 6.9 SMP Build 1.09.01 ";
  unsigned char SYSTEM_INFO_PASS[4] = {'T', 0x08, 0x00, 0x00};
};

//enum bist_result_codes{
//  CBMF_PASS = 0x7002,
//  CBMF_FAIL = 0x9400,
//  CPU_PASS = 0x7401,
//  TX_UNIT_PASS = 0x5002,
//  TX_UNIT_FAIL = 0xb400,
//  RX_CHANNELS_PASS = 0x8c46,
//  RX_NOISE_SPECTRUM_PASS = 0xb007,
//  CBMF_CPU_LINK_PASS = 0xe402,
//  TX_CHANNELS_VIA_RX_PASS = 0xc001,
//  TX_CHANNELS_VIA_RX_FAIL = 0x7c00,
//  SW_VERSIONS_PASS = 0x0c01,
//  RX_UNIT_PASS = 0x4402,
//  RX_UNIT_FAIL = 0xb400,
//  RX_CBMF_LINK_PASS = 0xe402,
//  RX_NOISE_LEVEL_PASS = 0xf819,
//  SYSTEM_INFO_PASS = 0x5408,
//  FAIL_MASK = 0x00FF,
//};

//std::string bist_num_to_string(int num) {
//  std::string CPU = "CPU test";
//  std::string TX_UNIT = "TX unit test";
//  std::string RX_CHANNELS = "RX channels";
//  std::string RX_NOISE_SPECTRUM = "RX noise spectrum";
//  std::string CBMF = "CBMF test";
//  std::string CBMF_CPU = "CBMF-CPU link";
//  std::string TX_VIA_RX = "TX channels via RX";
//  std::string SOFTWARE_VERSIONS = "Software date and versions";
//  std::string RX_UNIT = "RX unit test";
//  std::string RX_CBMF = "RX-CBMF link";
//  std::string RX_NOISE_LEVEL = "RX noise level";
//  std::string SYSTEM_INFO = "System information";
////  switch (num) {
////    case 0 :
////      return bist_strings_def::CPU;
////    case 2 :
////      return bist_strings_def::CBMF;
////    case 3:
////      return bist_strings_def::RX_UNIT;
////    case 4:
////      return bist_strings_def::TX_UNIT;
////    case 5:
////      return bist_strings_def::CBMF_CPU;
////    case 6:
////      return bist_strings_def::RX_CBMF;
////    case 11:
////      return bist_strings_def::;
////    case 15:
////      return;
////    case 16:
////      return;
////  }
//}

//enum bist_result_codes{
//  CBMF_PASS = 0x0270,
//  CBMF_FAIL = 0x0094,
//  CPU_PASS = 0x0174,
//  TX_UNIT_PASS = 0x0250,
//  TX_UNIT_FAIL = 0x00b4,
//  RX_CHANNELS_PASS = 0x468c,
//  RX_CHANNELS_FAIL = 0x46D8,
//  RX_NOISE_SPECTRUM_PASS = 0x07b0,
//  CBMF_CPU_LINK_PASS = 0x02e4,
//  TX_CHANNELS_VIA_RX_PASS = 0x01c0,
//  TX_CHANNELS_VIA_RX_FAIL = 0x007c,
//  SW_VERSIONS_PASS = 0x010c,
//  RX_UNIT_PASS = 0x0244,
//  RX_UNIT_FAIL = 0x00b4,
//  RX_CBMF_LINK_PASS = 0x02e4,
//  RX_NOISE_LEVEL_PASS = 0x19f8,
//  SYSTEM_INFO_PASS = 0x0854,
//  FAIL_MASK = 0xFF00,
//};

//struct bist_result_def
//{
//  bool pass;
//  uint16_t result_code; //t\x01
//  uint16_t zeros; // \x00\x00
//  std::string ibr; // #IBR //ALWAYS
//  uint8_t code[18]; // \x00\x82\xf8\x07 //ALWAYS // Then the rest?
//  std::string bist_type;
//  std::string sounder_type;
//  std::vector<std::string> msg_lines;
//
//  void read_result_code(){
////    bist_strings bs;
//    switch (result_code) {
//      case bist_result_codes::CPU_PASS :
////        bist_type = bs.CPU;
//        pass = true;
//        break;
//      case bist_result_codes::TX_UNIT_PASS :
////        bist_type = bs.TX_UNIT;
//        pass = true;
//        break;
//      case bist_result_codes::RX_CHANNELS_PASS :
////        bist_type = bs.RX_CHANNELS;
//        pass = true;
//        break;
//      case bist_result_codes::RX_NOISE_SPECTRUM_PASS :
////        bist_type = bs.RX_NOISE_SPECTRUM;
//        pass = true;
//        break;
//      case bist_result_codes::CBMF_CPU_LINK_PASS :
////        bist_type = bs.CBMF_CPU;
//        pass = true;
//        break;
//      case bist_result_codes::TX_CHANNELS_VIA_RX_PASS :
////        bist_type = bs.TX_VIA_RX;
//        pass = true;
//        break;
//      case bist_result_codes::SW_VERSIONS_PASS :
////        bist_type = bs.SOFTWARE_VERSIONS;
//        pass = true;
//        break;
//      case bist_result_codes::RX_UNIT_PASS :
////        bist_type = bs.RX_UNIT;
//        pass = true;
//        break;
////      case bist_result_codes::RX_CBMF_LINK_PASS :
////        bist_type = bs.RX_CBMF;
////        pass = true;
////        break;
//      case bist_result_codes::CBMF_PASS :
////        bist_type = bs.CBMF;
//        pass = true;
//        break;
//      case bist_result_codes::RX_NOISE_LEVEL_PASS :
////        bist_type = bs.RX_NOISE_LEVEL;
//        pass = true;
//        break;
//      case bist_result_codes::SYSTEM_INFO_PASS :
////        bist_type = bs.SYSTEM_INFO;
//        pass = true;
//        break;
//      default :
//        pass = false;
//    }
//  }
//};
//typedef struct bist_result_def bist_result;

} //namespace
