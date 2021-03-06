/*
 * Desc: Libraray for parsing Gamebots messages
 * Author: Jijun Wang
 * Date: 13 MAY 2004

 // Modified:
// 3 Mars 2005 	   Erik Winter 	added Ringhorne IR
// 11 Mars 2005    Erik Winter 	added RinghornePyro
// 14 Mars 2005    Erik Winter 	added RHPyro config
// 14 Mars 2005    Erik Winter Started porting USARSim to Player1.6,
//		   added PLAYERV_1_6 constant
// 18 Mars 2005    Erik Winter Changed ir_geom from player_ir_pose_req_t to player_ir_pose__t in the 1.6 version
// 22 Nov 2006     Florian Halbritter Added support for UsarSim RangeScanner3d
*/

#ifndef USPARSER_H
#define USPARSER_H

#include "libplayercore/player.h"
#include <libplayercore/playercore.h>
#include <map>
#include <stdlib.h>
#include <stdint.h>
#include <string>
#include <sstream>
#include <iostream>
#include <math.h>
#include <sys/time.h>
#include <time.h>

using namespace std;

#include <fstream>

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************
 * Constants, etc
 **************************************************************************/
#define US_MAX_MSG_LEN      4096

#define US_DATA_POSITION   0x01
//#define US_DATA_POSITION3D 0x01<<1
#define US_DATA_LASER      0x01<<1
#define US_DATA_SONAR      0x01<<2
#define US_DATA_PTZ        0x01<<3
#define US_DATA_IR         0x01<<4
#define US_DATA_INU        0x01<<5
#define US_DATA_ODOM       0x01<<6
#define US_DATA_FIDUCIAL   0x01<<7
#define US_DATA_VICTIM_FIDUCIAL 0x01<<8
#define US_DATA_ENCODER    0x01<<9
#define US_DATA_LASER3D    0x01<<10

#define US_GEOM_LASER      0x01<<11
#define US_GEOM_SONAR      0x01<<12
#define US_GEOM_IR         0x01<<13
#define US_GEOM_PTZ        0x01<<14
#define US_GEOM_LASER3D    0x01<<15
#define US_GEOM_ROBOT      0x01<<16

#define US_CONF_LASER      0x01<<17
#define US_CONF_SONAR      0x01<<18
#define US_CONF_IR         0x01<<19
#define US_CONF_CAMERA     0x01<<20
#define US_CONF_ROBOT      0x01<<21
#define US_CONF_LASER3D    0x01<<22

#define US_GROUND_TRUTH    0x01<<23

#define US_DATA_RANGER    0x01<<24
#define US_GEOM_RANGER    0x01<<25
#define US_CONF_RANGER    0x01<<26

#define US_GEOM_BUMPER    0x01<<27
#define US_DATA_BUMPER    0x01<<28
#define US_DATA_ACTOR     0x01<<29
// basic parse functions
int us_get_word(char* data, int pos, char* word);
int us_get_type_and_name(char* data, char** ppBody,string &name);
int us_get_segment(char* data, int pos, char* segment);
int us_get_segmentByName(char* data, int pos, char* name, char* segment);
int us_get_value(char* segment, char* name, char* value);
char* us_get_body(char* data);

// get data we are interested in
int us_get_position(char* data, player_position2d_data_t *position);
/**
 * Parses encoder ticks
 * Message format looks like:
 * @verbatim
 * {Type Encoder} {Name ECLeft Tick -331} {Name ECRight Tick -4}
 * @endverbatim
 *
 * @param data Message string
 * @param mEnc Map of encoder name and tick count
 * @return 0 if successful, else error occured
 */
int us_get_encoder(char* data, map<string, int> &mEnc);
int us_get_position3d(char* data, player_position3d_data_t *position3d);
int us_get_groundTruth(char* data, player_localize_data_t *location);
int us_get_groundTruthPos(char* data, player_position2d_data_t *pose);
int us_get_sonar(char* data,const char* name, player_sonar_data_t *sonar);
int us_get_ir(char* data, char* name, player_ir_data_t *ir);
int us_get_laser(char* data, player_laser_data_t *laser);
int us_get_ranger(char* data, player_ranger_data_range_t *laser);
int us_get_ptz(char* data,char* name, player_ptz_data_t *ptz, player_ptz_geom_t *ptz_geom);

int us_get_sonar_geom_all(char* data, map<string, player_sonar_geom_t*> mGeom);
int us_get_ir_geom(char* data, player_ir_pose_t *ir,char* name);
int us_get_ranger_geom_all(char*, map<string,player_ranger_geom_t*> mGeom);
int us_get_ir_geom_all(char* data, map<char*, player_ir_pose_t*> *mGeom);
//int us_get_bumper_geom(char* data, player_bumper_geom_t *bumper_geom);
/**
 * Parses laser geometry information for several lasers at once
 * Parses a packet that has been requested by the command
 * 'GETGEOM {Type RangeScanner}' without any name information.
 * Information is passed within the map, user has to cleanup (delete)
 * unneeded objects.
 *
 * @param data Message string
 * @param mGeom Map that will be filled with player_laser_geom objects
 * @return 0 if successful, else a value <= -1 defining the error
 */
int us_get_laser_geom_all(char* data, map<string, player_laser_geom_t*> mGeom);
int us_get_laser_config(char* data,char* name, player_laser_config_t *laser);
/**
 * Parses laser config information for several lasers at once
 * Parses a packet that has been requested by the command
 * 'GETCONF {Type RangeScanner}' without any name information.
 * Information is passed within the map, user has to cleanup (delete)
 * unneeded objects.
 *
 * @param data Message string
 * @param mConf Map that will be filled with player_laser_config objects
 * @return 0 if successful, else a value <= -1 defining the error
 */
int us_get_laser_config_all(char* data, map<string, player_laser_config_t*> mConf);
int us_get_ranger_config_all(char*, map<string, player_ranger_config_t*> mConf);
int us_get_camera_config(char* data,char* name, player_ptz_data_t *sonar);
int us_get_robot_config(char* data, char* steeringType, double &mass, double &maxSpeed,
            double &maxTorque, double &maxFrontSteer, double &maxRearSteer);
int us_get_robot_geom(char* data, player_bbox3d_t *dimensions, double* COG, double &wheelRadius,
                      double &maxWheelSeparation, double &wheelBase);
int us_get_inu(char* data, player_position3d_data_t *inu);
int us_get_fiducial(char* data,player_fiducial_data_t *fid);
int us_get_bumper(char* data1, player_bumper_data_t *data2);
//int us_get_victim_fiducial(char* data, player_victim_fiducial_data_t *fid);

/**
 * Transforms the given sensory input into a list of 3D point data.
 * The signal is of the form:
 * {Name sensorName} {Resolution vertRes, horizRes} {FOV vertFOV, horizFOV} {Range r1, r2, ...}
 * Assumes that the scans are traversed row-wise, i.e. that for one fixed vertical angle
 * first all possible horizontal angles are scanned before the procedure is repeated for
 * the next vertical angle.
 *
 * @param data A string of the form specified above.
 * @param name Name of the scanner (to be parsed from the data).
 * @param laser3d The result will be stored in this object.
 *
 * @return '-1' if failed at any stage (e.g. if the given sensory message is not a
 *         3D rangescanner message), '0' otherwise
 */
int us_get_laser3d(char* data, player_pointcloud3d_data_t *laser3d);

#ifdef __cplusplus
}
#endif

#endif
