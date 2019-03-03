/*
 *  Player - One Hell of a Robot Server
 *     Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
///////////////////////////////////////////////////////////////////////////
//
// Desc: USARSim (simulator) UTBot client functions
// Author: Jijun Wang
// Date: 11 May 2004
//
// 3 Mars 2005     Erik Winter 	added Ringhorne IR
// 11 Mars 2005    Erik Winter 	added RinghornePyro
// 14 Mars 2005    Erik Winter 	added call to RHPyro config
// 14 Mars 2005    Erik Winter 	Started porting USARSim to Player1.6, changed the constructor
// 18 Mars 2005    Erik Winter     Changed ir_geom from player_ir_pose_req_t
//                                 to player_ir_pose__t in the 1.6 version
// 21 April 2005   Andreas Pfeil   Made Driver loadable as a plugin and only
//                                 compatible to player 1.6
// 21 Juli 2006 Stefan Stiene and
//              Nils Rosemann      update the driver to player-2.0.2
// 20 Nov 2006  Nils Kehrein Modified laser<->bot interaction
// 22 Nov 2006  Stefan Stiene and
//              Florian Halbritter Added support for UsarSim RangeScanner3d.
///////////////////////////////////////////////////////////////////////////

#ifndef US_BOT_H
#define US_BOT_H

#include <iostream>
#include <vector>
#include <map>
#include <pthread.h>

using std::vector;
using std::map;
using std::string;

#include <libplayercore/playercore.h>

// Include a header file containing all definitions of structures etc. not yet
// included in the player headerfile of the standard release.
//#include "temp_laser3d_structures.h"

#include "us_parser.h"
/*
 * Forward declaration of classes to make the compile happy
 */
class UsRanger;
class UsLaser;
class UsSonar;
class UsPosition;
class UsPosition3d;
class UsLaser3d;
class UsIR;
class UsFakeLocalize;
class UsEncoder;
class UsFiducial;
class UsBumper;
class UsActor;

// default Gamebots address
#define DEFAULT_GAMEBOTS_IP "127.0.0.1"
#define DEFAULT_GAMEBOTS_PORT 3000

// default bot parameters
#define DEFAULT_GAMEBOTS_POS "0,0,0"
#define DEFAULT_GAMEBOTS_ROT "0,0,0"
#define DEFAULT_GAMEBOTS_CLASS "P2AT"
#define DEFAULT_TIRE_RADIUS "-1"
#define DEFAULT_ROBOT_RADIUS "-1"

/* the following setting mean that we first try to connect after 1 seconds,
 * then try every 100ms for 6 more seconds before giving up */
#define USBOT_STARTUP_USEC 1000000 /* wait before first connection attempt */
#define USBOT_MAIN_LOOP_USEC 100
#define USBOT_STARTUP_INTERVAL_USEC 100000 /* wait between connection attempts */
#define USBOT_STARTUP_CONN_LIMIT 60 /* number of attempts to make */

/* delay 10ms inside loop */
#define USBOT_DELAY_USEC 10000

#define USBOT_MAX_QUE_LEN 32
#define USBOT_MAX_MSG_LEN 4096
#define USBOT_MAX_CMD_LEN 1024
///This class implements the UTBot client handling.
/**
 * This class stands for one robot in the USARSim simulation environment. It connects to the USARSim\n
 * server (UsBot::Setup()).\n
 * After that it reads in an endless loop (UsBot::Main()) the USARSim Sensor, Geo, Conf,... Strings.\n
 * This String is send to the us_parser(UsBot::ParseData()). The parser determines the type of the String and accoring to\n
 * this type and the fact if a player driver subscribed for this data (UsBot::devices) the UsBot decides\n
 * which method in us_parser is called (or no method).\n
 * If you want to sent a string to USARSim (GETGEO, GETCONF,...)you can use the UsBot::AddCommand() function.\n
 *\n
 * The player configfile entry looks like this:\n
 *driver\n
 *(\n
 *  name "us_bot"\n
 *  provides ["simulation:0"]\n
 *  port 3000\n
 *  host "127.0.0.1"\n
 *  pos "-1,-13,0.7"\n
 *  rot "0,0,0"\n
 *  bot "USARBot.P2AT"\n
 *  botname "robot1"\n
 *  robot_radius "0.5"\n
 *  tire_radius "0.1"\n
 *)\n
 * The port and host values are the ones you run the USARSim server.\n
 * pos == start position in the map\n
 * rot == start rotation in the map\n
 * bot == bot class of this robot\n
 * botname == botname\n
 * robot_radius and tire_radius are needed to compute the DRIVE commands
 */
class UsBot : public ThreadedDriver
{
private:
  /// a queue to hold commands will be sent to Gamebots
  vector<char*> queue;
  /// switch witch determines if we preload geo and conf information of our sensors
  //bool preloadSensorInfo;
  /**
   * this method parses the usarsim string
   * called in the main loop
   */
  void ParseData(char* data);
  /**
   * this method waits a little amount of time
   * to give the devices time to send the data away
   */
  bool WaitUnlock(bool* lock);

public:
  /// Gamebots Address
  char host[512];
  /// Gamebots port
  int port;
  /// socket for Gamebots
  int sock;
  /// the subscribed devices
  int devices;

  /// Initial bot spawning parameter pose
  player_pose3d_t initPose;
  /// Initial bot spawning parameter class name
  char botClass[256];
  /// Initial bot spawning parameter roboter name
  char botName[256];

  ///
  /// actual robot position according to odometrie
  /// contains x (position->px) , y (position->py)
  /// and theta (position->pa)\n
  /// Its the difference between the INIT Position
  /// or the last odometry reset position and the actual
  /// position based on the USARSim Odometry driver or
  /// USARSim encoder sensors.
  /// position driver
  UsPosition *drvPosition;
  bool bPositionSubscribed;

  /// Encoder driver
  UsEncoder *drvEncoder;
  bool bEncoderSubscribed;
  ///enc string | value
  map<string, int> mEncoder;

  //player_robot_config_t *robotConf;
  /// robot dimensions
  //player_robot_geom_t *robotGeom;

  UsFakeLocalize *mFakeLocalize;
  /**
   * This variable contains the same information as the
   * UsBot::position variable but stores it in an player_position3d_data_t
   * struct. The z coordinate is set to zero,\n
   * The only difference is, that the speed is computed in different ways.
   * (parser::us_get_position3d() and parser::us_get_position2d())
   */
  UsPosition3d *drvPosition3d;
  bool bPosition3dSubscribed;

  ///
  map<string, UsIR*> mIR;
  map<char*, player_ir_pose_t*>* irGeom;
  map<char*, player_ir_data_t*>* irData;
  map<string, bool> mIRSubscribed;

  map<string, UsLaser3d*> mLaser3d;
  /// map identifies which laser3ds are currently subscribed
  map<string, bool> mLaser3dSubscribed;

  /// map holds laser names and the associated laser object
  map<string, UsLaser*> mLaser;
  /// map identifies which lasers are currently subscribed
  map<string, bool> mLaserSubscribed;
  /// holds pointers to the laser conf fields
  map<string, player_laser_config_t*> mLaserConfig;
  /// hold pointers to the laser geom fields
  map<string, player_laser_geom_t*> mLaserGeom;

  /// map holds sonar names and the associated sonar object
  map<string, UsSonar*> mSonar;
  /// map identifies which sonar are currently subscribed
  map<string, bool> mSonarSubscribed;
  /// hold pointers to the sonar geom fields
  map<string, player_sonar_geom_t*> mSonarGeom;

  map<string, UsRanger*> mRanger;
  map<string, bool> mRangerSubscribed;
  map<string, player_ranger_config_t*> mRangerConfig;
  map<string, player_ranger_geom_t*> mRangerGeom;
  ///pan, tilt, zoom data of the ptz (camera)
  vector<player_ptz_data_t *> ptz;
  ///position an dimension of the ptz (camera)
  vector<player_ptz_geom_t *> ptz_geom;
  ///mispkg ptz (camera) names we need this to get the corresponding MISPKG in parser
  vector<char *> ptz_name;
  ///camera names we need this to get the camera zoom in parser for the UsBot::ptz data field
  vector<char *> cam_name;
  ///new tilt and zoom for camera i available
  vector<bool> bNewPtz;
  ///new camera zoom for camera i available
  vector<bool> bNewPtzZoom;
  ///bLockPtz[i] == true -> UsPtz [i] sends data away -> lock UsBot::ptz [i]
  vector<bool> bLockPtz;
  ///bGeoPtz[i] == true -> new ptz geometry available for UsPtz i\n
  /**
   *contains pose and size of ptz
   */
  vector<bool> bGeoPtz;

  UsBumper *drvBumper;

  UsActor *drvActor;

  UsFiducial *drvFiducial;
  ///
  //player_victim_fiducial_data_t *victim_fiducial;
  ///
  bool bNewVictimFiducial;
  ///
  bool bLockVictimFiducial;

  ///
  bool bGeoRobot;
  ///
  bool bConfRobot;
  ///
  double robotMass;
  ///
  player_bbox3d_t *robotDimensions;
  ///
  double COG[3];
  ///
  double wheelBase;
  ///
  double maxSpeed;
  ///
  double maxTorque;
  ///
  char* steeringType;
  /// tire radius used to compute driving commands
  double wheelRadius;
  /// robot radius used to compute driving commands
  double maxWheelSeparation;
  ///
  double maxFrontSteer;
  ///
  double maxRearSteer;



  /**
   * Constructer
   * @param cf the player ConfigFile containing all drivers for example usarsim.cfg
   * @param section the section for this driver in the configfile
   */
  UsBot(ConfigFile* cf, int section);
  /**
   *
   */
  ~UsBot();
  /**
   * main method contains a while(true) loop
   * it reads the usarsim string from the socket
   * and calls the UsBot::ParseData() method.
   */
  void Main();
  /**
   * this method realises the connection to usarsim
   * it is called in the UsBot Constructor
   */
  int MainSetup();
  /**
   * stops the us_bot thread
   */
  void MainQuit();
  /**
   * adds a usarsim command to the UsBot::queue.
   * this command is send in the main loop
   * @param command the usarsim command
   */
  void AddCommand(char* command);
  /**
   * this method is called in UsBot::Setup() and generates a
   * usarsim INIT command and calls the UsBot::AddCommand(char * command)
   * method with this command.
   */
  void SpawnBot();

  /**
   * Registers a driver, so it is known to the bot
   * This method is called by driver objects because the
   * bot must be able to call methods of the laser object
   * @param type String identifying driver type, e.g. "RangeScanner"
   * @param name Unique name of the driver object, e.g. "Laser_Rear"
   * @param laser drv Pointer to dirver object
   */
  void RegisterDriver(char* type, char* name, Driver* drv);
  /**
   * Subscribes a driver to get data (activates parsing)
   * @param type String identifying driver type, e.g. "RangeScanner"
   * @param name Unique name of the driver object, e.g. "Laser_Rear"
   */
  void SubscribeDriver(char* type, char* name);
  /**
   * Unsubscribe a driver from data updates (deactivates parsing)
   * @param type String identifying driver type, e.g. "RangeScanner"
   * @param name Unique name of the driver object, e.g. "Laser_Rear"
   */
  void UnsubscribeDriver(char* type, char* name);
  /**
   * Request geometry data for a certain sensor
   * @param type Sensor type, e.g. "RangeScanner"
   * @param name Unique sensor name, e.g. "Laser_Rear"
   */
  void RequestGeom(char* type, char* name);
  /**
   * Request config data for a certain sensor (works currently only for lasers)
   * @param type Sensor type, e.g. RangeScanner
   * @param name Unique sensor name
   */
  void RequestConf(char* type, char* name);

};
#endif //US_BOT_H
