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
// Desc: USARSim (simulator) Bot functions
// Author: Jijun Wang
// Date: 11 May 2004
// Modified:
// 3 Mars 2005     Erik Winter 	added Ringhorne IR
// 11 Mars 2005    Erik Winter 	added RinghornePyro
// 14 Mars 2005    Erik Winter 	added call to RHPyro config
// 14 Mars 2005    Erik Winter   Started porting USARSim to Player1.6
// 15 Mars 2005    Erik Winter   Continued porting, it compiles but gives segmentation faults
// 16 Mars 2005    Erik Winter   No more segmentation faults, can get sensor data but not geometry information
// 21 April 2005   Andreas Pfeil Made Driver loadable as a plugin and only
//                               compatible to player 1.6
// 15 June 2005    Stefan Markov            Added support for providing robot name when
//                                          spawning a bot.
// 12 July 2005    Stefan Markov            Added support for providing robot rotation
//                                          when spawning a bot.
// 12 July 2006    Stefan Stiene            Upgrade the whole thing to player 2.0.2
//                 and Nils Rosemann
// 22 Nov 2006     Stefan Stiene
//                 and Florian Halbritter   Added support for UsarSim RangeScanner3d
///////////////////////////////////////////////////////////////////////////

#include <assert.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>  /* for strncpy(3),memcpy(3) */
#include <unistd.h> /* close(2),fcntl(2),getpid(2),usleep(3),execlp(3),fork(2)*/
#include <fcntl.h>  /* for fcntl(2) */
#include <sys/socket.h>  /* for accept(2) */
#include <sys/types.h>  /* for socket(2) */
#include <netdb.h> /* for gethostbyaddr(3) */
#include <netinet/in.h> /* for struct sockaddr_in, SOCK_STREAM */
//#include <libplayertcp/socket_util.h> /* for create_and_bind_socket() */
#include <pthread.h>  /* for pthread stuff */
#include <sys/select.h> /* for select (2) call */

#include <time.h>

#include "us_bot.h"
#include "us_laser.h"
#include "us_ranger.h"
#include "us_position.h"
#include "us_position3d.h"
#include "us_ir.h"
#include "us_laser3d.h"
#include "us_fakelocalize.h"
#include "us_fiducial.h"
#include "us_bumper.h"
#include "us_ptz.h"
#include "us_sonar.h"
///////////////////////////////////////////////////////////////////////////
// Instantiate an instance of this driver
Driver* USARSim_Init(ConfigFile* cf, int section)
{
  char modul[128];
  strncpy(modul, cf->ReadString(section, "modul",""), sizeof(modul));

  if(strcmp(modul,"ranger") == 0) {
      PLAYER_MSG0(3,"init USARSim ranger");
      return ((Driver*) (new UsRanger(cf, section)));
  }
  else if (strcmp(modul,"laser") == 0) {
      PLAYER_MSG0(3,"init USARSim laser");
      return ((Driver*) (new UsLaser(cf, section)));
  }
  else if (strcmp(modul,"position2d") == 0) {
      PLAYER_MSG0(3,"init USARSim position2d");
      return ((Driver*) (new UsPosition(cf, section)));
  }
  else if (strcmp(modul,"fakelocalize") == 0) {
      PLAYER_MSG0(3,"init USARSim fakelocalize");
      return ((Driver*) (new UsFakeLocalize(cf, section)));
  }
  else if (strcmp(modul,"fiducial") == 0) {
      PLAYER_MSG0(3,"init USARSim fiducial");
      return ((Driver*) (new UsFiducial(cf, section)));
  }
  else if (strcmp(modul,"bumper") == 0) {
      PLAYER_MSG0(3,"init USARSim bumper");
     return ((Driver*) (new UsBumper(cf, section)));
  }
  else if (strcmp(modul,"laser3d") == 0) {
      PLAYER_MSG0(3,"init USARSim laser3d");
      return ((Driver*) (new UsLaser3d(cf, section)));
  }
  else if (strcmp(modul,"ptz") == 0) {
      PLAYER_MSG0(3,"init USARSim ptz");
      return ((Driver*) (new UsPtz(cf, section)));
  }
  else if (strcmp(modul,"sonar") == 0) {
      PLAYER_MSG0(3,"init USARSim sonar");
      return ((Driver*) (new UsSonar(cf, section)));
  }
  else if (strcmp(modul,"ir") == 0) {
      PLAYER_MSG0(3,"init USARSim ir");
      return ((Driver*) (new UsIR(cf, section)));
  }
  else if (strcmp(modul,"bot") == 0) {
      PLAYER_MSG0(3,"init USARSim bot");
      return ((Driver*) (new UsBot(cf, section)));
  }
  PLAYER_MSG1(0,"ERROR USARSim unknown modul: %",modul);
  return NULL;
}


///////////////////////////////////////////////////////////////////////////
// Register driver
void USARSim_Register(DriverTable* table)
{
  table->AddDriver((char*)"usarsim", USARSim_Init);
}

////////////////////////////////////////////////////////////////////////////////
// Constructor
//
/////////////////////////////////////////////////////////////////////////
UsBot::UsBot(ConfigFile* cf, int section) :
    ThreadedDriver(cf, section, true, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_SIMULATION_CODE)
{
  sock = -1;
  devices = 0;
  strncpy(this->host, cf->ReadString(section, "host", DEFAULT_GAMEBOTS_IP), sizeof(this->host));
  this->port = cf->ReadInt(section, "port", DEFAULT_GAMEBOTS_PORT);
  initPose.px = cf->ReadTupleLength(section,"pos",0,0);
  initPose.py = cf->ReadTupleLength(section,"pos",1,0);
  initPose.pz = cf->ReadTupleLength(section,"pos",2,0);
  initPose.proll = cf->ReadTupleAngle(section,"rot",0,0);
  initPose.ppitch = cf->ReadTupleAngle(section,"rot",1,0);
  initPose.pyaw = cf->ReadTupleAngle(section,"rot",2,0);
  strncpy(this->botClass, cf->ReadString(section, "bot", DEFAULT_GAMEBOTS_CLASS), sizeof(this->botClass));
  strncpy(this->botName, cf->ReadString(section, "botname", DEFAULT_GAMEBOTS_CLASS), sizeof(this->botName));
  //Move these initializations in front of the function body ?

  robotDimensions = new player_bbox3d_t();
  steeringType = new char[128];


  bPositionSubscribed = false;
  bEncoderSubscribed  = false;

  irData = NULL;  //TODO rewrite ir to register subscribe unsubscribe
  irGeom = NULL;  //put data and geom in ir driver

  drvEncoder = NULL;
  drvPosition = NULL;
  drvPosition3d = NULL;
  drvFiducial = NULL;
  drvBumper = NULL;

  bNewVictimFiducial = false;
  bLockVictimFiducial = false;

  bConfRobot = false;
  bGeoRobot = false;
  maxWheelSeparation = -1;
  wheelRadius = -1;
  COG[0] = 0.0;
  COG[1] = 0.0;
  COG[2] = 0.0;
  wheelBase = 0.0;
  Setup();
  return;
}
/**
 * Destructor
 */
UsBot::~UsBot()
{
  // TODO: delete all maps
  fprintf(stderr,"UsBot -- Destructor\n");
  Shutdown();
  delete[] this->steeringType;
  delete[] this->robotDimensions;
  return;
}
/**
 * Setup the UsBot -- create communication with Gamebots
 */
int UsBot::MainSetup()
{
  static struct sockaddr_in server;
  struct hostent* entp;
  int j;
  PLAYER_MSG2(1,"Player <-> USARSim connection initializing (%s:%d)...", host, port);
  /******************************************************************/
  // Use getaddrinfo(3) to be halfway IPv6 capable?
  // fill in addr structure
  server.sin_family = PF_INET;
  // this is okay to do, because gethostbyname(3) does no lookup if the
  // 'host' * arg is already an IP addr
  if((entp = gethostbyname(host)) == NULL) {
    fprintf(stderr, "UsBot::Setup(): \"%s\" is unknown host; "
      "can't connect to gamebots\n", host);
    return(1);
  }
  memcpy(&server.sin_addr, entp->h_addr_list[0], entp->h_length);
  server.sin_port = htons(port);
  // ok, we'll make this a bit smarter.  first, we wait a baseline amount
  // of time, then try to connect periodically for some predefined number
  // of times
  usleep(USBOT_STARTUP_USEC);

  for(j = 0;j < USBOT_STARTUP_CONN_LIMIT; j++) {
    // make a new socket, because connect() screws with the old one somehow
    if((sock = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
      perror("UsBot::Setup(): socket(2) failed");
      return(1);
    }
    // hook it up
    if(connect(sock, (struct sockaddr*)&server, sizeof(server)) == 0) break;
    usleep(USBOT_STARTUP_INTERVAL_USEC);
  }
  if(j == USBOT_STARTUP_CONN_LIMIT) {
    perror("UsBot::Setup(): connect(2) failed");
    perror("Start USARSim before starting player server");
    exit(0);
    return(-1);
  }
  puts("Done.");
  // Until here.
  /***********************************************************************/
  /*****************************/
  // Left out - changed "readline" procedure to use select.
  // make it nonblocking
  //   if(fcntl(sock,F_SETFL,O_NONBLOCK) < 0) {
  //     perror("UsBot::Setup(): fcntl(2) failed");
  //     return(1);
  //   }
  //Until here.
  SpawnBot();  // Spawn a bot into UT world
  return(0);
}


/**
 * Shutdown the UsBot
 */
void UsBot::MainQuit()
{
  fprintf(stderr,"UsBot -- Shutdown\n");
  /* if Setup() was never called, don't do anything */
  if(sock == -1)
    return;

  sock = -1;
  puts("UsBot device has been shutdown");
}
/**
 * The main function that reads data from Gamebots
 */
void UsBot::Main()
{
    // use this to hold temp data
    char buf[USBOT_MAX_MSG_LEN+1];
    // loop and read
    while (1) {
        // perhaps we have to do something like this so us_bot doesn't take
        // all computation time in a very fast while loop if no device wants data
        pthread_testcancel(); // test if we are supposed to cancel
        // TODO: at the beginning there is no answer from usarsim
        while (!(queue.empty())) { //if there is a usarsim command send it
            char* tmpstr = queue.back(); //Took Command out of queue
            //
            if(tmpstr == NULL) {
                queue.pop_back();
                continue;
            }
            if (write(sock, tmpstr, strlen(tmpstr)) == -1)// send command to Gamebots
            {
                perror("USBot Thread: write() failed sending string; exiting.");
                break;
            }
            //delete[] tmpstr;// delete or not there is a segmentation fault here
                        // perhaps some module create the string without new
            queue.pop_back();
            PLAYER_MSG1(9,"bot cmd: %s", tmpstr);

        }
        //Change this to use select(2) on a blocking port?
        //- This would allow to read lines continuously after each other.
        // read a line
        unsigned int numread = 0;
        buf[numread] = 0;
        // space: number of bytes left in buffer - one less than the size to be
        // able to null-terminate string
        int space = sizeof(buf) - numread;

        bool timeout = false;
        while ( (!strchr(buf, '\n')) && (space>0)) {
            // Set read timeout to 100 ms... - so that we stop reading a line,
            // if we did not get anything for 100ms. This can lead to a big delay
            // (100ms * sizeof(buf)), if chars arrive one-by-one, but it should be
            // okay.
            struct timeval tv = { 0, 100*1000 };
            fd_set fd;
            FD_ZERO(&fd);
            FD_SET(sock, &fd);
               int r = select(sock+1, &fd, NULL, NULL, &tv);
            if (r < 0) {
                fprintf(stderr, "%s: While waiting for socket: %s", __PRETTY_FUNCTION__, strerror(errno));
            } else if (r == 0) {
                // Timeout
                timeout = true;
                break;
            }
            // read as much as there is for the socket.
            if ((r = read(sock, &buf[numread], 1)) < 0) {
                perror("when reading from network");
                break;
            }
            // update numread
            numread += r;
            // update how much space is left in the buffer
            space = sizeof(buf) - numread;
            // null-terminate buffer to allow strchr to work properly
            buf[numread]='\0';
        }
        //If we exit from here, we'll have read exactly one line into the buffer
        //or encountered a timeout/read error.
        if (timeout) {
            PLAYER_MSG0(6,"Read Timeout while trying to read from server");
        } else {
            //struct timeval start,end;
            //gettimeofday(&start, NULL);
            ParseData(buf);
            //gettimeofday(&end, NULL);
            //double time_diff = (double)(end.tv_sec - start.tv_sec) +
            //          (double)(end.tv_usec - start.tv_usec)/1000000;
            //PLAYER_MSG1(1,"Parse Data dauerte %lf seconds",time_diff);
        }
    }
}
/**
 *
 */
void UsBot::SpawnBot()
{
  char* cmd = new char[USBOT_MAX_CMD_LEN];
  cmd[0] = 0;
  sprintf(cmd,"INIT {ClassName %s} {Name %s} {Location %f %f %f} {Rotation %f %f %f}\r\n",botClass,botName,initPose.px,initPose.py,initPose.pz,initPose.proll,initPose.ppitch,initPose.pyaw);
  AddCommand(cmd);
}
/**
 *
 */
void UsBot::AddCommand(char* command)
{
    queue.push_back(command);
}
/**
 *
 */
void UsBot::ParseData(char* data)
{
  char *pBody;
  string name;
  int type = us_get_type_and_name(data,&pBody,name);
  if (type==-1 || name.size() == 0) return;
  //PLAYER_MSG2(1, "US BOT PARSE DATA TYPE %d NAME %s",type, name.c_str());
  //
  // FAKELOCALIZE
  //
  if(type & devices & US_GROUND_TRUTH) {
      //PLAYER_MSG0(1,"us bot parsing ground truth");
      if(us_get_groundTruth(pBody, mFakeLocalize->location) < 0) {
          PLAYER_MSG0(1,"US_BOT ERROR PARSING GROUND TRUTH SENSOR");
      }
      mFakeLocalize->PublishNewData();
  }
  //
  // Ground Truth parse data of ground truth sensor
  //
  if(type & devices & US_GROUND_TRUTH && drvPosition != NULL)
  {
      if(us_get_groundTruthPos(pBody, drvPosition->pos) < 0) {
          PLAYER_ERROR("us_bot: position data parsing error");
      }
      else {
          drvPosition->PublishNewData();
      }
  }
  //
  // Odometry parse data of odometry device
  //
  if(type & devices & US_DATA_POSITION && drvPosition != NULL)
  {
      if(us_get_position(pBody, drvPosition->pos) < 0) {
          PLAYER_ERROR("us_bot: position data parsing error");
      }
      else {
          drvPosition->PublishNewData();
      }
  }
  //
  // PTZ
  //
  if (type & devices & US_DATA_PTZ)
  {
      for(unsigned int i=0; i < ptz.size();i++) {
          if(ptz[i]==NULL) break;
          bool dummy = bLockPtz[i];
          if (!bLockPtz[i] || WaitUnlock(&dummy)) {
              if( us_get_ptz(pBody,cam_name.at(i),ptz.at(i),ptz_geom[i]) == 0){
                  bNewPtz.at(i) = true;
                  bGeoPtz.at(i) = true;  //ptz sen also has geo information
              }
          }
      }
  }
  //
  // IMU
  //
  else if ((type & devices & US_DATA_INU) && (drvPosition3d!=NULL)){
      if(us_get_inu(pBody,drvPosition3d->data) < 0) {
          PLAYER_ERROR("ERROR PARSING IMU DATA");
      } else {
          drvPosition3d->PublishNewData();
      }
  }
  //
  // FIDUCIAL
  //
  else if ((type & devices & US_DATA_FIDUCIAL) && (drvFiducial!=NULL))
  {
      if (us_get_fiducial(pBody,drvFiducial->fiducial_data) < 0) {
          PLAYER_ERROR("ERROR PARSING FIDUCIAL DATA");
      } else {
          drvFiducial->PublishNewData();
      }
  }
  //
  // BUMPER
  //
  else if ((type & devices & US_DATA_BUMPER) && (drvBumper!=NULL))
  {
      PLAYER_MSG0(1,"us bot us_get_bumper");
      if (us_get_bumper(pBody,drvBumper->bumper_data) < 0) {
          PLAYER_ERROR("ERROR PARSING BUMPER DATA");
      } else {
          drvBumper->PublishNewData();
      }
  }
  else if ((type & devices & US_GEOM_BUMPER) &&
           (drvBumper != NULL)) {
      PLAYER_MSG0(1,"us bot TODO us_get_bumper_geom");
      /*
      if(us_get_bumper_geom(pBody,drvBumper->geom) < 0)
      {
          PLAYER_ERROR("ERROR PARSING BUMPER GEOM\n");
      } else {
          drvBumper->gotGeom = true;
      }
      */
  }
  /*
  else if ((type & devices & US_DATA_VICTIM_FIDUCIAL) && (victim_fiducial!=NULL))
  {
      bNewVictimFiducial = false;
      if (!bLockVictimFiducial || WaitUnlock(&bLockVictimFiducial))
    us_get_victim_fiducial(pBody,victim_fiducial);
      bNewVictimFiducial = true;
  }
    */


  //
  //  RANGER
  //
  else if (type & devices & US_GEOM_RANGER)
  {
    if(us_get_ranger_geom_all(pBody, mRangerGeom) < 0){
      PLAYER_MSG0(1,"us_bot: ranger geom parsing error");
    }
  }
  else if (type & devices & US_CONF_RANGER)
  {
    if(us_get_ranger_config_all(pBody, mRangerConfig) < 0) {
        PLAYER_MSG0(1,"us_bot: ranger conf parsing error");
    }
  }
  else if (type & devices & US_DATA_RANGER && mRangerSubscribed.find(name) != mRangerSubscribed.end())
  {
    if(us_get_ranger(pBody,mRanger[name]->data) < 0)
    {
        PLAYER_MSG0(1,"us_bot: ranger data parsing error");
    }
    mRanger[name]->PublishNewData();
  }
  //
  // LASER3D
  //
  else if (type & devices & US_DATA_LASER3D && mLaser3dSubscribed.find(name) != mLaser3dSubscribed.end())
  {
      if(us_get_laser3d(pBody, mLaser3d[name]->data) < 0) {
          PLAYER_MSG0(1,"us_bot: laser3d data parsing error");
      }
      mLaser3d[name]->PublishNewData();
  }
  //
  // LASER
  //
  else if (type & devices & US_GEOM_LASER)
  {
    if(us_get_laser_geom_all(pBody, mLaserGeom) < 0){
        PLAYER_MSG0(1,"us_bot: laser geom parsing error");
    }
  }
  else if (type & devices & US_CONF_LASER)
  {
      if(us_get_laser_config_all(pBody, mLaserConfig) < 0) {
          PLAYER_MSG0(1,"us_bot: laser conf parsing error");
      }
  }
  else if (type & devices & US_DATA_LASER && mLaserSubscribed.find(name) != mLaserSubscribed.end())
  {
      if(us_get_laser(pBody,mLaser[name]->data) < 0)
      {
          PLAYER_MSG0(1,"us_bot: laser data parsing error");
      }
      mLaser[name]->PublishNewData();
  }
  //
  // IR
  //
  else if (type & devices & US_GEOM_IR)
  {
      map<char*, player_ir_pose_t*>::iterator iter;
      for(iter = irGeom->begin(); iter != (*irGeom).end(); iter++) {
          if(us_get_ir_geom(pBody,(*iter).second,(*iter).first)!=-1){
              string s((*iter).first);
              if (mIR[s] != NULL) {
                  mIR[s]->SetGeom((*iter).second);
              }
              else PLAYER_MSG1(1,"ERROR IN GEOM IR MIR == NULL %s\n",s.c_str());
          }
          else PLAYER_MSG0(1,"ERROR IN GEOM IR\n");
      }
  }
  else if ((type & devices & US_DATA_IR) && (irData != NULL))
  {
      map<char*, player_ir_data_t*>::iterator iter = (*irData).begin();
      for(; iter != irData->end(); iter++) {
          (*irData)[iter->first] = new player_ir_data_t();
          if(us_get_ir(pBody, iter->first, iter->second) == 0)
          {
              if (mIRSubscribed[ iter->first ]) {
                  mIR[ iter->first ]->SetData( iter->second );
              }
          }
      }
  }
  //
  // SONAR
  //
  else if (type & devices & US_GEOM_SONAR)
  {
      //PLAYER_MSG0(1,"US BOT get geom sonar");
      if(us_get_sonar_geom_all(pBody,this->mSonarGeom) < 0){
          PLAYER_ERROR("ERROR SONAR GEOM PARSING");
      }
      else {
           map<string,UsSonar*>::iterator iter;
           for(iter = mSonar.begin(); iter != mSonar.end();iter++) {
               iter->second->gotGeom = true;
           }
      }
  }
  else if (type & devices & US_DATA_SONAR)
  {
      map<string,UsSonar*>::iterator iter;
      for(iter = mSonar.begin(); iter != mSonar.end();iter++) {
          if(!(this->mSonarSubscribed[iter->first]))continue;
          if(us_get_sonar(pBody,iter->first.c_str(),iter->second->sonar_data) < 0)
          {
              PLAYER_MSG0(1,"us_bot: laser data parsing error");
          }
          iter->second->PublishNewData();
      }
  }

  /*
  else if (type & devices & US_GEOM_PTZ)
  {
    cout<<"ptz Geom"<<endl;
    for(unsigned int i = 0; i < bGeoPtz.size(); i++) {
   cout<<i<<endl;
   if(us_get_ptz_geom(pBody,ptz_name[i],ptz_geom[i])!=-1){
     bGeoPtz.at(i) = true;
   }
    }
  }
  */
  else if (type & devices & US_CONF_CAMERA)
  {
    for(unsigned int i = 0; i < bGeoPtz.size(); i++) {
   if(us_get_camera_config(pBody,cam_name[i],ptz[i])!=-1){
     bNewPtzZoom.at(i) = true;
   }
    }
  }
  else if ((type & devices & US_CONF_ROBOT) &&
           steeringType !=NULL)
  {
    bConfRobot = false;
    //PLAYER_MSG0(3,"us bot get conf robot");
    if(us_get_robot_config(pBody,steeringType, robotMass, maxSpeed,
                           maxTorque, maxFrontSteer,maxRearSteer) == 0)
    {
      PLAYER_MSG0(3,"us bot has conf");
      bConfRobot = true;
    }
    else PLAYER_MSG0(1,"ERROR PARSING ROBOT CONF\n");
  }
  else if ((type & devices & US_GEOM_ROBOT) &&
           robotDimensions != NULL) {
      if(us_get_robot_geom(pBody,robotDimensions,
                             COG,wheelRadius,
                             maxWheelSeparation, wheelBase) < 0)
      {
          PLAYER_ERROR("ERROR PARSING ROBOT GEO\n");
      } else {
          bGeoRobot = true;
      }
  }
}
/*
 *
 */
bool UsBot::WaitUnlock(bool* lock) {
  int count = 20;
  int delay = USBOT_DELAY_USEC/10;

  while(*lock && count-->0) usleep(delay);

  return count>0;
}

/*
 * Register a driver object so we can access it
 */
void UsBot::RegisterDriver(char* type, char* name, Driver* drv)
{
    PLAYER_MSG2(3,"US_BOT: REGISTER TYPE %s NAME %s",type,name);
  if(strcmp(type, "RangeScanner") == 0) {
      mLaser[string(name)] = (UsLaser*)drv;
      mLaserConfig[string(name)] = ((UsLaser*)drv)->conf;
      mLaserGeom[string(name)] = ((UsLaser*)drv)->geom;
  }else if(strcmp(type, "Sonar") == 0) {
      mSonar[string(name)] = (UsSonar*)drv;
      mSonarGeom[string(name)] = ((UsSonar*)drv)->geom;
  }else if(strcmp(type, "Ranger") == 0) {
      mRanger[string(name)] = (UsRanger*)drv;
      mRangerConfig[string(name)] = ((UsRanger*)drv)->conf;
      mRangerGeom[string(name)] = ((UsRanger*)drv)->geom;
  }else if(strcmp(type, "RangeScanner3d") == 0) {
      mLaser3d[string(name)] = (UsLaser3d*)drv;
  } else if(strcmp(type, "Position") == 0) {
    drvPosition = (UsPosition*)drv;
  } else if(strcmp(type, "Position3d") == 0) {
    drvPosition3d = (UsPosition3d*)drv;
  } else if(strcmp(type, "Encoder") == 0) {
    drvEncoder = (UsEncoder*)drv;
  } else if(strcmp(type, "FakeLocalize") == 0) {
      mFakeLocalize = (UsFakeLocalize*)drv;
  } else if(strcmp(type, "Fiducial") == 0) {
      drvFiducial = (UsFiducial*)drv;
  } else if(strcmp(type, "Bumper") == 0) {
      drvBumper = (UsBumper*)drv;
  } else if(strcmp(type, "Actor") == 0) {
      drvActor = (UsActor*)drv;
  }
  else if(strcmp(type, "IR") == 0) {
      //since all sonar values are in one command I have to initialize it here
      //geom
      if (irGeom == NULL) irGeom = new map<char*, player_ir_pose_t*>;
      if (irGeom->find(name) == irGeom->end())(*irGeom)[name] = new player_ir_pose_t;
      //data
      if (irData == NULL) irData = new map<char*, player_ir_data_t*>;
      if (irData->find(name) == irData->end())(*irData)[name] = new player_ir_data_t;
      //driver
      mIR[string(name)] = (UsIR*)drv;
  } else {
    PLAYER_ERROR2("unhandled driver type: %s, %s", type, name);
  }
}

/*
 * Subcribes a driver for data updates
 */
void UsBot::SubscribeDriver(char* type, char* name)
{
  if(strcmp(type, "RangeScanner") == 0)
  {
      PLAYER_MSG0(1,"US_BOT subscribe RangeScanner");
      mLaserSubscribed[string(name)] = true;
      devices |= US_DATA_LASER;
  }
  else if(strcmp(type, "Sonar") == 0)
  {
      PLAYER_MSG0(1,"US_BOT subscribe Sonar");
      mSonarSubscribed[string(name)] = true;
      devices |= US_DATA_SONAR;
  }
  else if(strcmp(type, "Ranger") == 0)
  {
    mRangerSubscribed[string(name)] = true;
    devices |= US_DATA_RANGER;
  }
  else if(strcmp(type, "RangeScanner3d") == 0)
  {
    mLaser3dSubscribed[string(name)] = true;
    devices |= US_DATA_LASER3D;
  }
  else if(strcmp(type, "Position") == 0)
  {
    // seems like noone needs this data...
    this->bPositionSubscribed = true;
    devices |= US_DATA_POSITION;
  }
  else if(strcmp(type, "Position3d") == 0)
  {
    this->bPosition3dSubscribed = true;
    this->devices |= US_DATA_POSITION;
  }
  else if(strcmp(type, "Encoder") == 0)
  {
    this->bEncoderSubscribed = true;
    devices |= US_DATA_ENCODER;
  }
  else if(strcmp(type, "IR") == 0)
  {
      mIRSubscribed[string(name)] = true;
      //this->bIRSubscribed = true;
      devices |= US_DATA_IR;
  }
  else if(strcmp(type, "FakeLocalize") == 0)
  {
    devices |= US_GROUND_TRUTH;
  }
  else if(strcmp(type, "Fiducial") == 0)
  {
    devices |= US_DATA_FIDUCIAL;
  }
  else if(strcmp(type, "Bumper") == 0)
  {
    devices |= US_DATA_BUMPER;
  }
  else if(strcmp(type, "Actor") == 0)
  {
    devices |= US_DATA_ACTOR;
  }
  else {
    PLAYER_ERROR2("unhandled driver subscribe: %s, %s", type, name);
  }
}

/*
 * Unsubscribes a driver from updates
 */
void UsBot::UnsubscribeDriver(char* type, char* name)
{
  if(strcmp(type, "RangeScanner") == 0)
  {
      mLaserSubscribed.erase(string(name));
      if(mLaserSubscribed.size() == 0) {
      this->devices &= ~(US_DATA_LASER);
      }
  }
  else if(strcmp(type, "Sonar") == 0)
  {
      mSonarSubscribed.erase(string(name));
      if(mSonarSubscribed.size() == 0) {
      this->devices &= ~(US_DATA_SONAR);
      }
  }
  else if(strcmp(type, "Ranger") == 0)
  {
    mRangerSubscribed.erase(string(name));
    if(mRangerSubscribed.size() == 0) {
      this->devices &= ~(US_DATA_RANGER);
    }
  }
  else if(strcmp(type, "RangeScanner3d") == 0)
  {
    mLaser3dSubscribed.erase(string(name));
    if(mLaser3dSubscribed.size() == 0) {
      this->devices &= ~(US_DATA_LASER3D);
    }
  }
  else if(strcmp(type, "Position") == 0)
  {
    this->bPositionSubscribed = false;
    this->devices &= ~(US_DATA_POSITION);
  }
  else if(strcmp(type, "Position3d") == 0)
  {
    this->bPosition3dSubscribed = false;
    this->devices &= ~(US_DATA_INU);
  }
  else if(strcmp(type, "Encoder") == 0)
  {
    this->bEncoderSubscribed = false;
    this->devices &= ~(US_DATA_ENCODER);
  }
  else if(strcmp(type, "IR") == 0)
  {
      mIRSubscribed.erase(string(name));
      if(mIRSubscribed.size() == 0) {
          this->devices &= ~(US_DATA_IR);
      }
  }
  else if(strcmp(type, "FakeLocalize") == 0)
  {
    this->devices &= ~(US_GROUND_TRUTH);
  }
  else if(strcmp(type, "Fiducial") == 0)
  {
    this->devices &= ~(US_DATA_FIDUCIAL);
  }
  else if(strcmp(type, "Bumper") == 0)
  {
    this->devices &= ~(US_DATA_BUMPER);
  }
  else if(strcmp(type, "Actor") == 0)
  {
    this->devices &= ~(US_DATA_ACTOR);
  }
  else {
    PLAYER_ERROR2("unhandled driver unsubscribe: %s, %s", type, name);
  }
}

/*
 * Request geometry information from USARSim, will be pushed to driver
 */
void UsBot::RequestGeom(char* type, char* name)
{
  if(strcmp(type, "RangeScanner") == 0) {
      this->devices |= US_GEOM_LASER;
  }
  else if(strcmp(type, "Sonar") == 0) {
        this->devices |= US_GEOM_SONAR;
  }
  else if(strcmp(type, "Bumper") == 0) {
        this->devices |= US_GEOM_BUMPER;
        type=(char*)"Touch";
  }
  else if(strcmp(type, "Ranger") == 0) {
      this->devices |= US_GEOM_RANGER;
      type = (char*)"RangeScanner";//testing damit sowohl laser als auch ranger funktionieren
  }
  else if(strcmp(type, "RangeScanner3d") == 0) {
      this->devices |= US_GEOM_LASER3D;
  }
  else if(strcmp(type, "Robot") == 0) {
    this->devices |= US_GEOM_ROBOT;
  }
  else if(strcmp(type, "IR") == 0) {
      this->devices |= US_GEOM_IR;
  }
  else {
    PLAYER_ERROR2("unhandled geom type: %s, %s", type, name);
    return;
  }

  char* cmd = new char[USBOT_MAX_CMD_LEN];
  // request data only for one specific sensor
  //sprintf(cmd, "GETGEO {Type %s} {Name %s}\r\n", type, name);
  // request data for all sensor of same type
  sprintf(cmd, "GETGEO {Type %s}\r\n", type);
  this->AddCommand(cmd);
}
/*
 * Request config information from USARSim, will be pushed to driver
 */
void UsBot::RequestConf(char* type, char* name)
{
  if(strcmp(type, "RangeScanner") == 0) {
      this->devices |= US_CONF_LASER;
  }
  else if(strcmp(type, "Ranger") == 0) {
      this->devices |= US_CONF_RANGER;
      type = (char*)"RangeScanner";//testing damit sowohl laser als auch ranger funktionieren
  }
  else if(strcmp(type, "RangeScanner3d") == 0) {
      this->devices |= US_CONF_LASER3D;
  }
  else if(strcmp(type, "Robot") == 0) {
      this->devices |= US_CONF_ROBOT;
  }
  else if(strcmp(type, "IR") == 0) {
      this->devices |= US_CONF_IR;
  }
  else {
    PLAYER_ERROR2("unhandled config type: %s, %s", type, name);
    return;
  }

  char* cmd = new char[USBOT_MAX_CMD_LEN];
  // request data only for one specific sensor
  //sprintf(cmd, "GETCONF {Type %s} {Name %s}\r\n", type, name);
  // request data for all sensor of same type
  sprintf(cmd, "GETCONF {Type %s}\r\n", type);
  this->AddCommand(cmd);
}

extern "C" {
    int player_driver_init(DriverTable* table)
    {
        USARSim_Register(table);
        return(0);
    }
}
