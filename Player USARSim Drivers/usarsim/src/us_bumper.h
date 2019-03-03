/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
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
// Desc: usarsim (simulator) fiducial driver
// Author: Stefan Markov
// Date: 23 Feb 2006
//
// Modified: 18 aug 2006 Stefan Stiene update to player-2.0.2 but not tested
///////////////////////////////////////////////////////////////////////////
#ifndef US_BUMPER_H
#define US_BUMPER_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "libplayercore/player.h"

#include "us_bot.h"

//#include "debug.h"
///player fiducial driver !!!not tested yet!!!
/**
 * This class implements the USARSim RFID sensor.\n
 * Like all us_... sensor player drivers it gets the corresponding bot from\n
 * the player device table (UsBumper::Setup()). Handles all incomming Messages\n
 * in the Usfiducial::ProcessMessage() function and publish the data to each player\n
 * driver that has subscribed for this device using the UsBumper::PublishNewData()\n
 * function.\n
 * So far only support for one RFID sensor per robot is added.\n
 * \n
 * The player configfile entry looks like this:\n
 *driver\n
 *(\n
 *  name "us_fiducial"\n
 *  provides ["fiducial:0"]\n
 *  requires ["simulation:0"]\n
 *  fiducial_name "RFID1"\n
 *)\n
 * provides == this driver provides the player laser interface\n
 * requires == the UsBot this RFID is mounted on.\n
 * fiducial_name == the name of RFID sensor.(not used yet)\n
 */
class UsBumper : public Driver
{

public:
  /**
   * Constructor
   * @param cf Configfile
   * @param section The section in the configfile
   */
  UsBumper(ConfigFile* cf, int section);
  /**
   * Destructor
   */
  ~UsBumper();
  /**
   * get the UsBumper::bot using the UsBumper::bot_id
   */
  int Setup();
  int Shutdown();
  /**
   * this method handles messages send to this device. Hence this device
   * works directly on the UsBot member variables, this method only needs to
   * catch a player request for the fiducials's geometry
   */
  int ProcessMessage(QueuePointer &resp_queue, player_msghdr* hdr,void* data);
  /**
   * this method publish new fiducial data (if available) to any
   * device that has subscribed to this device.
   */
  void PublishNewData();
  ///
  player_bumper_data_t *bumper_data;
  player_bumper_geom_t *geom;
  bool gotGeom;
  /// usarsim bumper name
  char name[128];
private:
  /// bot player device addr
  player_devaddr_t bot_id;
  /// bot player driver using this driver we can access the us_bot fields directly
  UsBot* bot;
};
#endif // US_BUMPER_H

