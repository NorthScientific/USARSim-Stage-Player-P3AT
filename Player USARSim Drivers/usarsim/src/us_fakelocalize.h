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
#ifndef US_FAKELOCALIZE_H
#define US_FAKELOCALIZE_H

#include <libplayercore/player.h>

#include "us_bot.h"

#define SLEEPTIME_US 100000
#define SQR(x) ((x)*(x))
/// Driver for ground truth localization
/**
 * The UsFakeLocalize driver polls a simulation device (UsBot::location) for
 * 2D ground truth pose
 * data, then reports this data as if it were generated by a localization
 * system.  This driver is useful for running software (e.g., util_playernav,
 * driver_wavefront) that needs a interface_localize device without incurring
 * the computational cost of actually running a localization algorithm.\n
 * \n
 * The player configfile entry looks like this:\n
 * driver\n
 * (\n
 *  name "us_fakelocalize"\n
 *  provides ["position2d:1"]\n
 *  requires ["simulation:0"]\n
 *  origin [2.0 5.0 0]\n
 * )\n
 * provides == this driver provides the player position2d interface\n
 * requires == the UsBot this localization is for.\n
 * origin == adjust localization to player map.\n
 */
class UsFakeLocalize : public Driver
{
  public:
  /**
   * Constructor
   * @param cf Configfile
   * @param section The section in the configfile
   */
  UsFakeLocalize( ConfigFile* cf, int section);
  ~UsFakeLocalize();
  /**
   * this method connects to the UsBot and alocates the data fields like UsBot::sonar
   */
  int Setup();
  /**
   * this method handles messages send to this device.
   */
  int ProcessMessage(QueuePointer &resp_queue, player_msghdr* hdr,void* data);
  int Shutdown();
  /**
  * this method publish new laser data (if available) to any
  * device that has subscribed for this laser.
  */
  void PublishNewData();
  /// ground truth robot position in global coordinate system
  player_localize_data_t *location;
private:
    // interfaces we might be using
    player_devaddr_t position_addr;
    player_devaddr_t odom_addr;
    player_devaddr_t localize_addr;

    /// bot player device addr
    player_devaddr_t bot_id;
    /// bot player driver using this driver we can access the us_bot fields directly
    UsBot* bot;
    /// using this value you can adjust the localization to the player map.
    player_pose2d_t origin;
    /// buffer position data to prevent sigseg faults (necessary?)
    player_position2d_data_t myPosition;
    /// buffer position data to prevent sigseg faults (necessary?)
    player_position2d_data_t myOdom;

    player_pose2d_t old_odom;
    player_pose2d_t odom_error;
    player_pose2d_t max_error;
};
#endif //US_FAKELOCALIZE_H