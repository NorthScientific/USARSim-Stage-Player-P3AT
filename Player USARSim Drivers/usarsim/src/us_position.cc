#include "us_position.h"

////////////////////////////////////////////////////////////////////////////////
//
template <typename T>
inline T sign(const T& value)
{
    if (value == 0)
        return 0;
    return (value > 0 ? T(1) : T(-1));
}

////////////////////////////////////////////////////////////////////////////////
// computes the signed minimum difference between the two angles.  inputs
// and return values are in degrees.
inline double angleDiff(double a, double b)
{
  double ra, rb;
  double d1, d2;

  ra = NORMALIZE(a);
  rb = NORMALIZE(b);

  d1 = ra-rb;
  d2 = 2.0 * M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;

  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}
////////////////////////////////////////////////////////////////////////////////
// Constructor
UsPosition::UsPosition(ConfigFile* cf, int section) : Driver(cf, section, true, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_POSITION2D_CODE)
{
  PLAYER_MSG0(2,"UsPosition - Constructor");
  if (cf->ReadDeviceAddr(&this->bot_id, section, "requires",
                         PLAYER_SIMULATION_CODE, -1,NULL) != 0)
  {
    this->SetError(-1);
    return;
  }
  strncpy(this->odo_name, cf->ReadString(section,"odo_name", "Odometry"),sizeof(this->odo_name));
  max_angle_diff = cf->ReadFloat(section, "max_angle_diff", DTOR(5));
  max_speed_diff = cf->ReadFloat(section, "max_speed_diff", 0.1);

  // bot device
  Device* bot_device;
  if(!(bot_device = deviceTable->GetDevice(this->bot_id)))
  {
    PLAYER_ERROR("unable to locate suitable bot device");
  }
  //now I can access the fields in driver directly
  bot = ((UsBot*)bot_device->driver);
  // make this position driver known for the bot
  bot->RegisterDriver((char*)"Position", odo_name, this);
  bot->SubscribeDriver((char*)"Position",(char*)"");
  last_cmd.vel.px = 0.0;
  last_cmd.vel.py = 0.0;
  last_cmd.vel.pa = 0.0;
  this->pos = new player_position2d_data;
  int count =0;
  sleep(1);
  bot->RequestConf((char*)"Robot", (char*)"");

  while (bot->bConfRobot != true && count < 10 * USBOT_STARTUP_CONN_LIMIT)
  {
      usleep(USBOT_DELAY_USEC);
      count++;
  }
  if(bot->bConfRobot == true) {
      setSteerType();
  }else {
      PLAYER_ERROR("US POSITION UNABLE TO GET ROBOT CONFIG");
  }

  count = 0;
  bot->RequestGeom((char*)"Robot", (char*)"");
  while (bot->bGeoRobot != true && count <10 * USBOT_STARTUP_CONN_LIMIT)
  {
    usleep(USBOT_DELAY_USEC);
    count++;
  }
  if (bot->bGeoRobot != true)
  {
      PLAYER_MSG0(1,"US POSITION UNABLE TO GET ROBOT GEOM");
  }

  return;
}
////////////////////////////////////////////////////////////////////////////////
// Set up the device (called by server thread).
int UsPosition::Setup()
{
  PLAYER_MSG0(2,"UsPosition - Setup");
  // init refs
  //pos  = NULL;
  bot->SubscribeDriver((char*)"Position", odo_name);
  //todo hardcoded :-(
  bot->wheelBase = 0.64;
  return 0;
}
////////////////////////////////////////////////////////////////////////////////
// Destructor
UsPosition::~UsPosition()
{
    PLAYER_MSG0(1,"UsPosition Destructor");
    delete this->pos;
    this->pos = NULL;
    return; // Nothing to do...
}

////////////////////////////////////////////////////////////////////////////////
// ProcessMessages
int UsPosition::ProcessMessage(QueuePointer &resp_queue, player_msghdr *hdr, void *data)
{
    if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                             PLAYER_POSITION2D_REQ_GET_GEOM,
                             this->device_addr))
    {
        int count = 0;
        PLAYER_MSG0(3,"POSITION REQ GEO\n");
        bot->RequestGeom((char*)"Robot", (char*)"");

        while (bot->bGeoRobot != true && count < 10 * USBOT_STARTUP_CONN_LIMIT)
        {
            usleep(USBOT_DELAY_USEC);
            count++;
        }
        if (bot->bGeoRobot != true)
        {
            PLAYER_MSG0(3,"POSITION REQ GEO NACK\n");
            this->Publish(this->device_addr,resp_queue,
                          PLAYER_MSGTYPE_RESP_NACK,
                          PLAYER_POSITION2D_REQ_GET_GEOM,
                          NULL, sizeof(player_position2d_geom_t),NULL);
            return -1;
        }
        PLAYER_MSG0(3,"POSITION REQ GEO ACK\n");
        PLAYER_MSG3(3,"sw %f sl %f sh %f",bot->robotDimensions->sw,
                                          bot->robotDimensions->sl,
                                          bot->robotDimensions->sh);
        player_position2d_geom_t geom;
        geom.pose.px = 0;
        geom.pose.py = 0;
        geom.pose.pz = 0;
        geom.size.sl = bot->robotDimensions->sl;
        geom.size.sw = bot->robotDimensions->sw;
        geom.size.sh = bot->robotDimensions->sh;
        this->Publish(this->device_addr,resp_queue,
                      PLAYER_MSGTYPE_RESP_ACK,
                      PLAYER_POSITION2D_REQ_GET_GEOM,
                      (void*)&geom, sizeof(geom),NULL);
        return 0;
    }

    else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                                   PLAYER_POSITION2D_REQ_MOTOR_POWER,
                                   this->device_addr))
    {
        bot->devices |= US_DATA_POSITION;
        this->Publish(this->device_addr,resp_queue,
                      PLAYER_MSGTYPE_RESP_ACK,
                      PLAYER_POSITION2D_REQ_MOTOR_POWER,
                      data, sizeof(player_position2d_power_config_t),NULL);
        return 0;
    }

    else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                                   PLAYER_POSITION2D_REQ_RESET_ODOM,
                                   this->device_addr))
    {
        char* cmd = new char[USBOT_MAX_CMD_LEN];
        sprintf(cmd,"SET {Type Odometry} {Name %s} {Opcode RESET} \r\n",odo_name);
        bot->AddCommand(cmd);
        this->Publish(this->device_addr,resp_queue,
                PLAYER_MSGTYPE_RESP_ACK,PLAYER_POSITION2D_REQ_RESET_ODOM,
                data, sizeof(data),NULL);
        return 0;
    }
    /****************** CAR-MODE *****************
     * only for Ackermann steered robots
     */
    else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
                                   PLAYER_POSITION2D_CMD_CAR,
                                   this->device_addr))
    {
        if(steer_type != ACKERMANNSTEERED) return -1;
        player_position2d_cmd_car_t position_cmd = *reinterpret_cast<player_position2d_cmd_car_t *> (data);
        double vel = velToSpinSpeed(position_cmd.velocity);
        double angle = position_cmd.angle;
        char* cmd = new char[USBOT_MAX_CMD_LEN];
        // Set control commands:
        sprintf(cmd,"DRIVE {Speed %f} {FrontSteer %f} {RearSteer %f}\n\r",vel,angle,angle);
        PLAYER_MSG1(4,"position ackermann cmd: %s", cmd);
        bot->AddCommand(cmd);    // Add commands to robot's queue:
        return 0;
    }
    /****************** VELOCITY-MODE  *****************
     * normale drive method for SKID robots
     */
    else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
                                   PLAYER_POSITION2D_CMD_VEL,
                                   this->device_addr))
    {
        player_position2d_cmd_vel_t position_cmd = *reinterpret_cast<player_position2d_cmd_vel_t *> (data);
        if(fabs(position_cmd.vel.px - last_cmd.vel.px) < 0.001 &&
           fabs(position_cmd.vel.py - last_cmd.vel.py) < 0.001 &&
           fabs(position_cmd.vel.pa - last_cmd.vel.pa) < 0.001) return 0;
        //PLAYER_MSG3(1,"Steered vel_x %f vel_y %f vel_a %f ",position_cmd.vel.px,position_cmd.vel.py,position_cmd.vel.pa);
        char* cmd = new char[USBOT_MAX_CMD_LEN];
        if(steer_type == SKIDSTEERED)
        {
            if (bot->maxWheelSeparation == -1) {
            	bot->maxWheelSeparation = 0.4;
            }
            if (bot->wheelRadius == -1) {
            	bot->wheelRadius = 0.05;
            }
            double speed_left = (position_cmd.vel.px - (bot->maxWheelSeparation * position_cmd.vel.pa));
            double speed_right = (position_cmd.vel.px + (bot->maxWheelSeparation * position_cmd.vel.pa));
            double spin_speed_left = velToSpinSpeed(speed_left);
            double spin_speed_right = velToSpinSpeed(speed_right);
            sprintf(cmd,"DRIVE {Left %f} {Right %f}\r\n", spin_speed_left, spin_speed_right);
            //PLAYER_MSG2(1,"SKIDSteered_2 maxWheelSeparation %f wheel_radius %f ",bot->maxWheelSeparation, bot->wheelRadius);
            //PLAYER_MSG1(1,"position vel cmd: %s", cmd);
        }
        else if(steer_type == ACKERMANNSTEERED)
        {
            PLAYER_MSG0(4,"ACKERMAN STEERED");
            sprintf(cmd,"DRIVE {Speed %f} {FrontSteer %f} {RearSteer %f}\n\r",position_cmd.vel.px,position_cmd.vel.pa,position_cmd.vel.pa);//max 45°
        }
        else if(steer_type == HOLONOMSTEERED)
        {
            PLAYER_MSG3(1,"Holonom Steered vel_x %f vel_y %f vel_a %f ",position_cmd.vel.px,position_cmd.vel.py,position_cmd.vel.pa);
            double v_f,v_r, delta_r,delta_f;
            double speed = sqrt(position_cmd.vel.px*position_cmd.vel.px + position_cmd.vel.py*position_cmd.vel.py);
            double direction = atan2(position_cmd.vel.py,position_cmd.vel.px);
            double yawRate = position_cmd.vel.pa;
            this->InvBicycleKinematic(direction,speed,yawRate,delta_r,delta_f,v_r,v_f);
            testConfiguration(v_r,delta_r,v_f,delta_f,DTOR(130));
            this->last_wheelspeed_left = v_r;
            this->last_wheelspeed_right = v_f;
            this->last_wheelsteer_left = delta_r;
            this->last_wheelsteer_right = delta_f;
            sprintf(cmd,"DRIVE {WheelNumber 0} {WheelSpeed %f} {WheelSteer %f} "
                              "{WheelNumber 1} {WheelSpeed %f} {WheelSteer %f}\n\r",v_r,delta_r,v_f,delta_f);

            PLAYER_MSG1(1,"position vel cmd: %s", cmd);
        }
        else {
            PLAYER_ERROR1("US POSITION UNKNOWN DRIVING TYPE %d", steer_type);
            delete[] cmd;
            cmd = NULL;
            return -1;
        }
        if(cmd != NULL) {
            last_cmd.vel.px = position_cmd.vel.px;
            last_cmd.vel.py = position_cmd.vel.py;
            last_cmd.vel.pa = position_cmd.vel.pa;
              bot->AddCommand(cmd);	 //publishusarsim
        }
        /*
        if(wait_180_wheel_turn) {
            sleep(2);
            wait_180_wheel_turn  = false;
        }
        */
        return 0;
    }
    return -1;
}

////////////////////////////////////////////////////////////////////////////////
// PublishNewData
void UsPosition::PublishNewData()
{
    if (pos != NULL) {
        this->Publish(this->device_addr,
        PLAYER_MSGTYPE_DATA,
        PLAYER_POSITION2D_DATA_STATE, (void*)this->pos,
                sizeof(player_position2d_data_t), NULL);
    }
}

////////////////////////////////////////////////////////////////////////////////
// Shutdown the device (called by server thread).
int UsPosition::Shutdown()
{
  bot->UnsubscribeDriver((char*)"Position", odo_name);
  PLAYER_MSG0(1,"UsPosition - Shutdown");
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
//
bool UsPosition::setSteerType()
{
  if(bot->steeringType == NULL) return false;
  PLAYER_MSG1(3,"bot->steeringType %s",bot->steeringType);
  if(strstr(bot->steeringType,"SkidSteered") != NULL) steer_type = SKIDSTEERED;
  else if(strstr(bot->steeringType,"AckermanSteered") != NULL) steer_type = ACKERMANNSTEERED;
  else if(strstr(bot->steeringType,"HolonomSteered") != NULL) steer_type = HOLONOMSTEERED;
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// converts m/s to spin speed rad/s
double UsPosition::velToSpinSpeed(double vel)
{
    //PLAYER_MSG2(1,"vel %f radius %f",vel,bot->wheelRadius);
    return (vel / (2 * bot->wheelRadius * M_PI) ) * 2.0 * M_PI;  //  m/s -> rad/s
}

////////////////////////////////////////////////////////////////////////////////
// converts rad/s to spin speed rad/s
double UsPosition::rotRateToSpinSpeed(double rotRate)
{
    //PLAYER_MSG2(1,"vel %f radius %f",vel,bot->wheelRadius);
    double rotCirclePerimeter = bot->wheelBase * M_PI;
    double circleDistancePerSecond = rotCirclePerimeter / (2.0 * M_PI) * rotRate;
    //PLAYER_MSG3(1,"wheelBase %f perimeter %f circle segment %f",bot->wheelBase, rotCirclePerimeter,circleDistancePerSecond);
    return (circleDistancePerSecond / (2.0 * bot->wheelRadius * M_PI) ) * 2.0 * M_PI;  //  m/s -> rad/s
}

bool UsPosition::testConfiguration(double &wheelspeed_left,
                                   double &wheelsteer_left,
                                   double &wheelspeed_right,
                                   double &wheelsteer_right,double max_wheel_steer)
{
    if(fabs(angleDiff(wheelsteer_left,last_wheelsteer_left)) > M_PI/2.0) {
        //PLAYER_MSG2(1,"right old %f new %f",last_wheelsteer_left, wheelsteer_left);
        wheelspeed_left *=(-1.0);
        if(wheelsteer_left < 0)
            wheelsteer_left += M_PI;
        else
            wheelsteer_left -= M_PI;

        //PLAYER_MSG1(1,"MODIFIED left %f",wheelsteer_left);
    }
    if(fabs(angleDiff(wheelsteer_right,last_wheelsteer_right)) > M_PI/2.0) {
        //PLAYER_MSG2(1,"right old %f new %f",last_wheelsteer_right, wheelsteer_right);
        wheelspeed_right *=(-1.0);
        if(wheelsteer_right < 0)
            wheelsteer_right += M_PI;
        else
            wheelsteer_right -= M_PI;
        //PLAYER_MSG1(1,"MODIFIED right %f",wheelsteer_right);
    }
    if(wheelsteer_left > max_wheel_steer) {
        wheelsteer_left -= M_PI;
        wheelspeed_left *=(-1.0);
        //PLAYER_MSG1(1,"MODIFIED left %f",wheelsteer_left);
        wait_180_wheel_turn  = true;
    }
    else if(wheelsteer_left < -max_wheel_steer) {
        wheelsteer_left += M_PI;
        wheelspeed_left *=(-1.0);
        //PLAYER_MSG1(1,"MODIFIED left %f",wheelsteer_left);
        wait_180_wheel_turn  = true;
    }
    if(wheelsteer_right > max_wheel_steer) {
        wheelsteer_right -= M_PI;
        wheelspeed_right *=(-1.0);
        //PLAYER_MSG1(1,"MODIFIED right %f",wheelsteer_right);
        wait_180_wheel_turn  = true;
    }
    else if(wheelsteer_right < -max_wheel_steer) {
        wheelsteer_right += M_PI;
        wheelspeed_right *=(-1.0);
        wait_180_wheel_turn  = true;
        //PLAYER_MSG1(1,"MODIFIED right %f",wheelsteer_right);
    }

    //@todo der lisa roboter kann nicht mit zu großen Winkeländerungen arbeiten
    /*
    if(fabs(angleDiff(wheelsteer_left,last_wheelsteer_left)) > max_angle_diff) {
        PLAYER_WARN2("CHANGING LEFT WHEEL STEER FROM %f TO %f",wheelsteer_left,last_wheelsteer_left);
        if(angleDiff(wheelsteer_left,last_wheelsteer_left) < 0)
            wheelsteer_left = last_wheelsteer_left - max_angle_diff;
        else
            wheelsteer_left = last_wheelsteer_left + max_angle_diff;
    }
    if(fabs(angleDiff(wheelsteer_right,last_wheelsteer_right)) > max_angle_diff) {
        PLAYER_WARN2("CHANGING RIGHT WHEEL STEER FROM %f TO %f",wheelsteer_right,last_wheelsteer_right);
        if(angleDiff(wheelsteer_right,last_wheelsteer_right) < 0)
            wheelsteer_right = last_wheelsteer_left - max_angle_diff;
        else
            wheelsteer_right = last_wheelsteer_left + max_angle_diff;
    }
    */
    return true;
}

/////////////////////////////////////////////////////////////////////////////////
//
void UsPosition::InvBicycleKinematic(double direction, double speed, double yawRate,
                                     double &delta_r, double &delta_f, double &v_r, double &v_f)
{
    double l_r = bot->wheelBase/2.0; //distance rear wheel to robot reference point
    double l_f = bot->wheelBase/2.0; //distance front wheel to robot reference point
    double r = 0, r_f, r_r;
    //angle between wheel diagonal and robot x axis
    double delta_0 = atan2(this->bot->robotDimensions->sw,this->bot->robotDimensions->sl);
    double beta = direction + delta_0;
    //PLAYER_MSG4(1,"direction %f speed %f yawRate %f beta %f",direction,speed, yawRate,beta);
    if(speed == 0.0 ) {
        if(yawRate != 0.0) { //special case turn on place velocity vector is zero
            delta_r = delta_f = M_PI/2.0;
            v_f = rotRateToSpinSpeed(yawRate);
            v_r = -v_f;
        } else {             // stop command
            v_r = v_f = 0.0;
            delta_r = delta_f = beta;
        }
    } else if(cos(beta) != 0.0){
        delta_r = atan(tan(beta) - ((yawRate*(l_f + l_r))/(2.0 * speed * cos(beta))));  //rear wheel angle
        delta_f = atan(tan(beta) + ((yawRate*(l_f + l_r))/(2.0 * speed * cos(beta))));  //front wheel angle
    } else { //special case beta = pi/2
        v_r = speed;
        v_f = speed;
        delta_f = delta_r = M_PI/2.0;
    }
    if(delta_f != delta_r && beta != 0.0) {
        r = fabs((l_f + l_r)/(cos(beta) * (tan(delta_f) - tan(delta_r))));
        r_f = sqrt(r * r + l_f * l_f - 2.0 * r * l_f * sin(beta));
        r_r = sqrt(r * r + l_r * l_r - 2.0 * r * l_r * sin(-beta));
        v_r = speed * (r_r/r);
        v_f = speed * (r_f/r);
        v_r = velToSpinSpeed(v_r);  //convert to rad/sekunde (unreal)
        v_f = velToSpinSpeed(v_f);  //convert to rad/sekunde (unreal)
        //PLAYER_MSG3(1,"r %f r_r %f r_f %f",r,r_r,r_f);
    }
    else if(delta_f == delta_r && speed != 0.0) { //parallel steer
        v_r = speed;
        v_f = speed;
        v_r = velToSpinSpeed(v_r);  //convert to rad/sekunde (unreal)
        v_f = velToSpinSpeed(v_f);  //convert to rad/sekunde (unreal)
    }
    //hier weiter tan beta hat polstelle bei vielfachem von pi /2

    if(beta > (M_PI/2.0)) {
        delta_r += M_PI;
        delta_f += M_PI;
    }
    if(beta < -(M_PI/2.0)) {
        delta_r -= M_PI;
        delta_f -= M_PI;
    }
    //PLAYER_MSG2(1,"vorher  delta_f %f delta_r %f",delta_f,delta_r);
    delta_r -=delta_0;
    delta_f -=delta_0;
    //PLAYER_MSG2(1,"nachher delta_f %f delta_r %f",delta_f,delta_r);
    if(delta_r > M_PI) delta_r -= (2.0 * M_PI);
    else if(delta_r < -M_PI) delta_r += (2.0 * M_PI);
    if(delta_f >  M_PI) delta_f -= (2.0 * M_PI);
    else if(delta_f < -M_PI) delta_f += (2.0 * M_PI);
}
