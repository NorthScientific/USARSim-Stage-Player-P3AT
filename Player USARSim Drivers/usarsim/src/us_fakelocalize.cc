#include "us_fakelocalize.h"

////////////////////////////////////////////////////////////////////////////////
// Constructor
UsFakeLocalize::UsFakeLocalize( ConfigFile* cf, int section)
  : Driver(cf, section)
{
    memset(&this->localize_addr, 0, sizeof(player_devaddr_t));
    memset(&this->position_addr, 0, sizeof(player_devaddr_t));
    memset(&this->odom_addr, 0, sizeof(player_devaddr_t));
    // Do we create a localize interface?
    if(cf->ReadDeviceAddr(&(this->localize_addr), section, "provides",
                          PLAYER_LOCALIZE_CODE, -1, NULL) == 0)
    {
        if(this->AddInterface(this->localize_addr))
        {
            this->SetError(-1);
            return;
        }
        //PLAYER_MSG0(1,"FAKELOCALIZE ADD LOCALIZE INTERFACE");
    }

    // Do we create a position interface?
    if(cf->ReadDeviceAddr(&(this->position_addr), section, "provides",
                          PLAYER_POSITION2D_CODE, -1, "global") == 0)
    {
        if(this->AddInterface(this->position_addr))
        {
            this->SetError(-1);
            return;
        }
        //PLAYER_MSG0(1,"FAKELOCALIZE ADD POSITION2D LOCALIZE INTERFACE");
    }
    // Do we create a position interface?
    if(cf->ReadDeviceAddr(&(this->odom_addr), section, "provides",
                          PLAYER_POSITION2D_CODE, -1, "odom") == 0)
    {
        if(this->AddInterface(this->odom_addr))
        {
            this->SetError(-1);
            return;
        }
        //PLAYER_MSG0(1,"FAKELOCALIZE ADD POSITION2D ODOMETRY INTERFACE");
    }
  // Must have an input sim
  if (cf->ReadDeviceAddr(&this->bot_id, section, "requires",
                       PLAYER_SIMULATION_CODE, -1, NULL) != 0)
  {
    this->SetError(-1);
    return;
  }
  this->bot = NULL;

  max_error.px = cf->ReadTupleLength(section,"max_error",0,0.01);
  max_error.py = cf->ReadTupleLength(section,"max_error",1,0.01);
  max_error.pa = cf->ReadTupleAngle(section,"max_error",2,0.005);

  location = new player_localize_data_t;
  location->hypoths = new player_localize_hypoth_t;
  Device* bot_device;
  if(!(bot_device = deviceTable->GetDevice(this->bot_id)))
  {
    PLAYER_ERROR("unable to locate suitable bot device");
    return;
  }
  //now I can access the fields in driver directly
  bot = ((UsBot*)bot_device->driver);
  bot->RegisterDriver((char*)"FakeLocalize",(char*)"", this);

  this->old_odom.px = bot->initPose.px;
  this->old_odom.py = -bot->initPose.py;
  this->old_odom.pa = bot->initPose.pyaw;
  return;
}
////////////////////////////////////////////////////////////////////////////////
// Constructor
UsFakeLocalize::~UsFakeLocalize()
{
	delete location->hypoths;
    delete location;
}
////////////////////////////////////////////////////////////////////////////////
// Set up the device (called by server thread).
int UsFakeLocalize::Setup()
{
    bot->SubscribeDriver((char*)"FakeLocalize",(char*)"");
    PLAYER_MSG0(2,"us_fakeLocalization setup\n");
    return 0;
}
////////////////////////////////////////////////////////////////////////////////
//
int UsFakeLocalize::ProcessMessage(QueuePointer &resp_queue, player_msghdr *hdr, void *data)
{
    if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                       PLAYER_POSITION2D_REQ_GET_GEOM,
                       this->odom_addr) ||
       Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                             PLAYER_POSITION2D_REQ_GET_GEOM,
                             this->position_addr))
    {
        int count = 0;
        PLAYER_MSG0(1,"FAKELOCALIZE ODOM REQ GEO\n");
        bot->RequestGeom((char*)"Robot", (char*)"");

        while (bot->bGeoRobot != true && count < 10 * USBOT_STARTUP_CONN_LIMIT)
        {
            usleep(USBOT_DELAY_USEC);
            count++;
        }
        if (bot->bGeoRobot != true)
        {
            PLAYER_MSG0(1,"FAKELOCALIZE ODOM REQ GEO NACK\n");
            this->Publish(this->odom_addr,resp_queue,
                          PLAYER_MSGTYPE_RESP_NACK,
                          PLAYER_POSITION2D_REQ_GET_GEOM);
            return -1;
        }
        //PLAYER_MSG3(1,"sw %f sl %f sh %f",bot->robotDimensions->sw,
        //                                  bot->robotDimensions->sl,
        //                                  bot->robotDimensions->sh);
        player_position2d_geom_t geom;
        geom.pose.px = 0.0;
        geom.pose.py = 0.0;
        geom.pose.pz = 0.0;
        geom.pose.ppitch = 0.0;
        geom.pose.proll = 0.0;
        geom.pose.pyaw = 0.0;
        geom.size.sl = bot->robotDimensions->sl;
        geom.size.sw = bot->robotDimensions->sw;
        geom.size.sh = bot->robotDimensions->sh;
        PLAYER_MSG0(3,"FAKELOCALIZE ODOM REQ GEO ACK\n");
        if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                       PLAYER_POSITION2D_REQ_GET_GEOM,
                       this->odom_addr)) {
            this->Publish(this->odom_addr,resp_queue,
                          PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_GET_GEOM,
                      (void*)&geom, sizeof(player_position2d_geom_t),NULL);
        } else {
            this->Publish(this->position_addr,resp_queue,
                          PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_GET_GEOM,
                          (void*)&geom, sizeof(player_position2d_geom_t),NULL);
        }
        return 0;
    }
    else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
             PLAYER_POSITION2D_REQ_MOTOR_POWER,
             this->odom_addr))
    {
        this->Publish(this->odom_addr,resp_queue,
                      PLAYER_MSGTYPE_RESP_ACK,
                      PLAYER_POSITION2D_REQ_MOTOR_POWER);
        return 0;
    }
    else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
             PLAYER_POSITION2D_REQ_MOTOR_POWER,
             this->position_addr))
    {
        this->Publish(this->position_addr,resp_queue,
                      PLAYER_MSGTYPE_RESP_ACK,
                      PLAYER_POSITION2D_REQ_MOTOR_POWER);
        return 0;
    }
    else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
             PLAYER_POSITION2D_REQ_RESET_ODOM,
             this->odom_addr))
    {
        this->myOdom.pos.px = 0.0;
        this->myOdom.pos.py = 0.0;
        this->myOdom.pos.pa = 0.0;
        PLAYER_MSG0(1,"Fakelocalize reset odom");
        this->Publish(this->odom_addr,resp_queue,
                      PLAYER_MSGTYPE_RESP_ACK,
                      PLAYER_POSITION2D_REQ_RESET_ODOM);
        return 0;
    }
    else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
             PLAYER_POSITION2D_REQ_SET_ODOM,
             this->odom_addr))
    {
        player_position2d_set_odom_req_t* odom= (player_position2d_set_odom_req_t*) data;
        this->myOdom.pos.px = odom->pose.px;
        this->myOdom.pos.py = odom->pose.py;
        this->myOdom.pos.pa = odom->pose.pa;
        old_odom.px = odom->pose.px;
        old_odom.py = odom->pose.py;
        old_odom.pa = odom->pose.pa;

        PLAYER_MSG3(1,"Fakelocalize setting odom to %f %f %f",this->myOdom.pos.px,this->myOdom.pos.py,this->myOdom.pos.pa);
        this->Publish(this->odom_addr,resp_queue,
                      PLAYER_MSGTYPE_RESP_ACK,
                      PLAYER_POSITION2D_REQ_SET_ODOM);
        return 0;
    }
    // Is it a request to set the filter's pose?
    else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                                  PLAYER_LOCALIZE_REQ_SET_POSE,
                                  this->localize_addr))
    {
        PLAYER_MSG0(3,"FAKE LOCALIZE set Pose");
        if(hdr->size != sizeof(player_localize_set_pose_t)) {
            PLAYER_ERROR2("request is wrong length (%d != %d); ignoring",
                          hdr->size, sizeof(player_localize_set_pose_t));
            return(-1);
        }
        player_localize_set_pose_t* setposereq;
        setposereq = (player_localize_set_pose_t*)data;

        player_pose2d_t pose;

        pose.px = setposereq->mean.px;
        pose.py = setposereq->mean.py;
        pose.pa = setposereq->mean.pa;

        PLAYER_MSG3(2,"Pose %f %f %f",pose.px,pose.py,pose.pa);
        //@todo beam robot to desired place in usarsim
        // Send an ACK
        this->Publish(this->localize_addr, resp_queue,
                      PLAYER_MSGTYPE_RESP_ACK,
                      PLAYER_LOCALIZE_REQ_SET_POSE);
        return(0);
    }
    // Is it a request for the current particle set?
    else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                                  PLAYER_LOCALIZE_REQ_GET_PARTICLES,
                                  this->localize_addr))
    {
        //@todo
        PLAYER_MSG0(1,"!!!!!!!!!!!!!!!!FAKE LOCALIZE get Particles");
        return -1;
    }
    else
    {
        printf("position hdr: %d : %d : %d : %d \n",hdr->addr.host,
           hdr->addr.robot,
           hdr->addr.interf,
           hdr->addr.index);
          printf("position this: %d : %d : %d : %d \n",this->device_addr.host,
           this->device_addr.robot,
           this->device_addr.interf,
           this->device_addr.index);
          printf("type %d subtype %d not handled\n",hdr->type, hdr->subtype);
          fflush(stdout);
    }
    return -1;
}
////////////////////////////////////////////////////////////////////////////////
// publish new data if available
void UsFakeLocalize::PublishNewData() {
    player_pose2d_t odom_diff;
    static struct timeval old_time = {0,0};
    struct timeval time;

    odom_diff.px = location->hypoths[0].mean.px - old_odom.px;
    odom_diff.py = location->hypoths[0].mean.py - old_odom.py;
    odom_diff.pa = location->hypoths[0].mean.pa - old_odom.pa;
    // calculate speeds
    gettimeofday(&time, NULL);
    double time_diff = (double)(time.tv_sec - old_time.tv_sec) +
                (double)(time.tv_usec - old_time.tv_usec)/1000000;
    myOdom.vel.pa = (location->hypoths[0].mean.pa - old_odom.pa) / time_diff;
    double dist = sqrt(SQR(location->hypoths[0].mean.px - old_odom.px)
                     + SQR(location->hypoths[0].mean.py - old_odom.py));
    //TODO vel in robot or global coordinates?
    myOdom.vel.px = cos(location->hypoths[0].mean.pa) * dist/time_diff;
    myOdom.vel.py = sin(location->hypoths[0].mean.pa) * dist/time_diff;
    old_time = time;

    odom_error.px = (2.0 * max_error.px * ((1.0*rand())/(RAND_MAX))) - max_error.px;
    odom_error.py = (2.0 * max_error.py * ((1.0*rand())/(RAND_MAX))) - max_error.py;
    odom_error.pa = (2.0 * max_error.pa * ((1.0*rand())/(RAND_MAX))) - max_error.pa;
    if(odom_diff.px != 0 || odom_diff.py != 0 || odom_diff.pa != 0) {
        myOdom.pos.px += odom_diff.px + odom_error.px;
        myOdom.pos.py += odom_diff.py + odom_error.py;
        myOdom.pos.pa += odom_diff.pa + odom_error.pa;
    }
    old_odom = location->hypoths[0].mean;
    myPosition.pos = location->hypoths[0].mean;
    myPosition.vel = myOdom.vel;

    if(this->localize_addr.interf != 0) {
        //PLAYER_MSG3(4,"fakelocalize localize: (%f, %f, %f)",location->hypoths[0].mean.px,
        //            location->hypoths[0].mean.py,
        //            location->hypoths[0].mean.pa);
        Publish(localize_addr,
                PLAYER_MSGTYPE_DATA,
                PLAYER_LOCALIZE_DATA_HYPOTHS,
                location,
                sizeof(*location));
    }
    if(this->position_addr.interf != 0) {
        Publish(position_addr,
                PLAYER_MSGTYPE_DATA,
                PLAYER_POSITION2D_DATA_STATE,
                &myPosition,
                sizeof(myPosition));
    }
    if(this->odom_addr.interf != 0) {
        //PLAYER_MSG3(1,"fakelocalize odometry  : (%f, %f, %f)",myOdom.pos.px,myOdom.pos.py,myOdom.pos.pa);
        Publish(odom_addr,
                PLAYER_MSGTYPE_DATA,
                PLAYER_POSITION2D_DATA_STATE,
                &myOdom,
                sizeof(myOdom));
    }
}

////////////////////////////////////////////////////////////////////////////////
// Shutdown the device (called by server thread).
int UsFakeLocalize::Shutdown()
{
    PLAYER_MSG0(1,"UsFakelocalize Shutdown");
    bot->UnsubscribeDriver((char*)"FakeLocalize", (char*)"");
    // Unsubscribe from devices.
    return 0;
}
