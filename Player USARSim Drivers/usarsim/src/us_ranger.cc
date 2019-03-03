#include "us_ranger.h"

///////////////////////////////////////////////////////////////////////////////
// Constructor
UsRanger::UsRanger(ConfigFile* cf, int section)  :Driver(cf, section, true, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_RANGER_CODE)
{
  if (cf->ReadDeviceAddr(&this->bot_id, section, "requires",
                         PLAYER_SIMULATION_CODE, -1,NULL) != 0)
  {
    this->SetError(-1);
    return;
  }
  strncpy(this->name, cf->ReadString(section, "ranger_name", ""), sizeof(this->name));
  pose.proll = cf->ReadTupleLength(section,"orientation",0,-1);
  pose.ppitch = cf->ReadTupleLength(section,"orientation",1,-1);
  pose.pyaw = cf->ReadTupleAngle(section,"orientation",2,-1);
  pose.px = cf->ReadTupleLength(section,"position",0,-1);
  pose.py = cf->ReadTupleLength(section,"position",1,-1);
  pose.pz = cf->ReadTupleAngle(section,"position",2,-1);
  //strncpy(this->type, cf->ReadString(section, "ranger_type", ""), sizeof(this->name));

  Device* bot_device;// bot device
  if(!(bot_device = deviceTable->GetDevice(this->bot_id)))
  {
    PLAYER_ERROR("unable to locate suitable bot device");
  }
  //now I can access the fields in driver directly


  data = new player_ranger_data_range_t;
  conf = new player_ranger_config_t;
  conf->range_res = -1;
  geom = new player_ranger_geom_t;
  geom->sensor_poses = new player_pose3d_t;
  geom->sensor_sizes = new player_bbox3d_t;
  geom->sensor_sizes_count= 10;

  bot = ((UsBot*)bot_device->driver);
  bot->RegisterDriver((char*)"Ranger", name, this);

  return;
}

////////////////////////////////////////////////////////////////////////////////
// Set up the device (called by server thread).
int UsRanger::Setup()
{
    GetGeom();
    if(GetConf()) {
        int nr = ceil((conf->max_angle - conf->min_angle) / conf->resolution);
        data->ranges = new double[nr];
    }
    else return -1;
    bot->SubscribeDriver((char*)"Ranger", name);
    //PLAYER_MSG2(1,"subscribe type %s name %s",type, name);
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
UsRanger::~UsRanger()
{
  delete data;
  delete geom->sensor_poses;
  delete geom->sensor_sizes;
  delete geom;
  delete conf;
  return;
}
////////////////////////////////////////////////////////////////////////////////
// ProcessMessages
int UsRanger::ProcessMessage(QueuePointer &resp_queue, player_msghdr *hdr, void *data)
{
    if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                             PLAYER_RANGER_REQ_GET_GEOM,
                             this->device_addr)) {
      PLAYER_MSG1(2,"RAGER %s GET GEOM",name);
      /*
      // for debugging with valgrind
      geom->pose.px = pose.px;
      geom->pose.py = pose.py;
      geom->pose.pz = pose.pz;
      geom->pose.proll = pose.proll;
      geom->pose.ppitch = pose.ppitch;
      geom->pose.pyaw = pose.pyaw;
      geom->size.sw = 0.1;
      geom->size.sl = 0.1;
      geom->size.sh = 0.1;
      geom->sensor_poses_count = 1;
      geom->sensor_sizes_count = 1;
      *(geom->sensor_poses)= (geom->pose);
      *(geom->sensor_sizes)= (geom->size);
      this->Publish(this->device_addr,
              PLAYER_MSGTYPE_RESP_ACK,
              PLAYER_RANGER_REQ_GET_GEOM,
            geom,sizeof(*geom),NULL);
      return 0;
      */
      if(!GetGeom()) {
          PLAYER_MSG0(1,"RAGER NACK GEOM");
          this->Publish(this->device_addr,
                      PLAYER_MSGTYPE_RESP_NACK,
                      PLAYER_RANGER_REQ_GET_GEOM,
                      geom,sizeof(*geom),NULL);
          return -1;
      }
      PLAYER_MSG0(2,"RAGER ACK GEOM");
      this->Publish(this->device_addr,resp_queue,
                  PLAYER_MSGTYPE_RESP_ACK,
                  PLAYER_RANGER_REQ_GET_GEOM,
                  geom,
                  sizeof(*geom),NULL);
      return 0;
  }
  else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                           PLAYER_RANGER_REQ_GET_CONFIG,
                           this->device_addr)) {
      PLAYER_MSG1(2,"RANGER %s GET CONF", name);
      /*
      // for debugging with valgrind
      conf->max_range = 20.0;
      conf->resolution = 0.017453;
      conf->min_angle = -2.35619;
      conf->max_angle = 2.35619;
      int nr = ceil((conf->max_angle - conf->min_angle) / conf->resolution);
      this->data->ranges = new double[nr];
      bot->SubscribeDriver("Ranger", name);
      this->Publish(this->device_addr,
              PLAYER_MSGTYPE_RESP_ACK,
              PLAYER_RANGER_REQ_GET_CONFIG,
              conf,sizeof(*conf),NULL);
      return 0;
      */
      if(!GetConf()) {
          PLAYER_MSG0(1,"RANGER NACK CONF");
          this->Publish(this->device_addr,
                        PLAYER_MSGTYPE_RESP_NACK,
                        PLAYER_RANGER_REQ_GET_CONFIG,
                        conf,sizeof(*conf),NULL);
          return -1;
      }
      PLAYER_MSG0(2,"RANGER ACK CONF");
      this->Publish(this->device_addr,resp_queue,
                    PLAYER_MSGTYPE_RESP_ACK,
                    PLAYER_RANGER_REQ_GET_CONFIG,
                    conf,sizeof(*conf),NULL);
      return 0;
  }
  else if(Message::MatchMessage(hdr,
          PLAYER_MSGTYPE_RESP_ACK,
          -1,this->device_addr))
  {
    // everything's fine, do nothing
    return 0;
  }
  else
  {
    printf("ranger hdr: %d : %d : %d : %d \n",hdr->addr.host,
            hdr->addr.robot,
            hdr->addr.interf,
            hdr->addr.index);
    printf("ranger this: %d : %d : %d : %d \n",this->device_addr.host,
            this->device_addr.robot,
            this->device_addr.interf,
            this->device_addr.index);
    printf("type %d subtype %d not handled\n",hdr->type, hdr->subtype);
    fflush(stdout);
    return -1;
  }
}

////////////////////////////////////////////////////////////////////////////////
// PublishNewData
void UsRanger::PublishNewData()
{
    //PLAYER_MSG2(1,"Ranger data->ranges_count %d data->ranges[0] %f",data->ranges_count, data->ranges[0]);
    this->Publish(this->device_addr,
                  PLAYER_MSGTYPE_DATA,
                  PLAYER_RANGER_DATA_RANGE,
                  data,sizeof(*data),NULL);
}

////////////////////////////////////////////////////////////////////////////////
// Shutdown the device (called by server thread).
int UsRanger::Shutdown()
{
  PLAYER_MSG1(1,"UsRanger %s - Shutdown",name);
  bot->UnsubscribeDriver((char*)"Ranger", this->name);
  return 0;
}

/**
 * Returns pointer to geometry data.
 */
bool UsRanger::GetGeom()
{
  //PLAYER_MSG0(3,"us_ranger GetGeom");
  // no geom info available -> bot must fetch it
  if(geom->sensor_sizes_count == 10) {
    int retry_count = 0;
    int sleep_count = 0;

    while(geom->sensor_sizes_count == 10 && retry_count < 10)
    {
      bot->RequestGeom((char*)"Ranger", name);
      // TODO: we need constants for sleep and retry limit
      while(geom->sensor_sizes_count == 10 && sleep_count < 1000)
      {
        sleep_count++;
        usleep(1000);
      }
      retry_count++;
      sleep_count = 0;
    }
    if(geom->sensor_sizes_count == 10){
        return false;
    }
  }
  return true;
}

bool UsRanger::GetConf()
{
    // PLAYER_MSG0(3,"us_ranger GetConf");
    // no geom info available -> bot must fetch it
    if(conf->range_res < 0) {
        int retry_count = 0;
        int sleep_count = 0;

        while(conf->range_res < 0 && retry_count < 10)
        {
            bot->RequestConf((char*)"Ranger", name);
            // TODO: we need constants for sleep and retry limit
            while(conf->range_res < 0 && sleep_count < 1000)
            {
                sleep_count++;
                usleep(1000);
            }
            retry_count++;
            sleep_count = 0;
            if(conf->range_res < 0)
            {
                PLAYER_MSG0(1,"RANGER CONF STILL NULL");
            }
        }
        if(conf->range_res < 0) {
            return false;
        }
    }
    //bot->SubscribeDriver("Ranger", name);
    return true;
}
