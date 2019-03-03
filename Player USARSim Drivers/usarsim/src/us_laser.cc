#include "us_laser.h"

///////////////////////////////////////////////////////////////////////////////
// Constructor
UsLaser::UsLaser(ConfigFile* cf, int section):Driver(cf, section, true, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_LASER_CODE)
{
  if (cf->ReadDeviceAddr(&this->bot_id, section, "requires",
                         PLAYER_SIMULATION_CODE, -1,NULL) != 0)
  {
    this->SetError(-1);
    return;
  }
  strncpy(this->name, cf->ReadString(section, "laser_name", ""), sizeof(this->name));

  Device* bot_device;// bot device
  if(!(bot_device = deviceTable->GetDevice(this->bot_id)))
  {
    PLAYER_ERROR("unable to locate suitable bot device");
  }

  data = new player_laser_data_t;
  conf = new player_laser_config_t;
  conf->range_res = -1;
  geom = new player_laser_geom_t;
  geom->size.sl = -1;
  geom->size.sw = -1;
  geom->size.sh = -1;
  geom->pose.px = -1;
  geom->pose.py = -1;
  geom->pose.pz = -1;
  //now I can access the fields in driver directly
  bot = ((UsBot*)bot_device->driver);
  bot->RegisterDriver((char*)"RangeScanner", name, this);
  return;
}

////////////////////////////////////////////////////////////////////////////////
// Set up the device (called by server thread).
int UsLaser::Setup()
{
    GetGeom();
    GetConf();
    this->data->intensity = new uint8_t[(int)ceil((this->conf->max_angle - this->conf->min_angle) / this->conf->resolution)];
    this->data->ranges = new float[(int)ceil((this->conf->max_angle - this->conf->min_angle) / this->conf->resolution)];
    bot->SubscribeDriver((char*)"RangeScanner", name);
    PLAYER_MSG1(1,"subscribe %s", name);
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
UsLaser::~UsLaser()
{
    delete data;
    delete geom;
    delete conf;
    return;
}

////////////////////////////////////////////////////////////////////////////////
// ProcessMessages
int UsLaser::ProcessMessage(QueuePointer &resp_queue, player_msghdr *hdr, void *data)
{
  if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                           PLAYER_LASER_REQ_GET_GEOM,
                           this->device_addr)) {
    if (!GetGeom())
    {
      PLAYER_ERROR("US LASER REG GEOM NACK");
      this->Publish(this->device_addr,
                    PLAYER_MSGTYPE_RESP_NACK,
                    PLAYER_LASER_REQ_GET_GEOM,
                    data,sizeof(data),NULL);
      return -1;
    }
    this->Publish(this->device_addr,
                  PLAYER_MSGTYPE_RESP_ACK,
                  PLAYER_LASER_REQ_GET_GEOM,
                  geom,
                  sizeof(*geom),NULL);
    return 0;
  }
  else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                            PLAYER_LASER_REQ_GET_CONFIG,
                            this->device_addr)) {
     if (!GetConf())
     {
       PLAYER_ERROR("US LASER REG CONF NACK");
       this->Publish(this->device_addr,
                     PLAYER_MSGTYPE_RESP_NACK,
                     PLAYER_LASER_REQ_GET_CONFIG,
                     data,sizeof(data),NULL);
       return -1;
     }
     this->Publish(this->device_addr,
                   PLAYER_MSGTYPE_RESP_ACK,
                   PLAYER_LASER_REQ_GET_CONFIG,
                   conf,
                   sizeof(*conf),NULL);
     return 0;
   }
    else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                                  PLAYER_LASER_REQ_GET_ID,
                                  this->device_addr))
    {
      PLAYER_MSG0(2,"RANGERINTEGRATOR GET LASER ID");
      player_laser_get_id_config_t id;
      id.serial_number = 1;
      this->Publish(this->device_addr,
           PLAYER_MSGTYPE_RESP_ACK,
           PLAYER_LASER_REQ_GET_ID,
           (void*)&id, sizeof(id), NULL);
      return 0;
    }
  else
  {
    printf("laser hdr: %d : %d : %d : %d \n",hdr->addr.host,
            hdr->addr.robot,
            hdr->addr.interf,
            hdr->addr.index);
    printf("laser this: %d : %d : %d : %d \n",this->device_addr.host,
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
void UsLaser::PublishNewData()
{
  if (conf->range_res != -1)
  {
    data->min_angle  = conf->min_angle;
    data->max_angle  = conf->max_angle;
    data->max_range  = conf->max_range;
    data->resolution = conf->resolution;
    // workaround: usarsim is not precise enough for vfh
    if(fabs(data->resolution - 0.0174) <= 0.0002) {
      data->resolution = 0.0174533;
    }
    if(fabs(data->min_angle + 1.57075) <= 0.00003) {
        data->min_angle = -1.5707963;
    }
    if(fabs(data->max_angle - 1.57075) <= 0.00003) {
        data->max_angle = 1.5707963;
    }
    //data->resolution = 0.0174533;
    //data->min_angle = -1.5707963;
    //data->min_angle = 1.5707963;
    //PLAYER_MSG5(4,"min_a %f max_a %f max_r %f res %f count %d\n", data->min_angle,
    //data->max_angle, data->max_range, data->resolution,data->ranges_count);
    this->Publish(this->device_addr,
                  PLAYER_MSGTYPE_DATA,
                  PLAYER_LASER_DATA_SCAN,
                  data,sizeof(*data),NULL);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Shutdown the device (called by server thread).
int UsLaser::Shutdown()
{
  //fprintf(stderr,"UsLaser - Shutdown\n");
  bot->UnsubscribeDriver((char*)"RangeScanner", this->name);
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// get geom from bot
bool UsLaser::GetGeom()
{
  PLAYER_MSG1(3,"us_laser %s GetGeom", name);
  // no geom info available -> bot must fetch it
  if(geom->size.sl == -1)
  {
    int retry_count = 0;
    int sleep_count = 0;

    while(geom->size.sl == -1 && retry_count < 10)
    {
      bot->RequestGeom((char*)"RangeScanner", name);
      // TODO: we need constants for sleep and retry limit
      while(geom->size.sl == -1 && sleep_count < 1000)
      {
        sleep_count++;
        usleep(1000);
      }
      retry_count++;
      sleep_count = 0;
    }
    if(geom->size.sl == -1) return false;
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// get conf from bot
bool UsLaser::GetConf()
{
  PLAYER_MSG1(3,"us_laser %s GetConf", name);
  // no geom info available -> bot must fetch it
  if(conf->range_res == -1)
  {
    int retry_count = 0;
    int sleep_count = 0;

    while(conf->range_res == -1 && retry_count < 10)
    {
      bot->RequestConf((char*)"RangeScanner", name);
      // TODO: we need constants for sleep and retry limit
      while(conf->range_res == -1 && sleep_count < 1000)
      {
        sleep_count++;
        usleep(1000);
      }
      retry_count++;
      sleep_count = 0;
    }
    if(conf->range_res == -1) return false;
  }
  return true;
}
