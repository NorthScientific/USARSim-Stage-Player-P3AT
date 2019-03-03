#include "us_laser3d.h"

///////////////////////////////////////////////////////////////////////////////
// Constructor
UsLaser3d::UsLaser3d(ConfigFile* cf, int section):Driver(cf, section, true,
                       PLAYER_MSGQUEUE_DEFAULT_MAXLEN,
                       PLAYER_POINTCLOUD3D_CODE)
{
  fprintf(stderr,"UsLaser3d - Constructor\n");
  if (cf->ReadDeviceAddr(&this->bot_id, section, "requires",
                         PLAYER_SIMULATION_CODE, -1,NULL) != 0){
    this->SetError(-1);
    return;
  }
  strncpy(this->name, cf->ReadString(section, "laser3d_name", ""), sizeof(this->name));
  // init pointer to local data

  Device* bot_device;// bot device
  if(!(bot_device = deviceTable->GetDevice(this->bot_id))) {
    PLAYER_ERROR("unable to locate suitable bot device");
  }
  data = new player_pointcloud3d_data_t;
  //now I can access the fields in driver directly
  bot = ((UsBot*)bot_device->driver);
  bot->RegisterDriver((char*)"Laser3d", name, this);

  return;
}

////////////////////////////////////////////////////////////////////////////////
// Set up the device (called by server thread).
int UsLaser3d::Setup()
{
    bot->SubscribeDriver((char*)"Laser3d", name);
    PLAYER_MSG1(1,"subscribe %s", name);
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
UsLaser3d::~UsLaser3d()
{
    delete data;
    return;
}
////////////////////////////////////////////////////////////////////////////////
// ProcessMessages
int UsLaser3d::ProcessMessage(QueuePointer &resp_queue, player_msghdr *hdr, void *data)
{
    printf("laser3d hdr: %d : %d : %d : %d \n",hdr->addr.host,
            hdr->addr.robot,
            hdr->addr.interf,
            hdr->addr.index);
    printf("laser3d this: %d : %d : %d : %d \n",this->device_addr.host,
            this->device_addr.robot,
            this->device_addr.interf,
            this->device_addr.index);
    printf("type %d subtype %d not handled\n",hdr->type, hdr->subtype);
    fflush(stdout);
    return -1;
}

////////////////////////////////////////////////////////////////////////////////
// PublishNewData
void UsLaser3d::PublishNewData()
{
    Publish(device_addr, PLAYER_MSGTYPE_DATA, PLAYER_POINTCLOUD3D_CODE,
            data, sizeof(*data), NULL);
}

////////////////////////////////////////////////////////////////////////////////
// Shutdown the device (called by server thread).
int UsLaser3d::Shutdown()
{
  //fprintf(stderr,"UsLaser - Shutdown\n");
  bot->UnsubscribeDriver((char*)"Laser3d", this->name);
  return 0;
}
