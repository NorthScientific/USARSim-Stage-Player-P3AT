#include "us_position3d.h"
// Initialization function
Driver* UsPosition3d_Init(ConfigFile* cf, int section)
{
    fprintf(stderr,"UsPosition3d - Init\n");
    return ((Driver*) (new UsPosition3d(cf, section)));
}

// a driver registration function
void UsPosition3d_Register(DriverTable* table)
{
    table->AddDriver("us_position3d", UsPosition3d_Init);
    return;
}

////////////////////////////////////////////////////////////////////////////////
// Constructor
UsPosition3d::UsPosition3d(ConfigFile* cf, int section) : Driver(cf, section, true, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_POSITION3D_CODE)
{
    PLAYER_MSG0(1,"UsPosition3d - Constructor\n");
    if (cf->ReadDeviceAddr(&this->bot_id, section, "requires",
          PLAYER_SIMULATION_CODE, -1,NULL) != 0)
    {
        this->SetError(-1);
        return;
    }
    return;
}


////////////////////////////////////////////////////////////////////////////////
// Destructor
UsPosition3d::~UsPosition3d()
{
    // Nothing to do...
    return;
}

////////////////////////////////////////////////////////////////////////////////
// Set up the device (called by server thread).
int UsPosition3d::Setup()
{
    PLAYER_MSG0(1,"UsPosition3d - Setup");
    // Find my bot
    Device* bot_device;
    if(!(bot_device = deviceTable->GetDevice(this->bot_id)))
    {
        PLAYER_ERROR("unable to locate suitable bot device");
        return(-1);
    }
    //now I can access the fields in driver directly
    bot = ((UsBot*)bot_device->driver);
    // Prepare data for bot
    data = new player_position3d_data_t();
    geom = new player_position3d_geom_t();

    bot->RegisterDriver((char*)"Position3d", (char*)"", this);
    bot->SubscribeDriver((char*)"Position3d", (char*)"");
    bot->RequestConf((char*)"Robot", (char*)"");
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// ProcessMessages
int UsPosition3d::ProcessMessage(QueuePointer &resp_queue, player_msghdr *hdr, void *data) {
    if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                             PLAYER_POSITION3D_REQ_GET_GEOM,
                             this->device_addr)) {
         int count = 0;
         PLAYER_MSG0(3,"POSITION REQ GEO\n");
         bot->RequestGeom((char*)"Robot",(char*)"");

         while (!bot->bGeoRobot == true && count <USBOT_STARTUP_CONN_LIMIT)
         {
             usleep(USBOT_DELAY_USEC);
             count++;
         }
         if (!bot->bGeoRobot == true)
         {
             PLAYER_MSG0(3,"POSITION REQ GEO NACK\n");
             this->Publish(this->device_addr,resp_queue,
                           PLAYER_MSGTYPE_RESP_NACK,
                           PLAYER_POSITION3D_REQ_GET_GEOM,
                           NULL,
                           sizeof(player_position3d_geom_t),NULL);
             return -1;
         }
         PLAYER_MSG0(3,"POSITION REQ GEO ACK\n");
         player_position3d_geom_t geom;
         geom.pose.px = 0;
         geom.pose.py = 0;
         geom.pose.pz = 0;
         geom.size.sl = bot->robotDimensions->sl;
         geom.size.sw = bot->robotDimensions->sw;
         geom.size.sh = bot->robotDimensions->sh;
         this->Publish(this->device_addr,resp_queue,
                       PLAYER_MSGTYPE_RESP_ACK,
                       PLAYER_POSITION3D_REQ_GET_GEOM,
                       (void*)&geom, sizeof(geom),NULL);
         return 0;
    }
    else {
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
        return -1;
    }
}

////////////////////////////////////////////////////////////////////////////////
// PublishNewData
void UsPosition3d::PublishNewData() {
    this->Publish(this->device_addr,
         PLAYER_MSGTYPE_DATA,
         PLAYER_POSITION3D_DATA_STATE,
         (void*)data,sizeof(player_position3d_data_t),NULL);
}

////////////////////////////////////////////////////////////////////////////////
// Shutdown the device (called by server thread).
int UsPosition3d::Shutdown()
{
  PLAYER_MSG0(1,"UsPosition3d - Shutdown");
  // @todo free data for bot
  return 0;
}
