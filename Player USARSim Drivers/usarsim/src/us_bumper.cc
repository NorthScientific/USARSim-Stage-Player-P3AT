#include "us_bumper.h"

////////////////////////////////////////////////////////////////////////////////
// Constructor
UsBumper::UsBumper(ConfigFile* cf, int section)    : Driver(cf, section, true, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_BUMPER_CODE)
{
    if (cf->ReadDeviceAddr(&this->bot_id, section, "requires",
                           PLAYER_SIMULATION_CODE, -1,NULL) != 0) {
        this->SetError(-1);
        return;
    }
    strncpy(this->name, cf->ReadString(section, "bumper_name", ""), sizeof(this->name));
    this->bumper_data = new player_bumper_data_t();
    this->geom = new player_bumper_geom_t();
    this->gotGeom = false;
    Device* bot_device;// bot device
    if(!(bot_device = deviceTable->GetDevice(this->bot_id)))
    {
      PLAYER_ERROR("unable to locate suitable bot device");
    }
    //now I can access the fields in driver directly
    bot = ((UsBot*)bot_device->driver);
    bot->RegisterDriver((char*)"Bumper", name, this);
    sleep(1);
    bot->RequestGeom((char*)"Bumper",(char*)"");
    return;
}

////////////////////////////////////////////////////////////////////////////////
// Set up the device (called by server thread).
int UsBumper::Setup()
{
  PLAYER_MSG0(2,"UsBumper - Setup\n");
  bot->SubscribeDriver((char*)"Bumper", name);
  PLAYER_MSG1(1,"subscribe Bumper %s", name);
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
UsBumper::~UsBumper()
{
  // Nothing to do...
  return;
}
/**
 *
 */
int UsBumper::ProcessMessage(QueuePointer &resp_queue, player_msghdr *hdr, void *data) {
    if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                             PLAYER_BUMPER_REQ_GET_GEOM,
                             this->device_addr)) {
        short count = 0;
        if(!gotGeom) {
            bot->RequestGeom((char*)"Bumper",name);
        }
        while (gotGeom != true && count < USBOT_STARTUP_CONN_LIMIT) {
            usleep(USBOT_DELAY_USEC);
            count++;
        }
        if (gotGeom != true) {
            PLAYER_ERROR("UsBumper NACK GEOM");
            this->Publish(this->device_addr,
                    PLAYER_MSGTYPE_RESP_NACK,
                    PLAYER_BUMPER_REQ_GET_GEOM);
            return -1;
        }
        this->Publish(this->device_addr,
                PLAYER_MSGTYPE_RESP_ACK,
                PLAYER_BUMPER_REQ_GET_GEOM,
                (void*)geom,
                sizeof(player_bumper_geom_t),NULL);
        return 0;
    } else {
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
// Shutdown the device (called by server thread).
int UsBumper::Shutdown()
{
  fprintf(stderr,"UsBumper - Shutdown\n");
  // free data for bot
  //bot->devices &= ~(US_DATA_FIDUCIAL);
  //free(bot->fiducial);
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// PublishNewData
void UsBumper::PublishNewData(){
    PLAYER_MSG0(1,"us_bumper new data\n");
    this->Publish(this->device_addr,
         PLAYER_MSGTYPE_DATA,
         PLAYER_BUMPER_DATA_STATE,
         (void*)bumper_data,sizeof(player_bumper_data_t),NULL);
}

