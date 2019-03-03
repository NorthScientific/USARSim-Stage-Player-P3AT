#include "us_fiducial.h"

////////////////////////////////////////////////////////////////////////////////
// Constructor
UsFiducial::UsFiducial(ConfigFile* cf, int section)    : Driver(cf, section, true, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_FIDUCIAL_CODE)
{
    PLAYER_MSG0(1,"UsFiducial - Constructor\n");
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
UsFiducial::~UsFiducial()
{
  // Nothing to do...
  return;
}
/**
 *
 */
int UsFiducial::ProcessMessage(QueuePointer &resp_queue, player_msghdr *hdr, void *data) {
  if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
              PLAYER_FIDUCIAL_REQ_GET_GEOM,
              this->device_addr)) {

    player_fiducial_geom_t geom;
    geom.pose.px = 0;
    geom.pose.py = 0;
    geom.pose.pyaw = 0;
    geom.size.sl = 0.1;
    geom.size.sw = 0.1;
    geom.fiducial_size.sl = 0.1;
    geom.fiducial_size.sw = 0.1;
    this->Publish(this->device_addr,
         PLAYER_MSGTYPE_RESP_ACK,
         PLAYER_FIDUCIAL_REQ_GET_GEOM,
         &geom,
         sizeof(geom),NULL);
    return 0;
  }
  else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
            PLAYER_FIDUCIAL_REQ_GET_ID ,
                           this->device_addr)) {
    player_fiducial_id_t rfid;
    rfid.id = 0;
    this->Publish(this->device_addr,resp_queue,
         PLAYER_MSGTYPE_RESP_ACK,PLAYER_FIDUCIAL_REQ_GET_ID,
         &rfid, sizeof(rfid),NULL);
    return 0;
  }
  else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
            PLAYER_FIDUCIAL_REQ_SET_ID ,
                           this->device_addr)) {
    rfid.id = ((player_fiducial_id_t*)data)->id ;
    this->Publish(this->device_addr,resp_queue,
         PLAYER_MSGTYPE_RESP_ACK,PLAYER_FIDUCIAL_REQ_GET_ID,
         &rfid, sizeof(rfid),NULL);
    return 0;
  }
  else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
            PLAYER_FIDUCIAL_REQ_GET_FOV,
                           this->device_addr)) {
    this->Publish(this->device_addr,resp_queue,
         PLAYER_MSGTYPE_RESP_ACK,PLAYER_FIDUCIAL_REQ_GET_FOV,
         &fov, sizeof(fov),NULL);
    return 0;
  }
  else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
            PLAYER_FIDUCIAL_REQ_SET_FOV,
                           this->device_addr)) {
    this->Publish(this->device_addr,resp_queue,
         PLAYER_MSGTYPE_RESP_ACK,PLAYER_FIDUCIAL_REQ_SET_FOV,
         data, sizeof(data),NULL);
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
// Set up the device (called by server thread).
int UsFiducial::Setup()
{
  PLAYER_MSG0(2,"UsFiduical - Setup\n");
  Device* bot_device;
  if(!(bot_device = deviceTable->GetDevice(this->bot_id)))
  {
    PLAYER_ERROR("unable to locate suitable bot device");
    return(-1);
  }
  //now I can access the fields in driver directly
  bot = ((UsBot*)bot_device->driver);
  // Prepare data for bot
  fiducial_data = new player_fiducial_data_t();
  bot->devices |= US_DATA_FIDUCIAL;
  // Start the device thread
  return 0;
}


////////////////////////////////////////////////////////////////////////////////
// Shutdown the device (called by server thread).
int UsFiducial::Shutdown()
{
  fprintf(stderr,"UsFiducial - Shutdown\n");
  // free data for bot
  //bot->devices &= ~(US_DATA_FIDUCIAL);
  //free(bot->fiducial);
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// PublishNewData
void UsFiducial::PublishNewData(){
    PLAYER_MSG0(1,"us_fiducial new data\n");
    this->Publish(this->device_addr,
         PLAYER_MSGTYPE_DATA,
         PLAYER_FIDUCIAL_DATA_SCAN,
         (void*)fiducial_data,sizeof(player_fiducial_data_t),NULL);
}
