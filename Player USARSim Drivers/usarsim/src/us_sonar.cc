#include "us_sonar.h"

////////////////////////////////////////////////////////////////////////////////
// Constructor
UsSonar::UsSonar(ConfigFile* cf, int section)  :Driver(cf, section, true, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_SONAR_CODE)
{
    if (cf->ReadDeviceAddr(&this->bot_id, section, "requires",
                           PLAYER_SIMULATION_CODE, -1,NULL) != 0) {
        this->SetError(-1);
        return;
    }
    strncpy(this->name, cf->ReadString(section, "sonar_name", ""), sizeof(this->name));
    this->sonar_data = new player_sonar_data_t();
    this->sonar_data->ranges = new float[this->sonar_data->ranges_count];
    this->geom = new player_sonar_geom_t();
    this->geom->poses = NULL;
    this->gotGeom = false;
    Device* bot_device;// bot device
    if(!(bot_device = deviceTable->GetDevice(this->bot_id)))
    {
      PLAYER_ERROR("unable to locate suitable bot device");
    }
    //now I can access the fields in driver directly
    bot = ((UsBot*)bot_device->driver);
    bot->RegisterDriver((char*)"Sonar", name, this);
    sleep(1);
    bot->RequestGeom((char*)"Sonar",(char*)"");
    return;
}

////////////////////////////////////////////////////////////////////////////////
// Set up the device (called by server thread).
int UsSonar::Setup()
{
    bot->SubscribeDriver((char*)"Sonar", name);
    PLAYER_MSG1(1,"subscribe Sonar %s", name);
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
UsSonar::~UsSonar()
{
    // Nothing to do...
    return;
}

////////////////////////////////////////////////////////////////////////////////
//
int UsSonar::ProcessMessage(QueuePointer &resp_queue, player_msghdr *hdr, void *data) {
    if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                             PLAYER_SONAR_REQ_GET_GEOM,
                             this->device_addr)) {
        short count = 0;
        if(!gotGeom) {
            bot->RequestGeom((char*)"Sonar",name);
        }
        while (gotGeom != true && count < USBOT_STARTUP_CONN_LIMIT) {
            usleep(USBOT_DELAY_USEC);
            count++;
        }
        if (gotGeom != true) {
            PLAYER_ERROR("Us SONAR NACK GEOM");
            this->Publish(this->device_addr,
                    PLAYER_MSGTYPE_RESP_NACK,
                    PLAYER_SONAR_REQ_GET_GEOM);
            return -1;
        }
        this->Publish(this->device_addr,
                PLAYER_MSGTYPE_RESP_ACK,
                PLAYER_SONAR_REQ_GET_GEOM,
                (void*)geom,
                sizeof(player_sonar_geom_t),NULL);
        		
        return 0;
    }
    else {
        printf("sonar hdr: %d : %d : %d : %d \n",hdr->addr.host,
                hdr->addr.robot,
                hdr->addr.interf,
                hdr->addr.index);
        printf("sonar this: %d : %d : %d : %d \n",this->device_addr.host,
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
void UsSonar::PublishNewData() {
    //cout<<"sonar rangesCount "<<mySonarData.ranges_count<<endl;
    this->Publish(this->device_addr,
         PLAYER_MSGTYPE_DATA,
         PLAYER_SONAR_DATA_RANGES,
         (void*)sonar_data,sizeof(player_sonar_data_t),NULL);
}

////////////////////////////////////////////////////////////////////////////////
// Shutdown the device (called by server thread).
int UsSonar::Shutdown()
{
  fprintf(stderr,"UsSonar - Shutdown\n");
  bot->UnsubscribeDriver((char*)"Sonar", name);
  return 0;
}

