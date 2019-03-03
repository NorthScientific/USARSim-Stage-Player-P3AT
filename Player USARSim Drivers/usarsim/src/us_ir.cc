#include "us_ir.h"

///////////////////////////////////////////////////////////////////////////////
// Constructor
UsIR::UsIR(ConfigFile* cf, int section):ThreadedDriver(cf, section, true,
                       PLAYER_MSGQUEUE_DEFAULT_MAXLEN,
                       PLAYER_IR_CODE)
{
  fprintf(stderr,"UsIR - Constructor\n");
  if (cf->ReadDeviceAddr(&this->bot_id, section, "requires",
                         PLAYER_SIMULATION_CODE, -1,NULL) != 0)
  {
    this->SetError(-1);
    return;
  }
  strncpy(this->name, cf->ReadString(section, "ir_name", ""), sizeof(this->name));
  // init pointer to local data
  data = NULL;
  geom = NULL;

  Device* bot_device;// bot device
  if(!(bot_device = deviceTable->GetDevice(this->bot_id)))
  {
    PLAYER_ERROR("unable to locate suitable bot device");
  }
  //init mutexes
  pthread_mutex_init(&dataMutex, NULL);
  pthread_mutex_init(&geomMutex, NULL);

  //now I can access the fields in driver directly
  bot = ((UsBot*)bot_device->driver);
  PLAYER_MSG1(3,"us_ir register %s",name);
  bot->RegisterDriver((char*)"IR", name, this);

  return;
}

////////////////////////////////////////////////////////////////////////////////
// Set up the device (called by server thread).
int UsIR::Setup()
{
  bot->SubscribeDriver((char*)"IR", name);
  PLAYER_MSG1(1,"subscribe %s", name);
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
UsIR::~UsIR()
{
  // Nothing to do...
  delete data;
  delete geom;
  //delete conf;
  return;
}

////////////////////////////////////////////////////////////////////////////////
// Main
void UsIR::Main()
{
  while(true) {
    pthread_testcancel();
    usleep(USBOT_MAIN_LOOP_USEC);
    ProcessMessages();
    PublishNewData();
  }
}

////////////////////////////////////////////////////////////////////////////////
// ProcessMessages
int UsIR::ProcessMessage(QueuePointer &resp_queue, player_msghdr *hdr, void *data)
{
  if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                           PLAYER_IR_REQ_POSE,
                           this->device_addr)) {
    player_ir_pose_t* geom = GetGeom();
    if (geom == NULL)
    {
      PLAYER_ERROR("PutReply() failed");
      this->Publish(this->device_addr,
                    PLAYER_MSGTYPE_RESP_NACK,
                    PLAYER_IR_REQ_POSE,
                    data,sizeof(data),NULL);
      return -1;
    }
    pthread_mutex_lock(&geomMutex);
    this->Publish(this->device_addr,
                  PLAYER_MSGTYPE_RESP_ACK,
                  PLAYER_IR_REQ_POSE,
                  geom,
                  sizeof(*geom),NULL);
    pthread_mutex_unlock(&geomMutex);
    return 0;
  }
  return -1;
}

////////////////////////////////////////////////////////////////////////////////
// PublishNewData
void UsIR::PublishNewData()
{

  player_ir_data_t* data = GetData();
  //if new data available send it
  if (data != NULL ) //&& conf != NULL)
  {
    pthread_mutex_lock(&dataMutex);
    this->Publish(this->device_addr,
                  PLAYER_MSGTYPE_DATA,
                  PLAYER_IR_DATA_RANGES,
                  data,sizeof(*data),NULL);
    pthread_mutex_unlock(&dataMutex);
  }
  delete data;
}

////////////////////////////////////////////////////////////////////////////////
// Shutdown the device (called by server thread).
int UsIR::Shutdown()
{
  bot->UnsubscribeDriver((char*)"IR", this->name);
  return 0;
}

/**
 * Sets new ir data (called by US_BOT)
 *
 */
void UsIR::SetData(player_ir_data_t* data)
{
    PLAYER_MSG0(3,"us_ir setData");
    pthread_mutex_lock(&dataMutex);
    if(this->data != NULL) {
        delete this->data;
    }
    this->data = data;
    pthread_mutex_unlock(&dataMutex);
}

/**
 * Sets new geometry information (called by US_BOT)
 */
void UsIR::SetGeom(player_ir_pose_t* geom)
{
    PLAYER_MSG0(3,"us_ir GetGeom");
    pthread_mutex_lock(&geomMutex);
    if(this->geom != NULL) {
        delete this->geom;
    }
    this->geom = geom;
    pthread_mutex_unlock(&geomMutex);
}

/**
 * Returns pointer to most up to date ir data object. May return NULL
 * if no data is available since last method call.
 */
player_ir_data_t* UsIR::GetData()
{
  pthread_mutex_lock(&dataMutex);
  player_ir_data_t* curdata = data;
  data = NULL;
  pthread_mutex_unlock(&dataMutex);
  return curdata;
}

/**
 * Returns pointer to geometry data.
 */
player_ir_pose_t* UsIR::GetGeom()
{
  PLAYER_MSG0(3,"us_ir GetGeom");
  // no geom info available -> bot must fetch it
  if(geom == NULL)
  {
    int retry_count = 0;
    int sleep_count = 0;

    while(geom == NULL && retry_count < 10)
    {
      bot->RequestGeom((char*)"IR", name);
      // TODO: we need constants for sleep and retry limit
      while(geom == NULL && sleep_count < 1000)
      {
        sleep_count++;
        usleep(1000);
      }
      retry_count++;
      sleep_count = 0;
    }
    assert(geom != NULL);
  }
  return geom;
}
