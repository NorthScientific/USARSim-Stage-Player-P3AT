/*
 * Desc: Libraray for parsing Gamebots messages
 * Author: Jijun Wang
 * Date: 13 MAY 2004
// Modified:
// 3 Mars 2005 	   Erik Winter 	added Ringhorne IR
// 11 Mars 2005    Erik Winter 	added RinghornePyro
// 14 Mars 2005    Erik Winter 	implemented rh_get_pyro, rh_get_pyro_geom
// 14 Mars 2005    Erik Winter 	added RHPyro config, implemented rh_get_pyro_config
// 14 Mars 2005    Erik Winter Started porting USARSim to Player1.6
// 15 Mars 2005    Erik Winter Continued porting, it compiles but gives segmentation faults
// 18 Mars 2005    Erik Winter Changed ir_geom from player_ir_pose_req_t to player_ir_pose__t in the 1.6 version
// 07 June 2005    Bulit in support for INU
// 20 Nov 2006     Nils Kehrein Added all-at-once parsing for laser config/geom
// 22 Nov 2006     Florian Halbritter Added support for UsarSim RangeScanner3d (providing pointcloud3d data).
 */

#include "us_parser.h"

/**
 *
 */
// Define the indices of horizontal and vertical angles in the arrays
// for nicer handling later on.
#define HORIZ 1
#define VERT 0

int us_get_word(char* data, int pos, char* word)
{
  char *p, *pNext;
  int next;

  if (data == NULL || pos < 0 || pos >= (int)strlen(data)) return -1;
  for (p = data + pos;*p == ' ';p++);
  if ((pNext = strchr(p,' '))==NULL) next = strlen(data);
  else next = pNext - p;
  if (word!=NULL)
  {
    // trying to avoid buffer overflows:
    strncpy(word,p,next);
    word[next] = '\0';
  }
  return next + pos;
}
/**
 * get the pointer to the message body
 */
char* us_get_body(char* data)
{
  if (data==NULL) return NULL;
  return data + us_get_word(data,0,NULL);
}
/**
 * get the first data segment that is between "{" and "}" from the position pos.
 * TODO use memchr here
 */
int us_get_segment(char* data, int pos, char* segment)
{
  char *p1, *p2;
  char *tmp=(char*)"";
  if (data==NULL || pos<0 || pos>= (int)strlen(data)) return -1;
  if ((p1 = strchr(data+pos,'{'))==NULL) return -1;
  p1 += 1;
  if ((p2 = strchr(p1,'}'))==NULL) return -1;
  if (segment!=NULL)
  {
    // trying to avoid buffer overflows
    tmp[p2-p1+1]='\0';
    strncpy(tmp,p1,p2-p1);
  }
  segment=tmp;
  return p2-data+1;
}
/**
 * get the first data segment starts with name from the position pos.
 */
int us_get_segmentByName(char* data, int pos, char* name, char* segment)
{
  char *p1, *p2;
  char tmpStr[128];
  if (data == NULL ||
     name == NULL ||
     pos < 0 ||
     pos >= (int)strlen(data) ) return -1;
  tmpStr[0]='{';
  strncpy(tmpStr+1,name,sizeof(tmpStr));
  if ((p1 = strstr(data+pos,tmpStr)) == NULL) return -1;
  p1 += 1;
  if ((p2 = strchr(p1,'}')) == NULL) return -1;
  if (segment!=NULL)
  {
    //trying to avoid buffer overflows:
    strncpy(segment,p1,p2-p1);
    segment[p2-p1]='\0';
  }
  return p2-data+1;
}
/**
 * get the name value pair in a segment.
 */
int us_get_value(char* segment, char* name, char* value)
{
  char *p;
  if (segment==NULL || name==NULL) return -1;
  if ((p = strstr(segment,name))==NULL) return -1;

  return us_get_word(segment,p+strlen(name)-segment,value);
}

// get the first name value pair in a message.
int us_get_value2(char* data, char* name, char* value)
{
  char *p1, *p2;
  char tmpStr[128];
  int pos = 0;
  if (data == NULL || name == NULL) return -1;
  tmpStr[0]='{';
  //strncpy(tmpStr+1,name,sizeof(tmpStr));
  strcpy(tmpStr+1,name);
  if ((p1 = strstr(data + pos,tmpStr)) == NULL) return -1;
  p1 += strlen(tmpStr);
  if ((p2 = strchr(p1,' ')) == NULL) return -1;
  pos = us_get_word(data,p2-data+1,value);
  if(pos > 1 && pos < 4096) {
    if (data[pos-1] == '}')
     {
       pos-=1;
       value[strlen(value)-1] = 0;
     }
    return pos;
  }
  else {
    PLAYER_MSG2(1,"!!!!!!!!!!!!!!!!!!!!!!!!!!!error  pos %d data length %d\n",pos,strlen(data));
  }
  return -1;
}

/**
 * get the availabe data type and message body pointer
 */
int us_get_type_and_name(char* data, char** ppBody, string &name)
{
    char value[128];
    if (data==NULL) return -1;
    if(us_get_value2(data, (char*)"Name",value)>0) name.assign(value);
    else return -1; //TODO is there always a name?
  char head[16];
  head[0] = '\0';
  for(int i = 1; i < 16; i++)head[i] = 'A';
  int res = 0;
  if (data==NULL) return -1;
  *ppBody = data + us_get_word(data,0,head);
  if (!strcmp(head,"SEN"))
  {
    //sscanf(*ppBody,"{Type %s}",type);

    //PLAYER_MSG1(5,"us_parser:SEN %s\n",*ppBody);
    if (strstr(*ppBody,"{Type Sonar}")!=NULL) res |= US_DATA_SONAR;
    else if (strstr(*ppBody,"{Type RangeScanner}")!=NULL) res |= US_DATA_LASER|US_DATA_RANGER;
    else if (strstr(*ppBody,"{Type RangeScanner3D}")!=NULL || strstr(*ppBody,"{Type 3DRangeScanner}")!=NULL) res |= US_DATA_LASER3D;
    else if (strstr(*ppBody,"{Type IR}")!=NULL) res |= US_DATA_IR;
    else if (strstr(*ppBody,"{Type INU}")!=NULL) res |= US_DATA_INU;
    else if (strstr(*ppBody,"{Type Odometry}")!=NULL) res |= US_DATA_POSITION;
    else if (strstr(*ppBody,"{Type Encoder}")!=NULL) res |= US_DATA_ENCODER;
    else if (strstr(*ppBody,"{Type RFID}")!=NULL) res |= US_DATA_FIDUCIAL;
    else if (strstr(*ppBody,"{Type VictRFID}")!=NULL) res |= US_DATA_VICTIM_FIDUCIAL;
    else if (strstr(*ppBody,"{Type GroundTruth}")!=NULL) res |= US_GROUND_TRUTH;
    else if (strstr(*ppBody,"{Type Touch}")!=NULL) res |= US_DATA_BUMPER;
  }
  else if (!strcmp(head,"GEO"))
  {
    PLAYER_MSG1(5,"us_parser:GEO %s\n",*ppBody);
    if (strstr(*ppBody,"{Type Sonar}")!=NULL) res |= US_GEOM_SONAR;
    else if (strstr(*ppBody,"{Type RangeScanner}")!=NULL) res |= US_GEOM_LASER|US_GEOM_RANGER;
    else if (strstr(*ppBody,"{Type RangeScanner3D}")!=NULL || strstr(*ppBody,"{Type 3DRangeScanner}")!=NULL) res |= US_GEOM_LASER3D;
    else if (strstr(*ppBody,"{Type IR}")!=NULL) res |= US_GEOM_IR;
    else if (strstr(*ppBody,"{Type Camera}")!=NULL) res |= US_DATA_PTZ;
    else if (strstr(*ppBody,"{Type GroundVehicle}")!=NULL) res |= US_GEOM_ROBOT;
    else if (strstr(*ppBody,"{Type Touch}")!=NULL) PLAYER_MSG0(1,"GEOM_BUMPER");
  }

  else if (!strcmp(head,"MIS"))
  {
    //PLAYER_MSG1(5,"us_parser:MIS %s",*ppBody);
    if (strstr(*ppBody,"{Type PanTilt}")!=NULL) res |= US_GEOM_PTZ|US_DATA_PTZ; //for location geom and pan tilt data
  }
  else if (!strcmp(head,"CONF"))
  {
    PLAYER_MSG1(5,"us_parser:CONF %s",*ppBody);
    if (strstr(*ppBody,"{Type Sonar}")!=NULL) res |= US_CONF_SONAR;
    else if (strstr(*ppBody,"{Type RangeScanner}")!=NULL) res |= US_CONF_LASER|US_CONF_RANGER;
    else if (strstr(*ppBody,"{Type RangeScanner3D}")!=NULL || strstr(*ppBody,"{Type 3DRangeScanner}")!=NULL) res |= US_CONF_LASER3D;
    else if (strstr(*ppBody,"{Type IR}")!=NULL) res |= US_CONF_IR;
    else if (strstr(*ppBody,"{Type Camera}")!=NULL) res |= US_CONF_CAMERA;  //for ptz zoom
    else if (strstr(*ppBody,"{Type GroundVehicle}")!=NULL) res |= US_CONF_ROBOT;  //for ptz zoom
  }
  else if (!strcmp(head,"STA"))
  {
    //PLAYER_MSG1(6,"us_parser:STA %s",*ppBody);
    //res |= US_STATUS;
  }
  else res = -1;
  return res;
}
/**
 *
 */
int us_get_position(char* data, player_position2d_data_t *position)
{
  char tmp[128];

  static float old_yaw = 0, oldxpos = 0, oldypos = 0;
  static struct timeval old_time = {0,0};
  struct timeval time;
  float xpos, ypos, yaw, xspeed, yspeed;
  double time_diff;

  if (position == NULL || data == NULL) return -1;
  if (us_get_value2(data, (char*)"Pose", tmp)==-1) return -1;
  sscanf(tmp, "%f,%f,%f", &xpos, &ypos, &yaw);
  position->pos.px = xpos;
  position->pos.py = (-1.0) * ypos;
  position->pos.pa = (-1.0) * yaw;
  // calculate speeds
  gettimeofday(&time, NULL);
  time_diff = (double)(time.tv_sec - old_time.tv_sec) +
              (double)(time.tv_usec - old_time.tv_usec)/1000000;
  position->vel.pa = (yaw - old_yaw) / time_diff;
  xspeed = (xpos - oldxpos)/time_diff;
  yspeed = (ypos - oldypos)/time_diff;
  // TODO: kompletter bloedsinn?
  position->vel.px = sqrt(pow(xspeed,2)+pow(yspeed,2));
  position->vel.py = 0;
  position->stall = 0;

  oldxpos = xpos;
  oldypos = ypos;
  old_yaw = yaw;
  old_time = time;

  return 0;
}

/**
 *
 */
int us_get_groundTruthPos(char* data, player_position2d_data_t *pose)
{

      char tmp[128];
      float xpos, ypos, yaw;
      if (us_get_value2(data, (char*)"Location", tmp)==-1) {
    	  cout<<data<<endl;
    	  return -1;
      }
      sscanf(tmp, "%f,%f,%*f", &xpos, &ypos);
      if (us_get_value(data,(char*)"Orientation",tmp) == -1) return -1;
      sscanf(tmp, "%*f,%*f,%f", &yaw);
      pose->pos.px = xpos;
      pose->pos.py = (-1.0) * ypos;
      pose->pos.pa = (-1.0) * yaw;
      if(pose->pos.pa < -M_PI ) pose->pos.pa += (2.0 * M_PI);

      return 0;
}

/**
 *
 */
int us_get_groundTruth(char* data, player_localize_data_t *location)
{
  char tmp[128];
  struct timeval time;
  float xpos, ypos, yaw;
  //PLAYER_MSG1(1,"us_get_groundTruth data %s",data);
  if (us_get_value2(data, (char*)"Location", tmp)==-1) return -1;
  sscanf(tmp, "%f,%f,%*f", &xpos, &ypos);
  if (us_get_value(data,(char*)"Orientation",tmp) == -1) return -1;
  sscanf(tmp, "%*f,%*f,%f", &yaw);
  location->pending_count = 0;
  location->hypoths_count = 1;
  gettimeofday(&time, NULL);
  location->pending_time = time.tv_sec;
  location->hypoths[0].mean.px = xpos;
  location->hypoths[0].mean.py = (-1.0) * ypos;
  location->hypoths[0].mean.pa = (-1.0) * yaw;

  if(location->hypoths[0].mean.pa < -M_PI )
  {
    location->hypoths[0].mean.pa += (2 * M_PI);
  }

  // zero covariance and max weight
  location->hypoths[0].cov[0] = 0;
  location->hypoths[0].cov[1] = 0;
  location->hypoths[0].cov[2] = 0;

  location->hypoths[0].alpha = 1e6;
  return 0;
}

/**
 *
 */
int us_get_inu(char* data, player_position3d_data_t *inu){
  static struct timeval old_time;
  struct timeval time;
  static double oldyaw = 0;
  char value[64];
  float pitch, roll, yaw;
  int p;
  double time_diff;

  if (!inu || !data) return -1;
  gettimeofday(&time, NULL);
  p = us_get_value2(data, (char*)"Orientation", value);
  if (p == -1) return -1;

  p = sscanf(value, "%f,%f,%f", &roll, &pitch, &yaw);
  if(p < 3){
    return -1;
  }
  roll *= 1000;
  pitch *= -1000;
  yaw *= -1000;

  inu->pos.proll = roll;
  inu->pos.ppitch = pitch;
  inu->pos.pyaw = yaw;

  //Calculate yawspeed
  time_diff = (double)(time.tv_sec - old_time.tv_sec) +
              (double)(time.tv_usec - old_time.tv_usec)/1000000;
  // to make both yaw and oldyaw have the same sign
  if((yaw*oldyaw)<0){
    if(yaw>3)
   oldyaw=oldyaw+2*M_PI*1000;
    else{
   if(yaw<-3)
     oldyaw=oldyaw-2*M_PI*1000;
    }
  }
  inu->vel.pyaw = (yaw-oldyaw)/time_diff;
  inu->vel.proll = 0;
  inu->vel.ppitch = 0;
  old_time = time;
  oldyaw = yaw;
  return 0;
}
/**
 *
 */
int us_get_sonar(char* data,const char* name, player_sonar_data_t *sonar)
{

    if (data == NULL || name == NULL || sonar == NULL) return -1;
    string sub, s(data);
    char dummyName[128];

    snprintf(dummyName,128,"{Name %s",name);
    //cout<<"dummyName "<<dummyName<<endl;
    string::size_type pos, pos2;
    sonar->ranges_count = 0;
    pos = s.find(dummyName,0);
    while (pos != string::npos) {
        pos2 = s.find(dummyName,pos+1);
        sub = s.substr(pos,pos2-pos);
        sscanf(sub.c_str(),"{Name %*s Range %f",&(sonar->ranges[sonar->ranges_count]));
        sonar->ranges_count++;
        pos = pos2;
    }
    return 0;
}
/**
 *
 */
int us_get_ir(char* data, char* name, player_ir_data_t *ir )
{
  char seg[128];
  char dummyName[128];
  char val[64];
  int pos = 0;
  short count = 0;
  char *p = data;
  char tmp[7];

  if (ir == NULL) return -1;
  sprintf(dummyName,"Name %s",name);
  ir->ranges = new float[ir->ranges_count];
  while ((pos = us_get_segmentByName(p,pos,dummyName,seg)) > 0)
  {
    if (us_get_value(seg,(char*)"Range",val) == -1) return -1;
    strncpy(tmp,val,7);
    ir->ranges[count] = atof(tmp);
    count++;
    ir->ranges_count = count;
  }
  return 0;
}

/**
 *
 */
int us_get_laser(char* data, player_laser_data_t *laser)
{
  char ranges[US_MAX_MSG_LEN], *p1, *p2;
  int count = 0;
  static int laser_id = 0;

  if(data == NULL || laser == NULL) return -1;

  if (us_get_value2(data, (char*)"Range", ranges) == -1) return -1;
  p1 = ranges;
  while ((p2 = strchr(p1,','))>0)
  {
    *p2 = 0;
    laser->intensity[count] = 0; // right now we have no intensity info
    laser->ranges[count++] = atof(p1);
    p1 = p2+1;
  }
  laser->intensity[count] = (uint8_t)128;
  laser->intensity_count = 0;
  laser->ranges[count++] = atof(p1);
  laser->ranges_count = count;
  laser->id = laser_id;
  laser_id++;

  return 0;
}

/**
 *
 */
int us_get_ranger(char* data, player_ranger_data_range_t *ranger)
{
  char ranges[US_MAX_MSG_LEN], *p1, *p2;
  int count = 0;
  if(ranger == NULL || data == NULL) return -1;
  if (us_get_value2(data, (char*)"Range", ranges) == -1) return -1;
  p1 = ranges;
  while ((p2 = strchr(p1,','))>0)
  {
    *p2 = 0;
    ranger->ranges[count++] = atof(p1);
    p1 = p2+1;
  }
  ranger->ranges[count++] = atof(p1);
  ranger->ranges_count = count;
   return 0;
}


/**
 *
 */
int us_get_ptz(char* data,char* name, player_ptz_data_t *ptz,player_ptz_geom_t *ptz_geom)
{
  /*
  char dummyName[128];
  char tmp[128];

  char seg[256];
  char val[128];
  int pos = 0;
  short count = 0;
  char *p = data, *p1, *p2;


  if (ptz==NULL) return -1;
  us_get_value(data,"Name",dummyName);
  if(strstr(dummyName,name) == NULL) return -1;
  sprintf(dummyName,"Name %s",name);

  if (us_get_value2(data, "Camera", tmp)==-1) return -1;
  if ((p2 = strchr(tmp,','))==NULL) return -1;
  *p2 = 0;
  ptz->tilt = atof(tmp);
  //fprintf(stderr,"T=%s|%d ",tmp, (int16_t)(atof(tmp)*US_UU_DEG));
  p1 = p2+1;
  if ((p2 = strchr(p1,','))==NULL) return -1;
  *p2 = 0;
  ptz->pan = (-1.0) * atof(p1);
  //fprintf(stderr,"P=%s|%d\n",p1, (int16_t)(atof(p1)*US_UU_DEG));
  if (us_get_value2(data, "CameraVel", tmp)==-1) return -1;
  if ((p2 = strchr(tmp,','))==NULL) return -1;
  *p2 = 0;
  ptz->tiltspeed = atof(tmp);
  p1 = p2+1;
  if ((p2 = strchr(p1,','))==NULL) return -1;
  *p2 = 0;
  ptz->panspeed = (-1.0) * atof(p1);


  while ((pos = us_get_segmentByName(p,pos,dummyName,seg))>0)
  {
  //printf("%s",p);
    if (ptz_geom==NULL) return -1;
    //if ((pos =us_get_segmentByName(p,0,dummyName,seg))==-1) return -1;

    if (us_get_value(seg,"Part CameraBase Location",val)==-1) return -1;
    if ((p2 = strchr(val,','))==NULL) return -1;
    *p2 = 0;
    ptz_geom->pos.px = atof(val);
    p1 = p2+1;
    if ((p2 = strchr(p1,','))==NULL) return -1;
    *p2 = 0;
    ptz_geom->pos.py = (-1.0) * atof(p1);
    p1 = p2+1;
    ptz_geom->pos.pz = (-1.0) * atof(p1);

    //fprintf(stderr,"sonar# %d\n",ntohs(sonar_geom->pose_count));
    //usarsim doesn't provide information about the size of sensors
    ptz_geom->size.sw = 0.1;
    ptz_geom->size.sl = 0.1;
    ptz_geom->size.sh = 0.1;
  }

  */
  ptz->tilt = 0.1;
  ptz->pan = 0.1;
  ptz->tiltspeed = 0.1;
  ptz->panspeed = 0.1;
  ptz_geom->pos.px = 0.0;
  ptz_geom->pos.py = 0.0;
  ptz_geom->pos.pz = 0.0;
  ptz_geom->size.sw = 0.1;
  ptz_geom->size.sl = 0.1;
  ptz_geom->size.sh = 0.1;
  return 0;
}
/**
 *
 */
int us_get_sonar_geom_all(char* data, map<string, player_sonar_geom_t*> mGeom)
{
	// count the number of sensors (occurances of '{' in data -1 / 2)
    int count = 0, k = 0;
    while (data[k] != '\0')
    {
          if (data[k] == '{')
              count++;
          k++;
    }
    count--;
    count /= 2.0;

    map<string,player_sonar_geom_t*>::iterator iter;
    for(iter = mGeom.begin(); iter != mGeom.end();iter++) {
    	iter->second->poses = new player_pose3d_t[count];
    }

	//PLAYER_MSG2(1,"Data %s Number of Sensors %d",data, count);

    string d, name, dummyName;
    char c;
    int index = -1;
    if(data == NULL) return -1;
    string dummy(data);
    stringstream s(dummy, stringstream::in | stringstream::out);

    //{Type RangeScanner}
    s >> d >> d;
    while(s.good()){
        //{Name ScannerV
        s >> d >> name;

        if(strstr(name.c_str(),dummyName.c_str())==0){
            index = -1;
        }
        map<string,player_sonar_geom_t*>::iterator iter;
        for(iter = mGeom.begin(); iter != mGeom.end();iter++) {
        	//cout << iter->first.c_str() << endl;
            if(strstr(name.c_str(), iter->first.c_str())!=0){
                index++;
                dummyName = iter->first;
                break;
            }
        }
        if(iter == mGeom.end()) return -1;
        if(s.eof())break;
        //  Location         0.3801                         ,
        s >>    d   >> mGeom[dummyName]->poses[index].px >> c
                    >> mGeom[dummyName]->poses[index].py >> c
                    >> mGeom[dummyName]->poses[index].pz;
        // Orientation              3.1415                      ,
        s >>    d   >> mGeom[dummyName]->poses[index].proll  >> c
                    >> mGeom[dummyName]->poses[index].ppitch >> c
                    >> mGeom[dummyName]->poses[index].pyaw;
        //  Mount HARD}
        s >> d   >> d;
        mGeom[dummyName]->poses[index].py *= -1.0;
        mGeom[dummyName]->poses[index].proll *= -1.0;
        mGeom[dummyName]->poses[index].ppitch *= -1.0;
        mGeom[dummyName]->poses[index].pyaw *= -1.0;
        mGeom[dummyName]->poses_count = (index+1);

//          PLAYER_MSG5(1,"name %s pose[%d] (%f, %f, %f)",name.c_str(),index,
//                  mGeom[dummyName]->poses[index].px ,
//                  mGeom[dummyName]->poses[index].py ,
//                  mGeom[dummyName]->poses[index].pz);
//          PLAYER_MSG3(1,"orientation (%f, %f, %f)", mGeom[dummyName]->poses[index].ppitch,
//                  mGeom[dummyName]->poses[index].proll,
//                  mGeom[dummyName]->poses[index].pyaw);

    }
    return 1;
}
/**
 *
 */
int us_get_camera_config(char* data,char* name, player_ptz_data_t *ptz)
{
  char val[128];
  char dummyName[128];

  us_get_value(data,(char*)"Name",dummyName);
  if(strstr(dummyName,name) == NULL) return -1;
  //todo CameraFov not CameraDefFov
  if (us_get_value2(data,(char*)"CameraDefFov",val)==-1) return -1;
  ptz->zoom = atof(val);
  return 0;
}
/**
 *
 */
int us_get_ir_geom(char* data, player_ir_pose_t* ir_geom,char* name)
{
  // count the number of sensors (occurances of '{' in data -1 / 2)
  int cnt = 0, k = 0;
  while (data[k] != '\0')
  {
        if (data[k] == '{')
            cnt++;
        k++;
  }
  cnt--;
  //count /= 2.0;

  //PLAYER_MSG2(1,"Data %s Number of Sensors %d",data, cnt);

  char seg[256], val[128];
  char dummyName[128];
  int pos = 0;
  short count = 0;
  char *p = data;
  float dummy1, dummy2;
  if (ir_geom==NULL) return -1;
  sprintf(dummyName,"Name %s",name);
  ir_geom->poses = new player_pose3d_t[cnt];
  while ((pos = us_get_segmentByName(p,0,dummyName,seg))>0)
  {
    if (us_get_value(seg,(char*)"Location",val) == -1) return -1;
    sscanf(val, "%f,%f,%*f", &dummy1, &dummy2);
    ir_geom->poses[count].px = dummy1;
    ir_geom->poses[count].py = (-1.0) * dummy2;
    if (us_get_value(seg,(char*)"Orientation",val)==-1) return -1;
    sscanf(val, "%*f,%*f,%f", &dummy1);
    ir_geom->poses[count].pyaw = (-1.0) * dummy1;

    count++;
    p += pos;
    ir_geom->poses_count = count;
    PLAYER_MSG4(8,"name %s px %f py%f pa %f",dummyName, ir_geom->poses[count].px,
                ir_geom->poses[count].py, ir_geom->poses[count].pyaw);
  }

  return 0;
}

/*
 *
 */
int us_get_encoder(char* data, map<string, int> &mEnc)
{
    string d, name;
    char c;

    if(data == NULL) return -1;
    string dummy(data);
    stringstream s(dummy, stringstream::in | stringstream::out);
    //{Type RangeScanner}
    s >> d >> d;
    while(s.good()){
        //{Name ECLeft
        s >> d >> name;
        //   Tick         0          }
        s >>    d   >> mEnc[name] >> c;
        //PLAYER_MSG2(0,"name %s tick %d",name.c_str(), mEnc[name]);
    }
    return 1;
}

/*
 * Parses laser geometry for several lasers at once
 */
int us_get_laser_geom_all(char* data, map<string, player_laser_geom_t*> mGeom)
{
    string d, name;
    char c;

    cout << "Laser DATA: " << data << endl;

    if(data == NULL) return -1;
    string dummy(data);
    stringstream s(dummy, stringstream::in | stringstream::out);
    //{Type RangeScanner}
    s >> d >> d;
    while(s.good()){
        //{Name ScannerV
        s >> d >> name;

        if(s.eof())break;
        //  Location         0.3801            ,          0.2898            ,          -0.1140
        s >>    d   >> mGeom[name]->pose.px >> c >> mGeom[name]->pose.py >> c >> mGeom[name]->pose.pz;
        // Orientation              3.1415         ,             0.0000            ,          0.7852
        s >>    d   >> mGeom[name]->pose.proll >> c >> mGeom[name]->pose.ppitch >> c >> mGeom[name]->pose.pyaw;
        //  Mount HARD}
        s >> d   >> d;
        mGeom[name]->size.sw = 0.1;
        mGeom[name]->size.sl = 0.1;
        mGeom[name]->pose.py *= -1.0;
        mGeom[name]->pose.pz *= -1.0;
        mGeom[name]->pose.proll *= -1.0;
        mGeom[name]->pose.ppitch *= -1.0;
        mGeom[name]->pose.pyaw *= -1.0;
        PLAYER_MSG7(1,"name %s pose (%f, %f, %f) orientation (%f, %f, %f)",name.c_str(), mGeom[name]->pose.px ,
                mGeom[name]->pose.py ,mGeom[name]->pose.pz, mGeom[name]->pose.ppitch, mGeom[name]->pose.proll, mGeom[name]->pose.pyaw);
    }
    return 1;
}

/*
 * Parses laser geometry for several lasers at once
 */
int us_get_ranger_geom_all(char* data, map<string,player_ranger_geom_t*> mGeom)
{
    string d, name;
    char c;

    if(data == NULL) return -1;
    string dummy(data);
    stringstream s(dummy, stringstream::in | stringstream::out);
    //{Type RangeScanner}
    s >> d >> d;
    while(!s.eof()){
        //{Name ScannerV
        s >> d >> name;
        if(s.eof())break;
        if(mGeom[name] == NULL) return -1;
        //  Location         0.3801            ,          0.2898            ,          -0.1140
        s >>    d   >> mGeom[name]->pose.px >> c >> mGeom[name]->pose.py >> c >> mGeom[name]->pose.pz;
        // Orientation              3.1415         ,             0.0000            ,          0.7852
        s >>    d   >> mGeom[name]->pose.proll >> c >> mGeom[name]->pose.ppitch >> c >> mGeom[name]->pose.pyaw;
        //  Mount HARD}
        s >> d   >> d;
        mGeom[name]->size.sw = 0.1;
        mGeom[name]->size.sl = 0.1;
        mGeom[name]->size.sh = 0.1;
        mGeom[name]->pose.py *= -1.0;
        mGeom[name]->pose.pz *= -1.0;
        mGeom[name]->pose.proll *= -1.0;
        mGeom[name]->pose.ppitch *= -1.0;
        mGeom[name]->pose.pyaw *= -1.0;
        PLAYER_MSG7(3,"ranger name %s pose (%f, %f, %f) orientation (%f, %f, %f)",name.c_str(), mGeom[name]->pose.px ,
                mGeom[name]->pose.py ,mGeom[name]->pose.pz, mGeom[name]->pose.ppitch, mGeom[name]->pose.proll, mGeom[name]->pose.pyaw);

        // Prepare some space for storing geometry data - the parent class will clean this up when necessary
        mGeom[name]->sensor_poses_count = 1;

        if ((mGeom[name]->sensor_poses = new player_pose3d_t() ) == NULL) {
            PLAYER_ERROR ("Failed to allocate memory for sensor poses");
            return -1;
        }
        if ((mGeom[name]->sensor_sizes = new player_bbox3d_t() ) == NULL) {
            PLAYER_ERROR ("Failed to allocate memory for sensor sizes");
            delete mGeom[name]->sensor_sizes;
            mGeom[name]->sensor_sizes = NULL;
            return -1;
        }
        *(mGeom[name]->sensor_poses)= (mGeom[name]->pose);
        *(mGeom[name]->sensor_sizes)= (mGeom[name]->size);
        mGeom[name]->sensor_sizes_count = 1;
  }
  return 1;
}
/*
 * Parses laser config information for several lasers at once
 *
 * Only makes sense if command 'GETCONF {Type RangeScanner}' was sent
 * without any 'Name' field
 */
int us_get_ranger_config_all(char* data, map<string, player_ranger_config_t*> mConf)
{
  char val[128];
  char *p = data;
  float fov;
  char dummy[256];
  //if (mConf == NULL) return -1;

  cout << data << endl;

  // search for Name fields
  while ((p = strstr(p, "{Name")) != NULL)
  {
    us_get_value2(p, (char*)"Name", dummy);
    if(strlen(dummy) == 0) {
      return -2;
    }
    string name(dummy);
    if(mConf.find(name) == mConf.end())  {p += 5; continue;}
    if (us_get_value2(p, (char*)"MaxRange", val)==-1) return -3;
    mConf[name]->max_range = atof(val);

    if (us_get_value2(p, (char*)"Resolution", val)==-1) return -4;
    mConf[name]->resolution = atof(val);

    if (us_get_value2(p, (char*)"Fov", val)==-1) return -5;
    fov = atof(val);

    mConf[name]->min_angle = fov * -0.5;
    mConf[name]->max_angle = fov * 0.5;
    mConf[name]->range_res = 0.01;
    //(*mConf)[name]->intensity = 0; // usarsim doesn't support intensity
    PLAYER_MSG5(1,"US_PARSER: us_get_ranger_config_all Ranger %s Min %f Max %f resolution %f max_range %f",name.c_str(),mConf[name]->min_angle,mConf[name]->max_angle,mConf[name]->resolution,mConf[name]->max_range);
    p += 5; // magic number: we've to jump over the current Name segment to the next one
  }

  return 0;
}
/*
 * Parses laser config information for several lasers at once
 *
 * Only makes sense if command 'GETCONF {Type RangeScanner}' was sent
 * without any 'Name' field
 */
int us_get_laser_config_all(char* data, map<string, player_laser_config_t*> mConf)
{
  char val[128];
  char *p = data;
  float fov;
  char dummy[256];
  PLAYER_MSG1(1,"us_get_laser_config %s",data);
  // search for Name fields
  while ((p = strstr(p, "{Name")) != NULL)
  {

    us_get_value2(p, (char*)"Name", dummy);
    if(strlen(dummy) == 0) return -2;
    string name(dummy);

    if (us_get_value2(p, (char*)"MaxRange", val)==-1) return -3;
    mConf[name]->max_range = atof(val);

    if (us_get_value2(p, (char*)"Resolution", val)==-1) return -4;
    mConf[name]->resolution = atof(val);

    if (us_get_value2(p, (char*)"Fov", val)==-1) return -5;
    fov = atof(val);

    mConf[name]->min_angle = fov * -0.5;
    mConf[name]->max_angle = fov * 0.5;
    mConf[name]->range_res = 0.01;
    mConf[name]->intensity = 0; // usarsim doesn't support intensity

    p += 5; // magic number: we've to jump over the current Name segment to the next one
  }

  return 0;
}
/*
 *
 */
/*
int us_get_victim_fiducial(char* origdata, player_victim_fiducial_data_t *fid)
{
  char tmp[128];
  char *p1, *p2;
  char *data;
  uint16_t count;
  int pos = 0;
  int32_t timestamp = 0;
  data=origdata;
  if (fid == NULL) return -1;
  memset(fid, 0, sizeof(player_victim_fiducial_data_t));
  count = fid->fiducials_count;
  if (us_get_value2(data, "Time", tmp)==-1) return 0;
  timestamp = (int)(atof(tmp)*1000);

  while (1){
    count = fid->fiducials_count;
    if ( (pos = us_get_value2(data, "ID", tmp))==-1) return 0;
    fid->fiducials[count].timestamp = timestamp;
    data += pos;
    strncpy(fid->fiducials[count].id,tmp, PLAYER_FIDUCIAL_MAX_ID_LENGTH - 1);
    if ((pos = us_get_value2(data, "Status", tmp))==-1) return 0;
    data += pos;
    strncpy(fid->fiducials[count].status,tmp, PLAYER_FIDUCIAL_MAX_STATUS_LENGTH - 1);
    if ((pos = us_get_value2(data, "Location", tmp))==-1) return -1;
    data += pos;
    if ((p2 = strchr(tmp,','))==NULL) return -1;
    *p2 = 0;
    fid->fiducials[count].pose.px = atof(tmp);
    p1 = p2+1;
    if ((p2 = strchr(p1,','))==NULL) return -1;
    *p2 = 0;
    fid->fiducials[count].pose.py =  (-1.0) * atof(p1);
    p1 = p2+1;
    fid->fiducials[count].pose.pyaw =  (-1.0) * atof(p1);
    fid->fiducials_count = (count + 1)%PLAYER_FIDUCIAL_MAX_SAMPLES;
  }
  return 0;
}
*/
/**
 *
 */
int us_get_bumper(char* data, player_bumper_data_t *bumper_data)
{
    PLAYER_MSG1(1,"us_get_bumper %s",data);
    PLAYER_MSG0(1,"TODO IMPLEMENT THIS");
    return -1;
}
/**
 *
 */
/*
int us_get_bumper_geom(char* data, player_bumper_data_t *bumper_data)
{
    PLAYER_MSG1(1,"us_get_bumper_geom %s",data);
    PLAYER_MSG0(1,"TODO IMPLEMENT THIS");
    return -1;
}
*/
/**
 *
 */
int us_get_fiducial(char* origdata, player_fiducial_data_t *fid)
{
  char tmp[128];
  char *p1, *p2;
  char *data;
  uint16_t count;
  int pos = 0;
  data = origdata;
  if (fid==NULL) return -1;
  memset(fid, 0, sizeof(player_fiducial_data_t));
  count = fid->fiducials_count;

  while (1){
    count = fid->fiducials_count;
    if ( (pos=us_get_value2(data, (char*)"ID", tmp)) == -1) {
      return 0;
    }
    data += pos;
    fid->fiducials[count].id = atoi(tmp);
    if ((pos=us_get_value2(data, (char*)"Location", tmp)) == -1)  return -1;
    data += pos;
    if ((p2 = strchr(tmp,',')) == NULL) return -1;
    *p2 = 0;
    fid->fiducials[count].pose.px = atof(tmp);
    p1 = p2+1;
    if ((p2 = strchr(p1,',')) == NULL) return -1;
    *p2 = 0;
    fid->fiducials[count].pose.py = (-1.0) * atof(p1);
    p1 = p2+1;
    fid->fiducials[count].pose.pz = (-1.0) * atof(p1);
    fid->fiducials_count = (count + 1);
  }
  return 0;
}

/**
 *
 */
int us_get_robot_config(char* data, char *steeringType, double &robotMass, double &maxSpeed,
            double &maxTorque, double &maxFrontSteer, double &maxRearSteer)
{
  char tmp[128];
  PLAYER_MSG1(3,"us_get_robot_config data: %s",data);

  if(steeringType == NULL) return -1;
  if (us_get_value2(data,(char*)"SteeringType",steeringType)==-1) return -1;
  if (us_get_value2(data,(char*)"Mass",tmp)==-1) return -1;
  robotMass = atof(tmp);
  if (us_get_value2(data,(char*)"MaxSpeed",tmp)==-1) return -1;
  maxSpeed = atof(tmp);
  if (us_get_value2(data,(char*)"MaxTorque",tmp)==-1) return -1;
  maxTorque = atof(tmp);
  if (us_get_value2(data,(char*)"MaxFrontSteer",tmp)==-1) return -1;
  maxFrontSteer = atof(tmp);
  //@todo there is a problem sigseg (?last value in string?)
  /*
  if (us_get_value2(data,"MaxRearSteer",tmp)==-1) return -1;
  //strtok(tmp, "}");
  maxRearSteer = atof(tmp);
  */
  return 0;
}
/**
 *
 */
int us_get_robot_geom(char* data, player_bbox3d_t *dimensions, double* COG, double &wheelRadius,
            double &maxWheelSeparation, double &wheelBase)
{
  char tmp[128];
  PLAYER_MSG1(3,"us_parser: get_robot_geom %s",data);
  if(dimensions == NULL) return -1;

  if (us_get_value2(data, (char*)"Dimensions", tmp)==-1) return -1;
  sscanf(tmp, "%lf,%lf,%lf", &(dimensions->sl),&(dimensions->sw),&(dimensions->sh) );
  if (us_get_value2(data, (char*)"COG", tmp)==-1) return -1;
  sscanf(tmp, "%lf,%lf,%lf", &COG[0], &COG[1], & COG[2] );
  if (us_get_value2(data,(char*)"WheelRadius",tmp)==-1) return -1;
  wheelRadius = atof(tmp);
  if (us_get_value2(data,(char*)"WheelSeparation",tmp)==-1) return -1;
//  maxWheelSeparation = atof(tmp);
//  PLAYER_MSG2(1,"WheelRadius %f maxWheelSeparation %f",wheelRadius,maxWheelSeparation);
//  PLAYER_MSG6(1,"Dimension (%f,%f,%f) COG (%f,%f,%f)",
//                  dimensions->sl,dimensions->sw,dimensions->sh,
//                  COG[0],COG[1],COG[2]);
  /*
  if (us_get_value2(data,"WheelBase",tmp)==-1) return -1;
  PLAYER_MSG1(3,"base tmp %s",tmp);
  wheelBase = atof(tmp);
  */
  return 0;
}

//////////////// UsLaser3d changes /////////////////////

/*
 * Transforms the given sensory input into a list of 3D point data.
 * The signal is of the form: {Name sensorName} {Resolution vertRes, horizRes} {FOV vertFOV, horizFOV} {Range r1, r2, ...}
 * Assumes that the scans are traversed row-wise, i.e. that for one fixed vertical angle first all possible horizontal
 * angles are scanned before the procedure is repeated for the next vertical angle.
 */
int us_get_laser3d(char* data, player_pointcloud3d_data_t *laser3d)
{
  static int filecount = 0;

  char tmp[US_MAX_MSG_LEN], *p1, *p2;
  float ranges[US_MAX_MSG_LEN], delta_angles[2], angles[2];
  int count = 0, index = 0;
  char* filename = new char[64];;

  float horiz_angle, vert_angle;
  float x,y,z;

  // Check whether ther is nothing to do:
  if(laser3d == NULL) {
    return -1;
  }
  //us_get_value2(data, "Name", name);
  //if(strlen(name) == 0) {
  //  return -1;
  //}

  // ---- Parse delta values ----
  // from {Resolution x,y}
  // where vertical resolution = x
  //       horizontal resolution = y

  if (us_get_value2(data, (char*)"Resolution", tmp) == -1) {
    return -1;
  }

  count = 0;
  p1 = tmp;
  while ((p2 = strchr(p1,','))>0)
  {
    *p2 = 0;
    delta_angles[count++] = atof(p1);
    p1 = p2+1;
  }
  delta_angles[count++] = atof(p1);

  if(count > 2) {
    fprintf(stderr, "Something went wrong. Too many 'Resolution' values (%i)", count);
    return -1;
  }

  // ---- Parse angle values ----
  // from {FOV x,y}
  // where vertical max angle = -(x/2)
  //       horizontal max angle = -(y/2)


  if (us_get_value2(data, (char*)"FOV", tmp) == -1) {
    return -1;
  }

  count = 0;
  p1 = tmp;
  while ((p2 = strchr(p1,','))>0)
  {
    *p2 = 0;
    angles[count++] = atof(p1);
    p1 = p2+1;
  }
  angles[count++] = atof(p1);

  if(count > 2) {
    fprintf(stderr, "Something went wrong. Too many 'FOV' values (%i)", count);
    return -1;
  }

  // ---- Parse range values ----
  // from {Range r1, r2, r3, ...}

  if (us_get_value2(data, (char*)"Range", tmp) == -1) {
    return -1;
  }

  count = 0;
  p1 = tmp;
  while ((p2 = strchr(p1,','))>0)
  {
    *p2 = 0;
    ranges[count++] = atof(p1);
    p1 = p2+1;
  }
  ranges[count++] = atof(p1);

  // ---- Convert into point cloud ----

  laser3d->points_count = count;

  // Save scans in text files
  std::ofstream out;
  sprintf(filename, "scans/scan%03i.3d", (filecount++));
  out.open (filename, ofstream::out | ofstream::trunc);

  // This assumes that the range data has been scanned row-wise!
  for(vert_angle = 0; vert_angle < angles[VERT]; vert_angle += delta_angles[VERT]) {
    for(horiz_angle = 0; horiz_angle < angles[HORIZ]; horiz_angle += delta_angles[HORIZ]) {

      // Calculate (x,y,z)-coordinates
      x = ranges[index] * sin(vert_angle) * sin(horiz_angle);
      y = ranges[index] * sin(vert_angle) * cos(horiz_angle);
      z = ranges[index] * cos(vert_angle);

      // Save current point
      out<<x<<" "<<y<<" "<<z<<std::endl;

      // Store point in pointcloud
      laser3d->points[index].point.px = x;
      laser3d->points[index].point.py = y;
      laser3d->points[index].point.pz = z;

      index++;
    }
  }

  out.close();

  printf("Parsed Laser3d data.\n");
  fflush(stdout);

  return 0;
}
