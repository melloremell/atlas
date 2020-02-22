#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ArduinoJson.h"
#include <iostream>
#include <sstream>
#include <string>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <ctime>
#include <fstream>
#include <unistd.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/LaserScan.h>
#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>

#include <atlas80evo_msgs/SetONOFF.h>
#include <atlas80evo_msgs/SetSound.h>
#include <atlas80evo_msgs/SetFSMState.h>
#include <atlas80evo_msgs/FSMState.h>
#include <atlas80evo_msgs/SetFSMState.h>
#include <atlas80evo_msgs/SetPose2D.h>
#include <geometry_msgs/PoseStamped.h>



using namespace std;
using namespace std_msgs;
using namespace ros;

string ver = "FMS_Handler Version 1.9.7";
string AGV_ID = "AT200_1";

Publisher health_pub;
Publisher diag_pub;
Publisher slam_pt_pub;
Publisher encr_pub;
Publisher encl_pub;
Publisher sr_break_pub;
Publisher led_pub;
Publisher nav_pub;
Publisher arm_pub;
Publisher point_pub;
Publisher mv_pub;
Publisher skip_pub;
Publisher sos_pub;
Publisher moveskipper_pub;
Publisher sus_pub;
Publisher to_obu_pub;
Publisher sched_pub;
Publisher reg_pub;
Publisher arm_param_pub;
Publisher armxtra_pub;
Publisher sysled_pub;

ServiceClient shutdownClient;
ServiceClient findpickdropClient;
ServiceClient findchargerClient;
ServiceClient playsound;
ServiceClient findoutchargerClient;
ServiceClient stateClient;
ServiceClient findtableClient;
ServiceClient checkposeClient;


atlas80evo_msgs::SetSound srv;

geometry_msgs::Twist sr_break;

int SR_SW = 0;
bool suspend = false;
bool found_table = false;//change this if true if bypass findtable
bool found_charger = false;
float max_charge = 100.0;
float low_charge = 5.0;
float junc_offset = 1.2;
float last_pX = 0.0;
float last_pY = 0.0;
float last_encL = 0.0;
float last_encR = 0.0;
float distPXY = 0.0;
float distENC = 0.0;
float MoveSOS_XYtoENC_Err = 10.0;
int batt_int = 0;
int RainFall = 1;
int HighHumidity = 50;
int sr_led = 0;
bool error = false;
bool wait = false;
bool waita = false;
bool juncfound = false;
bool NoTableError = false;
bool NoChargerError = false;
bool HumidSOS = false;
bool RainSOS = false;
bool OfflineSOS = false;
bool NoExit = false;
bool stopobs = false;
bool firstNavi = true;

string cur_arm = "";
string ms_a="";
string ms_pt="";
string ms_x="999";
string ms_y="999";
string ms_x_1="999";
string ms_y_1="999";
string ms_z="";
string ms_w="";
string ms_act="done";
string ms_seq="";
string sched_id="";
string ss_id="";
string charge="0";
string SR="0";
string sos_ID="";
string sos_type="";
string env_temp="";
string humid="";
string obu_id="";
string obu_temp="";
string rain="";
string Tray1="1";
string Tray2="1";
string Tray3="1";
string Tray4="1";
string cur_obj_detected ="4.0";
string cur_state="STANDBY";
string before_state="STANDBY";
string STATE = "STANDBY";
string armparam;

vector<string> sosID;
vector<int>sosID_enum;


DynamicJsonBuffer msstobj_Buffer;
JsonVariant msstObj;
JsonVariant msstptxy_ref;

//General variable
bool move_skip = false;
string ms_id = "";
int ms_cnt = 0;
int ms_max = 0;
string cur_lat= "";
string cur_long = "";
string cur_mapfname = "";
string ptsX="0.0";
string ptsY="0.0";
bool DropError = false;
bool PickError = false;
bool DropFail = false;
bool PickFail = false;


//for diagnostic
string ldr_2df = "1", ldr_2dr = "1", ldr_3d= "1";
string batt = "";
string lowbatt = "1";
string mot_l = "1",mot_r = "1";
string llc = "1";
string obu = "1";
string enc_l = "1", enc_r = "1";
string cam = "1",cpu= "1",ram = "1",arm="1";
string esw="1";
string imu="1";
string tcam="1";
string tray="1";

Time llc_time;
Time lidar3_time;
Time lidar2f_time;
Time lidar2r_time;
Time cam_time;
Time obu_time;
Time arm_time;
Time health_t;
Time imu_time;
Time tray_time;
Time tcam_time;
Time sus_t;
Time wait_t;
Time waita_t;
Time obs_t;
Time alert_t;
Time same_loc_delay;


string sos_ref[]={
"indexIncreaser",
"5e0e1358119f20805c774db8",//1
"5e0e1362119f20805c774db9",//2
"5e0e1370119f20805c774dba",//3
"5e0e1370119f20805c774dba",//4
"5e0e1394119f20805c774dbb",//5
"5e0e13a3119f20805c774dbc",//6
"5e0e13b3119f20805c774dbd",//7
"5e0e13bf119f20805c774dbe",//8
"5e0e13cb119f20805c774dbf",//9
"5e0e13d9119f20805c774dc0",//10
"5e0e13e8119f20805c774dc1",//11
"5e0e13fd119f20805c774dc2",//12
"5e0e1410119f20805c774dc3",//13
"5e0e141d119f20805c774dc4",//14
"5e0e1436119f20805c774dc5",//15
"5e0e1452119f20805c774dc6",//16
"5e0e1467119f20805c774dc7",//17
"5e1a3335fb289204edb2490b",//18
"5e1a3348fb289204edb2490d",//19
"5e1a335ffb289204edb2490f",//20
"5e1a336bfb289204edb24911",//21
"5e0e1480119f20805c774dc8",//22
"5e0e1495119f20805c774dc9",//23
"5e0e14a7119f20805c774dca",//24
"5e0e14bb119f20805c774dcb",//25
"5e0e14e4119f20805c774dcc",//26
"5e0e259b3d05532a38036794" //27 override
};

string sos_res_ref[]={
"indexIncreaser",
"Suspend",//1
"WaitAlarm",//2
"GoCharge",//3
"AbortGoHome",//4
"Wait",//5
"AbortSuspend"//6
};

int findres_enum(string sos_res)
{
  int o = 99;
  int mx = sizeof(sos_res_ref)/sizeof(sos_res_ref[0]);
  for(int x = 0; x < mx;x++)
  {
    if(sos_res_ref[x]==sos_res)
    {
      o = x;
      break;
    }
  }
  return 0;
}

void callstate(string str)
{
  if(str == "SUSPEND" || str == "MANUAL"){
    before_state = cur_state;
  }
  cur_state = str;
  // cout<<"BS:"<<before_state<<endl;
  // cout<<"S:"<<cur_state<<endl;
  atlas80evo_msgs::SetFSMState srvcall;
  srvcall.request.state = str.c_str();
  stateClient.call(srvcall);
}

void process_mem_usage(double& vm_usage, double& resident_set)
{
    vm_usage     = 0.0;
    resident_set = 0.0;

    // the two fields we want
    unsigned long vsize;
    long rss;
    {
        std::string ignore;
        std::ifstream ifs("/proc/self/stat", std::ios_base::in);
        ifs >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore
                >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore
                >> ignore >> ignore >> vsize >> rss;
    }

    long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // in case x86-64 is configured to use 2MB pages
    vm_usage = vsize / 1024.0;
    resident_set = rss * page_size_kb;
}

void process_cpu_usage(double& cpu)
{
    cpu  = 0.0;

    {
        std::string ignore;
        std::ifstream ifs("/proc/loadavg", std::ios_base::in);
        ifs >> cpu >> ignore >> ignore >> ignore >> ignore;
    }

    cpu = cpu*100/6.0;

}

void soswait()
{
  Duration diff_w=Time::now()-wait_t;
  if(diff_w.toSec() > 5)
  {
    wait_t = Time::now();
    wait = true;
  }

  if(wait)
  {
    sr_break_pub.publish(sr_break);
    cout<<"Waiting 5s..........."<<endl;
    wait = false;
  }
}

void soswaitalarm()
{
  Duration diff_wa=Time::now()-waita_t;
  if(diff_wa.toSec() > 5)
  {
    waita_t = Time::now();
    waita = true;
    srv.request.path2file = "/home/endstruct2/catkin_ws/src/atlas80evo/sounds/pleasemove.wav";
    srv.request.channel = 1;
    srv.request.volume = 1.0;
    srv.request.loop = -1;
    srv.request.interval = 0;
    playsound.call(srv);   
  }

  if(waita)
  {
    sr_break_pub.publish(sr_break);
    cout<<"Waiting and Alarm 5s....."<<endl;
    sr_break_pub.publish(sr_break);    
    srv.request.path2file = "/home/endstruct2/catkin_ws/src/atlas80evo/sounds/siren_beeping.ogg";
    srv.request.channel = 1;
    srv.request.volume = 1.0;
    srv.request.loop = -1;
    srv.request.interval = 0;
    playsound.call(srv);
    waita = false;
  }
}

void publish_diagnostic()
{

  DynamicJsonBuffer jsonWriteBuffer;
  JsonObject& root = jsonWriteBuffer.createObject();

  // cout<<"in"<<endl; success output:in
  root["agv_ID"]= AGV_ID;
  // cout<<root["agv_ID"]<<endl; success output:1
  JsonObject& ldrObj = root.createNestedObject("ldr");
  ldrObj["2df"]= ldr_2df;
  // cout<<ldrObj["2df"]<<endl; success output:0
  ldrObj["2dr"]= ldr_2dr;
  // cout<<ldrObj["2dr"]<<endl; success output:0
  ldrObj["3d"]= ldr_3d;
  // cout<<ldrObj["3d"]<<endl; success output:0
  root["batt"]= lowbatt;
  //cout<<root["batt"]<<endl; success output:""
  JsonObject& motObj = root.createNestedObject("mot");
  motObj["l"]=mot_l;
  // cout<<motObj["l"]<<endl; success output:""
  motObj["r"]=mot_r;
  // cout<<motObj["r"]<<endl; success output:""
  root["cam"]= cam;
  // cout<<root["cam"]<<endl; success output:0
  root["cpu"]= cpu;
  // cout<<root["cpu"]<<endl; success output:7.8333
  root["ram"]= ram;
  // cout<<root["ram"]<<endl; success output:335148:11044
  root["llc"]= llc;
  // cout<<root["llc"]<<endl; success output:0
  root["arm"]= arm;
  // cout<<root["arm"]<<endl; success output:0
  root["obu"]= obu;
  // cout<<root["arm"]<<endl; success output:0
  root["esw"]= esw;
  // cout<<root["arm"]<<endl; success output:0
  root["imu"]= imu;
  // cout<<root["arm"]<<endl; success output:0
  root["tray"]= tray;
  // cout<<root["arm"]<<endl; success output:0
  root["tcam"]= tcam;
  // cout<<root["obu"]<<endl; success output:0
  JsonObject& encObj = root.createNestedObject("enc");
  encObj["l"]=enc_l;
  // cout<<encObj["l"]<<endl; success output:" "
  encObj["r"]=enc_r;
  // cout<<encObj["r"]<<endl; success output:" "

  string tmpstr;
  root.printTo(tmpstr);
  // cout<<tmpstr<<endl; success
  //output: {"agv_ID":"1","ldr":{"2df":"0","2dr":"0","3d":"0"},
  //         "batt":"","mot":{"l":"","r":""},"cam":"0",
  //         "cpu":"3.33333","ram":"335148:10836","llc":"0",
  //         "arm":"0","obu":"0","enc":{"l":"","r":""}}
  String strt;
  strt.data = tmpstr;
  //cout<<strt<<endl; //success
  // output: data: {"agv_ID":"1","ldr":{"2df":"0","2dr":"0","3d":"0"},
  //                "batt":"","mot":{"l":"","r":""},"cam":"0",
  //                "cpu":"2.33333","ram":"335152:10956","llc":"0",
  //                "arm":"0","obu":"0","enc":{"l":"","r":""}}
  diag_pub.publish(strt);
}

void gotocharge()
{
  DynamicJsonBuffer navcBufferNav;
  JsonObject& cnobj = navcBufferNav.createObject();
  cnobj["map"] = msstObj["fname"];
  cnobj ["to_pt"] = "Home";
  cnobj ["to_x"] = msstObj["fname"]["Home_x"];
  cnobj ["to_y"] = msstObj["fname"]["Home_y"];
  cnobj ["to_z"] = msstObj["fname"]["Home_z"];
  cnobj ["to_w"] = msstObj["fname"]["Home_w"];

  string root2;
  cnobj.printTo(root2);
  String s;
  s.data = root2;
  cout<<"Go To: Home to Charge"<<endl;
  nav_pub.publish(s);
}

//process and publish the diagnostic data to FMS when requested
void sys_diag_Callback(const std_msgs::String::ConstPtr& msg)
{
  // {"agv_ID" : "1",    "cmd"  : "1" }
  string out = msg->data.c_str();
  DynamicJsonBuffer jsonBufferSysDiag;
  JsonObject& doc = jsonBufferSysDiag.parseObject(out);
  //cout<<doc["agv_ID"].as<string>()<<endl; success output:1
  if(doc["agv_ID"]==AGV_ID)
  {
    //cout<<"in"<<endl;  success output:in

    if(doc["cmd"]=="1")
    {
      publish_diagnostic();
      cout<<"FMS request diagnostic..."<<endl;
    }
  }
}

void lidar3Callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  lidar3_time = Time::now();
}

void lidar2fCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  lidar2f_time = Time::now();
}

void lidar2rCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  lidar2r_time = Time::now();
}

void camCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  cam_time = Time::now();
}

void obuCallback(const std_msgs::String::ConstPtr& msg)
{
  obu_time = Time::now();

  string sr_data = msg->data.c_str();
  DynamicJsonBuffer jsonReadBufferObu;
  JsonObject& readObj = jsonReadBufferObu.parseObject(sr_data);



  if(readObj["envPM25"] == NULL)
  {
    obu_temp = readObj["temperature"].as<string>();  
    obu_id = readObj["obu_ID"].as<string>();
  }

  if(readObj["temperature"] == NULL)
  {
    env_temp = readObj["envTemperature"].as<string>();
    humid = readObj["envHumidity"].as<string>();
    rain = readObj["rainfallIntensity"].as<string>();
    if(atoi(humid.c_str())>= HighHumidity)
    {
      HumidSOS = true;
    }
    if(atoi(rain.c_str())>= RainFall)
    {
      RainSOS=true;
    }

  }
}

//void armCallback(const std_msgs::String::ConstPtr& msg)
void armCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  arm_time = Time::now();
}

void tcamCallback(const std_msgs::String::ConstPtr& msg)
{
  tcam_time = Time::now();
}

void trayCallback(const std_msgs::String::ConstPtr& msg)
{
  tray_time = Time::now();

  string tr_data = msg->data.c_str();
  DynamicJsonBuffer jsonReadBuffertray;
  JsonObject& readTRObj = jsonReadBuffertray.parseObject(tr_data);

  Tray1 = readTRObj["t1"].as<string>();
  Tray2 = readTRObj["t2"].as<string>();
  Tray3 = readTRObj["t3"].as<string>();
  Tray4 = readTRObj["t4"].as<string>();
}

void ObjectRecogCallback(const std_msgs::String::ConstPtr& msg)
{
  tray_time = Time::now();

  cur_obj_detected = msg->data.c_str();

}

bool isOnline()
{
  FILE *output;

  if(!(output = popen("/sbin/route -n | grep -c '^0\\.0\\.0\\.0'","r")))
  {
        return 1;
  }
  unsigned int i;
  int x = fscanf(output,"%u",&i);
  if(i==0)
  {
    OfflineSOS = true;
    //cout<<"offline"<<endl;
  }
  else if(i==1)
  {
    OfflineSOS = false;
    //cout<<"online"<<endl;
  }
  pclose(output);
  return 1;

}

void sos_trigger_response(int res)
{
  switch(res)
  {
    case 1://Suspend
    {
      suspend = true;
      cout<<"SOS Triggered: Suspend"<<endl;
      break;
    }
    case 2://Wait and alarm
    {
      soswaitalarm();
      cout<<"SOS Triggered: Wait Alarm"<<endl;
      break;
    }
    case 3://Go To Charge (AGV only)
    {
      gotocharge();
      cout<<"SOS Triggered: Go to charge"<<endl;
      break;
    }
    case 4://Abort Mission and go to Charge
    {
      cout<<"SOS Triggered: Abort! Cancel All go to Home"<<endl;
      gotocharge();

      msstObj = 0;//clear mission
      break;
    }
    case 5:
    {
      soswait();
      cout<<"SOS Triggered: Wait"<<endl;
      break;
    }
    case 6:
    {
      soswait();
      cout<<"SOS Triggered: Abort and Suspend"<<endl;
      suspend = true;
      break;
    }
  }
  //After trigger response, reset the error trigger
  PickError = false;
  DropError = false;
  PickFail = false;
  DropFail = false;
  juncfound = false;
  NoTableError=false;
  NoChargerError=false;
  HumidSOS = false;
  RainSOS = false;
  OfflineSOS = false;
  NoExit = false;
}

void sos_trigger_response_str(string res) 
{
  sos_trigger_response(findres_enum(res));
}

//apply response accordingly
void sos_response(int sos_ID)
{
  if(STATE != "ERROR" && cur_state != "ERROR")
    {
      callstate("ERROR");
    }
  switch(sos_ID)
  {
    case 1://SOS:Cam, Response: Suspend(1)
    {
      sos_trigger_response(1);
      break;
    }
    case 2://SOS:Lidar3D, Response: Suspend(1)
    {
      sos_trigger_response(1);
      break;
    }
    case 3://SOS:Lidar2DF, Response: Suspend(1)
    {
      sos_trigger_response(1);
      break;
    }
    case 4://SOS:Lidar2DR, Response: Suspend(1)
    {
      sos_trigger_response(1);
      break;
    }
    case 5://SOS:LLC, Response: Suspend(1)
    {
      sos_trigger_response(1);
      break;
    }
    case 6://SOS:OBU, Response: Suspend(1)
    {
      sos_trigger_response(1);
      break;
    }
    case 7://SOS:Arm, Response: Suspend(1)
    {
      sos_trigger_response(1);
      break;
    }
    case 8://SOS:Imu, Response: Suspend(1)
    {
      sos_trigger_response(1);
      break;
    }
    case 9://SOS:Tray, Response: Suspend(1)
    {
      sos_trigger_response(1);
      break;
    }
    case 10://SOS:tcam, Response: Suspend(1)
    {
      sos_trigger_response(1);
      break;
    }
    case 11://SOS:offline, Response: Suspend(1)
    {
      sos_trigger_response(1);
      break;
    }
    case 12://SOS:short, Response: WaitAlarm(4)
    {
      sos_trigger_response(4);
      break;
    }
    case 13://SOS:long, Response: Suspend(1)
    {
      sos_trigger_response(1);
      break;
    }
    case 14://SOS:rain, Response: Suspend(1)
    {
      sos_trigger_response(1);
      break;
    }
    case 15://SOS:humidity, Response: AbortGoHome(3)
    {
      sos_trigger_response(3);
      break;
    }
    case 16://SOS:eswitch, Response: AbortSuspend(6)
    {
      sos_trigger_response(4);
      break;
    }
    case 17://SOS:derr, Response: Suspend(1)
    {
      sos_trigger_response(1);
      break;
    }
    case 18://SOS:perr, Response: Suspend(1)
    {
      sos_trigger_response(1);
      break;
    }
    case 19://SOS:dfail, Response: (1)
    {
      sos_trigger_response(1);
      break;
    }
    case 20://SOS:pfail, Response: Suspend(1)
    {
      sos_trigger_response(1);
      break;
    }
    case 21://SOS:junc, Response: Suspend(1)
    {
      sos_trigger_response(1);
      break;
    }
    case 22://SOS:notable, Response: Suspend(1)
    {
      sos_trigger_response(1);
      break;
    }
    case 23://SOS:no charger, Response: Suspend(1)
    {
      sos_trigger_response(1);
      break;
    }
    case 24://SOS:noexit, Response: Suspend(1)
    {
      sos_trigger_response(1);
      break;
    }
    case 25://SOS:moveerr, Response: Suspend(1)
    {
      sos_trigger_response(1);
      break;
    }
    case 26://SOS:lowbatt, Response: Gocharge(2)
    {
      sos_trigger_response(2);
      break;
    }
  }
}

void sos_notify(string err_id)
{
  DynamicJsonBuffer jsonSosWriteBuffer;
  JsonObject& rosos = jsonSosWriteBuffer.createObject();
  rosos["agv_ID"]= AGV_ID;
  JsonArray& arr = rosos.createNestedArray("sos_ID");

  string ID="";
  if(err_id != "0")//if sos trigger coming from FMS
  {
    ID = err_id;
    arr.add(ID);
  }
  else
  {
    for(std::vector<string>::size_type i = 0; i != sosID.size(); i++) 
    {
      ID = sosID[i];
      arr.add(ID);
      //sos_response(sosID_enum[i]);
      //cout<<i<<":"<<sosID_enum[i]<<","<<sos_ref[i]<<","<<ID<<endl;
    }
    //cout<<endl;
  }

  //------------Shoot to FMS for NOTIFICATION--------------
  string tmpstr;
  rosos.printTo(tmpstr);
  String strt;
  strt.data = tmpstr;
  sos_pub.publish(strt);

  sosID.clear();
  sosID_enum.clear();
  error = false;

}

//check have exit or not before exit charging house
bool findExit()
{
  return true;
}

void generic_check()
{
  String sysled;
  sysled.data = "1";
  sysled_pub.publish(sysled);
  //----------------------GENERIC REPEAT CHECK---------------------------
  
  //check online
  isOnline();

  //check junction
  // JsonArray& msjunc = msstObj["junc"];
  // for(int x = 0; x<msjunc.size();x++)
  // {
  //   string sx = msjunc [x]["x"].as<string>();
  //   float px = atof(sx.c_str());
  //   string sy = msjunc [x]["y"].as<string>();
  //   float py = atof(sy.c_str());

  //   float djx = atof(ptsX.c_str())-px;
  //   float djy = atof(ptsY.c_str())-py;
  //   float distjxy = sqrt(pow(djx,2)+pow(djy,2));

  //   if(distjxy<junc_offset)
  //   {
  //     suspend = true;
  //     cout<<"Suspended on Junction"<<endl;
  //     juncfound = true;
  //   }

  // }


  // //check pickdrop region
  string lt = msstptxy_ref[ms_cnt]["arm"]["cmd"].as<string>();
  string first_four = lt.substr(0, 4);
  // if(msstptxy_ref.as<string>() != "")
    // cout <<"AMCL_X:"<< ptsX<<endl;
    // cout <<"AMCL_Y:"<< ptsY<<endl;
    // cout <<"TargetPoint_X:"<< ms_x<<endl;
    // cout <<"TargetPoint_Y:"<< ms_y<<endl;
  // if(first_four == "pick" || first_four == "drop" || lt  == "gohome")
  // {
    float dx = atof(ptsX.c_str())-atof(ms_x.c_str());
    float dy = atof(ptsY.c_str())-atof(ms_y.c_str());
    float distxy = sqrt(pow(dx,2)+pow(dy,2));

    float dx_1 = atof(ptsX.c_str())-atof(ms_x_1.c_str());
    float dy_1 = atof(ptsY.c_str())-atof(ms_y_1.c_str());
    float distxy_1 = sqrt(pow(dx_1,2)+pow(dy_1,2));
    // cout<< "Diff_CMsquare:"<<distxy<<endl;
    String m;
    if(distxy<junc_offset || distxy_1<junc_offset)
    {
      //cout<<"Entering PickDrop Region..."<<endl;
      
      m.data = "pickdrop";
      reg_pub.publish(m);
    }
    else
    {
      m.data = "normal";
      reg_pub.publish(m);    
    }
  // }




  //check charging condition
  if(charge == "1" && found_charger==true)//if in charging mode
  {
    if(atof(batt.c_str()) > max_charge)//check current battery
    {
      charge = "0";
      //go out of charger house
      if(STATE != "LEAVING" && cur_state != "LEAVING")
      {
        callstate("LEAVING");
      }
      atlas80evo_msgs::SetONOFF srvco;
      if(findExit() == true)
      {
        srvco.request.data = true;
        findoutchargerClient.call(srvco);
        found_charger=false;
        cout<<"Go out of chargin house..."<<endl;
      }else
      {
        NoExit = true;
      }
    }
  }

  //Execution of suspend and Resume
  if (suspend)
  {
    if(STATE != "SUSPEND" && cur_state != "SUSPEND")
    {
      callstate("SUSPEND");
    }
    sr_break_pub.publish(sr_break);
    sr_led = 1;
    SR = "1";//FMS SR status
    //cout << "Suspend -- Suspend -- Suspend -- Suspend --Suspend "<<Time::now()<< endl;
    sus_pub.publish(SR);
  }
  else
  { 
    // if(cur_state != before_state)
    // {
    //   callstate(before_state);
    // }
    sr_led = 0;
    SR = "0";//FMS SR status
    sus_pub.publish(SR);
    //cout << "Move -- Move -- Move -- Move -- Move -- Move -- Move -- Move " << endl;
  }

}


void sos_check()
{
  
  //-------------------------------Diagnostic Error SOS Publish------------
  //check ram
  double vm, rss;
  process_mem_usage(vm, rss);
  ostringstream ss,sw;
  ss << rss;
  sw << vm;
  ram = sw.str() + ":" + ss.str();

  //check cpu
  double cu;
  process_cpu_usage(cu);
  ostringstream cp;
  cp << cu;
  cpu = cp.str();


  // //1check camera (1)------------------------OK
  // Duration diff_c=Time::now()-cam_time;
  // if(diff_c.toSec() > 5)
  // {
  //   cam = "0";error = true;
  //   sosID.push_back(sos_ref[1]); 
  //   sosID_enum.push_back(1); 
  //   cout<<"SOS!: Camera Error"<<endl;
  // }
  // else
  // {
  //   cam = "1";
  // }

  // //2check lidar3d (2)
  // Duration diffl3=Time::now()-lidar3_time;
  // if(diffl3.toSec() > 5)
  // {
  //   ldr_3d = "0";error = true;
  //   sosID.push_back(sos_ref[2]);   
  //   sosID_enum.push_back(2); 
  //   cout<<"SOS!: Lidar3D Error"<<endl;
  // }
  // else
  // {
  //   ldr_3d = "1";
  // }

  // //3check lidar2d front (3)
  // Duration diffl2f=Time::now()-lidar2f_time;
  // if(diffl2f.toSec() > 5)
  // {
  //   ldr_2df = "0";error = true;
  //   sosID.push_back(sos_ref[3]); 
  //   sosID_enum.push_back(3); 
  //   cout<<"SOS!: Lidar2D Front Error"<<endl;  
  // }
  // else
  // {
  //   ldr_2df = "1";
  // }

  // //4check lidar2d rear (4)
  // Duration diffl2r=Time::now()-lidar2r_time;
  // if(diffl2r.toSec() > 5)
  // {
  //   ldr_2dr = "0";error = true;
  //   sosID.push_back(sos_ref[4]); 
  //   sosID_enum.push_back(4); 
  //   cout<<"SOS!: Lidar2D Rear Error"<<endl;  
  // }
  // else
  // {
  //   ldr_2dr = "1";
  // }

  // //5check llc (5)
  // Duration diff=Time::now()-llc_time;
  // if(diff.toSec() > 5)
  // {
  //   llc = "0";error = true;
  //   sosID.push_back(sos_ref[5]);  
  //   sosID_enum.push_back(5);  
  //   cout<<"SOS!: Low Level Controller Error"<<endl;
  // }
  // else
  // {
  //   llc = "1";
  // }

  // //6check obu (6)
  // Duration diffo=Time::now()-obu_time;
  // if(diffo.toSec() > 5)
  // {
  //   obu = "0";error = true;
  //   sosID.push_back(sos_ref[6]);  
  //   sosID_enum.push_back(6); 
  //   cout<<"SOS!: OBU Error"<<endl;
  // }
  // else
  // {
  //   obu= "1";
  // }

  // //7check arm (7)
  // Duration diffar=Time::now()-arm_time;
  // if(diffar.toSec() > 5)
  // {
  //   arm = "0";error = true;
  //   sosID.push_back(sos_ref[7]);  
  //   sosID_enum.push_back(7); 
  //   cout<<"SOS!: Arm Error"<<endl; 
  // }
  // else
  // {
  //   arm= "1";
  // }

  // //8check imu (8)
  // Duration diffimu=Time::now()-imu_time;
  // if(diffimu.toSec() > 5)
  // {
  //   imu = "0";error = true;
  //   sosID.push_back(sos_ref[8]);  
  //   sosID_enum.push_back(8);  
  //   cout<<"SOS!: IMU Error"<<endl;
  // }
  // else
  // {
  //   imu = "1";
  // }
  
  // //9check tray (9)
  // Duration difftray=Time::now()-tray_time;
  // if(difftray.toSec() > 5)
  // {
  //   tray = "0";error = true;
  //   sosID.push_back(sos_ref[9]); 
  //   sosID_enum.push_back(9);  
  //   cout<<"SOS!: Tray Sensor Error"<<endl; 
  // }
  // else
  // {
  //   tray = "1";
  // }

  // //10check tower camera (10)
  // Duration difftcam=Time::now()-tcam_time;
  // if(difftcam.toSec() > 5)
  // {
  //   tcam = "0";error = true;
  //   sosID.push_back(sos_ref[10]); 
  //   sosID_enum.push_back(10);  
  //   cout<<"SOS!: Tower Camera Error"<<endl; 
  // }
  // else
  // {
  //   tcam = "1";
  // }

  // //-------------------------------operation Error SOS only------------

  // //11check offline (11)
  
  if(OfflineSOS)
  {
    error = true;
    sosID.push_back(sos_ref[11]); 
    sosID_enum.push_back(11);  
    ROS_INFO("SOS!: Offline Error");
  }
  

  // //12check obstacle_short (12)
  // Duration diffobs=Time::now()-obs_t;
  // if(diffobs.toSec() > 5)//timeout if no obstacle detected in 5s
  // {
  //   stopobs=false;
  // }
  // else//if detected under 5s. keep true
  // {
  //   stopobs=true;
  // }
  // if(stopobs)//while true
  // {
  //   Duration durofobs=Time::now()-obs_t;
  //   if(durofobs.toSec() > 2 && durofobs.toSec() <= 30)
  //   { //if obstacle keep on detected between 5s to 30s
  //     Duration alertobs=Time::now()-alert_t;
  //     if(alertobs.toSec() > 5)//shoot alert every 5s only
  //     {
  //       //12check obstacle_short (12)
  //       error = true;
  //       sosID.push_back(sos_ref[12]); 
  //       sosID_enum.push_back(12);   
  //       alert_t = Time::now();  
  //       cout<<"SOS!: Obstacle Short Error"<<endl;   
  //     }

  //   }
  //   else if(durofobs.toSec() > 30)//if occure more than 30s
  //   { //if obstacle keep on detected more 30s
  //     //13check obstacle_long (13)
  //     error = true;
  //     sosID.push_back(sos_ref[13]); 
  //     sosID_enum.push_back(13); 
  //     cout<<"SOS!: Obstacle Long Error"<<endl;
  //   }

  // }


  // //14check rain (14)
  // if(HumidSOS)
  // {
  //   error = true;
  //   sosID.push_back(sos_ref[14]); 
  //   sosID_enum.push_back(14);  
  //   cout<<"SOS!: Rain Error"<<endl;
  // }
  

  // //15high humidity (15)
  // if(RainSOS)
  // {
  //   error = true;
  //   sosID.push_back(sos_ref[15]); 
  //   sosID_enum.push_back(15);  
  //   cout<<"SOS!: Humidity Error"<<endl;
  // }

  // //16check emergency button (16)
  if(esw == "0")
  {
    error = true;
    sosID.push_back(sos_ref[16]); 
    sosID_enum.push_back(16);  
    ROS_INFO("SOS!: Emergency Button Error");
  }

  // //17container_perr (17)
  // if(PickError)
  // {
  //   error = true;
  //   sosID.push_back(sos_ref[17]);  
  //   sosID_enum.push_back(17); 
  //   cout<<"SOS!: Container Pick Error"<<endl;
  // }

  // //18container_derr (18)
  // if(DropError)
  // {
  //   error = true;
  //   sosID.push_back(sos_ref[18]);  
  //   sosID_enum.push_back(18);  
  //   cout<<"SOS!: Container Drop Error"<<endl;
  // }

  // //19container_dfail(19)
  // if(DropFail)
  // {
  //   error = true;
  //   sosID.push_back(sos_ref[19]); 
  //   sosID_enum.push_back(19);   
  //   cout<<"SOS!: Container Drop Fail"<<endl;
  // }

  // //20container_pfail (20)
  // if(PickFail)
  // {
  //   error = true;
  //   sosID.push_back(sos_ref[20]);  
  //   sosID_enum.push_back(20);  
  //   cout<<"SOS!: Container Pick Fail"<<endl;
  // }


  // //21junction  (21)
  // if(juncfound)
  // {
  //   error = true;
  //   sosID.push_back(sos_ref[21]);  
  //   sosID_enum.push_back(21);  
  //   cout<<"SOS!: Junction Error"<<endl;
  // }

  // //22no_table (22)
  // if(NoTableError)
  // {
  //   error = true;
  //   sosID.push_back(sos_ref[22]);  
  //   sosID_enum.push_back(22); 
  //   cout<<"SOS!: No Table Error"<<endl; 
  // }


  // //23no_charger (23)
  // if(NoChargerError)
  // {
  //   error = true;
  //   sosID.push_back(sos_ref[23]);  
  //   sosID_enum.push_back(23); 
  //   cout<<"SOS!: No Charger Error"<<endl; 
  // }

  // //24no_exit (24)
  // if(NoExit)
  // {
  //   error = true;
  //   sosID.push_back(sos_ref[24]); 
  //   sosID_enum.push_back(24);   
  //   cout<<"SOS!: No Exit Error"<<endl;
  // }
  

  // //25move_error (25)
  // if(abs(distENC - distPXY) > MoveSOS_XYtoENC_Err)
  // {
  //   error = true;
  //   sosID.push_back(sos_ref[25]); 
  //   sosID_enum.push_back(25);   
  //   cout<<"SOS!: Move Error"<<endl;
  // }
  

  // //26low battery (26)
  if(lowbatt=="0")
  {
    error = true;
    sosID.push_back(sos_ref[26]);  
    sosID_enum.push_back(26);  
    ROS_INFO("SOS!: Low Battery");
  }


  //SOS RESPONSE AND NOTIFICATION
  if(error)//reset 
  {
  
    sos_notify("0");
    publish_diagnostic();

  }
}
//process and publish the health data to FMS periodically
//including all Suspend Resume stat
void agv_health_pub(const std_msgs::String::ConstPtr& msg)
{
  //get health data from here and there
  llc_time = Time::now();
  //publish it
  string sr_data = msg->data.c_str();
  DynamicJsonBuffer jsonReadBufferHealth;
  DynamicJsonBuffer jsonWriteBufferHealth;
  JsonObject& readObj = jsonReadBufferHealth.parseObject(sr_data);

  if(readObj.success())
  {
    // cout<<readObj<<endl; success output: ___
    JsonObject& root = jsonWriteBufferHealth.createObject();
    root["agv_ID"] = AGV_ID;
    // cout<<root<<endl; success output: {"agv_ID":"1"}
    string tmpRenc = readObj["ENC?"][0];
    string tmpLenc = readObj["ENC?"][2];

    float encR = atof(tmpRenc.c_str());
    float encL = atof(tmpLenc.c_str());

    distENC = sqrt(pow(encL-last_encL,2)+pow(encR-last_encR,2));
    last_encR = encR;
    last_encL = encL;

    string motorX = readObj["MOV?"][0];
    string motorZ = readObj["MOV?"][1];

    if(tmpRenc != ""){ enc_r = "1"; }

    else { enc_r = "0"; }

    if(tmpLenc != ""){ enc_l = "1"; }

    else { enc_l = "0"; }

    string tmpRT = readObj["STA?"][1];
    string tmpLT = readObj["STA?"][6];

    if(tmpRT != ""){ mot_r = "1"; }

    else { mot_r = "0"; }

    if(tmpLT!= ""){ mot_l = "1"; }

    else { mot_l = "0"; }

    string rawBatt = readObj["STA?"][0];
    
    batt = rawBatt;
    batt_int = atoi(batt.c_str());
    // cout<<batt_int<<endl;
    float bm = (float(batt_int-41))/15*100;
    // cout<<bm<<endl;
    ostringstream btw;
    btw << fixed<<setprecision(1)<<bm;
    root["batt"] = btw.str();

    string mmb = btw.str();
    if(atof(mmb.c_str()) < low_charge)
    {
      charge = "1";
      lowbatt = "0";
    }
    else
    {
      lowbatt = "1";
    }


    // Duration diff_sus=Time::now()-sus_t;
    // if(diff_sus.toSec() > 1 && readObj["DI?"][3] == 0 && readObj["DI?"][3] == SR_SW)
    // {
    //   SR_SW = readObj["DI?"][3];
    //   suspend = !suspend;
    //   sus_t = Time::now();

    //   if(cur_state == "SUSPEND")
    //   {
    //     callstate(before_state);
    //   }

    // }


    JsonObject& msObj = root.createNestedObject("ms");
    msObj["ms_ID"] = ms_id;
    msObj["sched_ID"] = sched_id;
    msObj["ss_id"] = ss_id;
    ostringstream ss;
    ss << ms_cnt+1;
    msObj["act_Seq"] = ms_seq;
    msObj["point"] = ms_pt;
    msObj["act"] = ms_act;


    root["charge"]=charge;
    root["SR"]=SR;

    JsonObject& msObj2 = root.createNestedObject("loc");
    msObj2["lat"]=cur_lat;
    msObj2["long"]=cur_long;

    JsonObject& msObj3 = root.createNestedObject("pnt");
    msObj3["x"]=ptsX;
    msObj3["y"]=ptsY;

    JsonObject& msObj4 = root.createNestedObject("spd");
    msObj4["x"]=motorX;
    msObj4["z"]=motorZ;

    JsonObject& msObj5 = root.createNestedObject("tmp");
    msObj5["l"]=tmpLT;
    msObj5["r"]=tmpRT;

    string tmpobu;
    root.printTo(tmpobu);
    String stobu;
    stobu.data = tmpobu;


    JsonObject& msObj7 = root.createNestedObject("obu");
    msObj7["env_temp"]=env_temp;
    msObj7["humid"]=humid;
    msObj7["obu_id"]=obu_id;
    msObj7["obu_temp"]=obu_temp;
    msObj7["rainfallintensity"]=rain;

    string tmpstr;
    root.printTo(tmpstr);
    String strt;
    strt.data = tmpstr;

    Duration diff_c=Time::now()-health_t;
    if(diff_c.toSec() > 1)
    {
      //cout<<strt<<endl;
      health_pub.publish(strt);
      to_obu_pub.publish(stobu);
      health_t = Time::now();
     /* cout<<"cpu :"<<cpu<<"%"<<endl;*/

    }
  }

}



//entering Teleop
void TeleOPCallback(const std_msgs::String::ConstPtr &msg)
{
  string manData = msg->data.c_str();
  DynamicJsonBuffer jsonReadBufferMvMan;
  JsonObject& readObj = jsonReadBufferMvMan.parseObject(manData);

  // cout<<readObj["agv_ID"]<<endl; success output:1

  if(readObj["agv_ID"]== AGV_ID)
  {
    if(readObj["status"]== "0")
    {
      if(STATE != before_state && cur_state != before_state)
      {
        callstate(before_state);
      }
      // srv.request.path2file = "/home/endstruct2/catkin_ws/src/atlas80evo/sounds/stand.wav";
      // srv.request.channel = 1;
      // srv.request.volume = 1.0;
      // srv.request.loop = -1;
      // srv.request.interval = 0;
      // playsound.call(srv);
    }
    else if(readObj["status"]== "1")
    {
      if(STATE != "MANUAL" && cur_state != "MANUAL")
      {
        callstate("MANUAL");
      }
      // srv.request.path2file = "/home/endstruct2/catkin_ws/src/atlas80evo/sounds/manual.wav";
      // srv.request.channel = 1;
      // srv.request.volume = 1.0;
      // srv.request.loop = -1;
      // srv.request.interval = 0;
      // playsound.call(srv);
    }

  }

}

//Handlin TeleOP of AGV
void mvSysCallback(const std_msgs::String::ConstPtr &msg)
{ //move robot using fms UI from server
  //{"agv_ID" : "1","x": "0.1","z": "0.1","cmd": "0/1"}
  //cmd 1 is start, 0 is stop
  string mvData = msg->data.c_str();
  DynamicJsonBuffer jsonReadBufferMvSys;
  JsonObject& readObj = jsonReadBufferMvSys.parseObject(mvData);

  // cout<<readObj["agv_ID"]<<endl; success output:1

  if(readObj["agv_ID"]== AGV_ID)
  {
    // cout<<"in"<<endl; success output:in
    if(readObj["cmd"]=="1")
    {
      // cout<<"in"<<endl; success when cmd is 1 output:in
      if(esw != "0")
      {
        geometry_msgs::Twist move;
        move.linear.x = float(readObj["x"]);
        move.linear.y = move.linear.z = 0.0;
        move.angular.x = move.angular.y = 0.0;
        move.angular.z = float(readObj["z"]);
        cout<<move<<endl;
        mv_pub.publish(move);
      }
    }

  }
}

void nanocallback(const std_msgs::String::ConstPtr& msg) 
{
  string srt_data = msg->data.c_str();
  DynamicJsonBuffer jsonReadBuffernano;
  JsonObject& readnano = jsonReadBuffernano.parseObject(srt_data);

  //emergency switch
  ostringstream sesw;
  sesw << readnano["DMI?"][4];
  esw = sesw.str();


  Duration diff_sus=Time::now()-sus_t;
  if(diff_sus.toSec() > 1 && readnano["DMI?"][5] == 0 && readnano["DMI?"][5] == SR_SW)
  {
    string srt_data = msg->data.c_str();
    DynamicJsonBuffer jsonReadBuffernano;
    JsonObject& readnano = jsonReadBuffernano.parseObject(srt_data);
    Duration diff_sus=Time::now()-sus_t;
    if(diff_sus.toSec() > 1 && readnano["DMI?"][5] == 0 && readnano["DMI?"][5] == SR_SW)
    {
      SR_SW = readnano["DMI?"][5];
      suspend = !suspend;
      sus_t = Time::now();


      if(cur_state == "SUSPEND")
      {
        String tz;
        tz.data = "resume";
        armxtra_pub.publish(tz);

        if(cur_arm != "")
        {
          String armove;
          armove.data = cur_arm;
          arm_pub.publish(armove);
        }

        callstate(before_state);
      }
      else
      {
        String ta;
        ta.data = "suspend";
        armxtra_pub.publish(ta);
      }

    }
      // SR_SW = readnano["DMI?"][3];
      // suspend = !suspend;
      // sus_t = Time::now();

      // if(cur_state == "SUSPEND")
      // {
      //   callstate(before_state);

      //   String tz;
      //   tz.data = "resume";
      //   armxtra_pub.publish(tz);
      //   Duration(1).sleep(); 
      //   if(cur_arm != "")
      //   {
      //     String armove;
      //     armove.data = cur_arm;
      //     arm_pub.publish(armove);
      //   }
      // }
      // else
      // {
      //   String ta;
      //   ta.data = "suspend";
      //   armxtra_pub.publish(ta);
      // }
    }
}

void gpsCallback(const sensor_msgs::NavSatFixConstPtr& msg)
{
  char lt[30];
  char lg[30];
  sprintf(lg,"%.25f",msg->longitude);
  sprintf(lt,"%.25f",msg->latitude);

  string xlg(lg);
  string xlt(lt);
  cur_long = xlg;
  cur_lat = xlt;

 

}
void ptsCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL){
  ostringstream px,py;
  float pX = msgAMCL->pose.pose.position.x;
  float pY = msgAMCL->pose.pose.position.y;
  px << pX;
  py << pY;
  ptsX = px.str();
  ptsY = py.str();

  distPXY = sqrt(pow(pX-last_pX,2)+pow(pY-last_pY,2));

  last_pX = pX;
  last_pY = pY;

}

void sys_sos_Callback(const std_msgs::String::ConstPtr &msg){
  string off = msg->data.c_str();
  DynamicJsonBuffer jsonReadBufferOff;
  JsonObject& readObjO = jsonReadBufferOff.parseObject(off);

  if (readObjO["agv_ID"]==AGV_ID)//check ID
  {
    if(readObjO["sos_ID"] == sos_ref[26])//if it is override
    {
      sos_notify(readObjO["sos_ID"]);//for notification
      sos_trigger_response_str(readObjO["res"]);//response
    }
  }
}


void sys_off_Callback(const std_msgs::String::ConstPtr &msg){
  string off = msg->data.c_str();
  DynamicJsonBuffer jsonReadBufferOff;
  JsonObject& readObjO = jsonReadBufferOff.parseObject(off);

  if (readObjO["agv_ID"]==AGV_ID)//check ID
  {
    if (readObjO["status"] == "shutdown")//Check command
    {
      //call python shutdown service
      // std_srvs::Empty srv;
      // shutdownClient.call(srv);
      // std_srvs::Empty srvg;
      // armgohomeClient.call(srvg);

    }
  }
}


void goOutHome()
{
  //go out of charger house
  atlas80evo_msgs::SetONOFF srvco;
  if(findExit() == true)
  {
    if(STATE != "LEAVING" && cur_state != "LEAVING")
    {
      callstate("LEAVING");
    }
    srvco.request.data = true;
    findoutchargerClient.call(srvco);
    cout<<"Go out of chargin house (HOME)..."<<endl;
  }else
  {
    NoExit = true;
  }
}



void NaviFirstMission()
{
  if(STATE != "DELIVERY" && cur_state != "DELIVERY")
  {
    callstate("DELIVERY");
  }
  DynamicJsonBuffer navBuffer;
  JsonObject& obj = navBuffer.createObject();

  JsonArray& msstptxy = msstObj["activity"];
  msstptxy_ref = msstptxy;
  obj["map"] = msstObj["fname"];
  ms_max = msstptxy.size();
  //cout<<"Json msst"<<msstptxy<<endl;

  //cout<<ms_max<<endl; //success output:2
  sched_id = msstObj["sched_ID"].as<string>();
  ms_id = msstObj["ms_ID"].as<string>();
  ss_id = msstObj["ss_id"].as<string>();



  obj ["to_pt"] = msstptxy [0]["pt"].as<string>();

  obj ["to_x"] = msstptxy [0]["x"].as<string>();

  obj ["to_y"] = msstptxy [0]["y"].as<string>();

  obj ["to_z"] = msstptxy [0]["z"].as<string>();

  obj ["to_w"] = msstptxy [0]["w"].as<string>();

  ms_seq = msstptxy [0]["Seq"].as<string>();;
  ms_pt = "home";
  ms_act = "move";

  ms_x_1 = ms_x;
  ms_y_1 = ms_y;

  ms_x = msstptxy [0]["x"].as<string>();
  ms_y = msstptxy [0]["y"].as<string>();

  string root2;
  obj.printTo(root2);
  // cout<<root2<<endl; success output:{"map":"OneNorth_1351434245",
  //                                    "to_x":"4.4","to_y":"5.5"}
  String s;
  s.data = root2;
  nav_pub.publish(s);//go to 1st location msstptxy [0]["pt"]

  ms_pt = msstptxy [0]["pt"].as<string>();
  cout<<"Go To: "<<obj["to_pt"]<<endl;
}

//start the mission from FMS web
void msstCallback(const std_msgs::String::ConstPtr &msg)
{
  if(ms_act=="done")
  {

    string Data = msg->data.c_str();
    msstObj = msstobj_Buffer.parseObject(Data);
    // cout<<msstObj["agv_ID"]<<endl; success output:1
    //cout<<msstObj<<endl;
    //cout<<msstObj<<endl;
    if(msstObj["agv_ID"]==AGV_ID)
    {
      //cout<<"in"<<endl;
      //cout<<msstObj["fname"]<<endl;
      // cout<<"in"<<endl;  success output:in
      //goOutHome();
      NaviFirstMission();

    }
  }

}


//find table before do navigate to table
void findPickDrop()
{
    //cout<<"Detecting Table with camera..."<<endl;
    if(cur_obj_detected =="4.0")//check can see table or not
    {
      
      cout<<"Table Found: Calling PickDrop Point"<<endl;
      // cout<<"Table Found: Navigating to table."<<endl;
      atlas80evo_msgs::SetONOFF srvft;
      srvft.request.data = true;
      findpickdropClient.call(srvft);
    }
    else
    {
      NoTableError=true;
    }
}

void findTable()
{

  atlas80evo_msgs::SetONOFF srvft;
  srvft.request.data = true;
  Duration(1).sleep(); 
  findtableClient.call(srvft);

  cout<<"Find and align with table..."<<endl;
}

//Server callback when charging Navigation to Charging Dock Done
bool DoneFindChargingDockCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  if(ms_act=="gohome")
  {
    String s;
    s.data = "{\"st\":\"0\",\"act\":\"arm\"}";
    sched_pub.publish(s);
  }
  else
  {
    found_charger = true;
  }
  return true;
}

//will be called after AGV get out of the Charging house, then proceed with navigation
bool DoneFindOutChargingDockCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  if(firstNavi)
  {
    NaviFirstMission();
  }
  else
  {
    //publish hdl
    DynamicJsonBuffer navBuffer;
    JsonObject& nobj = navBuffer.createObject();
    nobj["st"] = "0";
    nobj ["act"] = "arm";
    string root2;
    nobj.printTo(root2);
    String s;
    s.data = root2;
    cout<<"Charging DONE: Go to next point directly"<<s<<endl;
    sched_pub.publish(s);
  }
  return true;
}

//will be called after AGV get out of the Charging house, then proceed with navigation
bool DoneFindPickupCallback(atlas80evo_msgs::SetPose2D::Request& request, atlas80evo_msgs::SetPose2D::Response& response)
{
  
  found_table = true;
  cout<<"Called /check/pose. Waiting for service callback"<<endl;
  //Duration(0.5).sleep(); 
  atlas80evo_msgs::SetPose2D srvft;
  srvft.request.x = request.x;
  srvft.request.y = request.y;
  srvft.request.theta = request.theta;
  checkposeClient.call(srvft);

  

  armparam="";
  DynamicJsonBuffer paramBuffer;
  JsonObject& paramobj = paramBuffer.createObject();
  ostringstream xx,yy,tt;
  xx << request.x;
  yy << request.y;
  tt << request.theta;
  paramobj["new_x"]=xx.str();
  paramobj["new_y"]=yy.str();
  paramobj["new_deg"]=tt.str();
  paramobj.printTo(armparam);
  


  return true;
}


//Server callback when find table for arm Done
bool DoneFindTableCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  // found_table = true;
  
  // DynamicJsonBuffer dftBuffer;
  // JsonObject& dftobj = dftBuffer.createObject();
  // dftobj["st"] = "0";
  // dftobj ["act"] = "move";
  // string root2;
  // dftobj.printTo(root2);
  // String s;
  // s.data = root2;
  // cout<<"Find Table DONE: Preceed to Action"<<endl;
  // sched_pub.publish(s);
  cout<<"Find Table DONE: Preceed to Find PickDrop Point"<<endl;
  findPickDrop();
  

  return true;
}

//Server callback when checkpose done
bool DoneCheckPoseCallback(atlas80evo_msgs::SetONOFF::Request& request, atlas80evo_msgs::SetONOFF::Response& response)
{
    cout<<"PickDrop point OK: Proceed with PickDrop"<<endl;
    String param;
    param.data = armparam;
    arm_param_pub.publish(param);
    cout<<"Sending new parameter to Robotic arm: "<<armparam<<endl;
    //Duration(1).sleep();
    //pickup action to arm
    DynamicJsonBuffer dftBuffer;
    JsonObject& dftobj = dftBuffer.createObject();
    dftobj["st"] = "0";
    dftobj ["act"] = "move";
    string root2;
    dftobj.printTo(root2);
    String s;
    s.data = root2;
    cout<<"PickDrop Point Found: Updating new Arm parameter and PickDrop"<<endl;
    sched_pub.publish(s);


  // cout<<request<<endl;
  // if(request.data == true)
  // {
  //   //do pickdrop
  //   cout<<"PickDrop point OK: Proceed with PickDrop"<<endl;
  //   String param;
  //   param.data = armparam;
  //   arm_param_pub.publish(param);
  //   cout<<"Sending new parameter to Robotic arm: "<<armparam<<endl;
  //   //Duration(1).sleep();
  //   //pickup action to arm
  //   DynamicJsonBuffer dftBuffer;
  //   JsonObject& dftobj = dftBuffer.createObject();
  //   dftobj["st"] = "0";
  //   dftobj ["act"] = "move";
  //   string root2;
  //   dftobj.printTo(root2);
  //   String s;
  //   s.data = root2;
  //   cout<<"PickDrop Point Found: Updating new Arm parameter and PickDrop"<<endl;
  //   sched_pub.publish(s);
  // }
  // else
  // {
  //   //do table align call
  //   cout<<"PickDrop point NOK: Proceed with Table Alignment Call"<<endl;
  //   atlas80evo_msgs::SetONOFF srvft;
  //   srvft.request.data = true;
  //   findtableClient.call(srvft);
  // }

  return true;
}

void navactCallback(const std_msgs::String::ConstPtr &msg)//mission scheduler
{
  //cout<<"obj:"<<msstObj<<endl;
  // handling msg from navi or charge or arm:
  // {
  //   "st":"0",
  //   "to_y":"move/arm",
  // }
  string Data = msg->data.c_str();
  DynamicJsonBuffer jsonReadBufferNav;
  JsonObject& readObj = jsonReadBufferNav.parseObject(Data);
  DynamicJsonBuffer navBufferNav;
  JsonObject& nobj = navBufferNav.createObject();

  JsonArray& msstptxy = msstObj["activity"];
  msstptxy_ref = msstptxy;
  string lt="";
  string first_four="";
  int traynum= 0;
  //cout<<"in"<<endl; failure cause: success if to_x and to_y initialize according to payload
  if(readObj["st"]== "0")//if task done
  {
      // cout<<"in"<<endl; failure cause:success if readObj["st"] is set according to payload
      if (ms_cnt < ms_max) //check within mission list
      {
        // cout<<"in"<<endl; failure cause: 1<0 IS WRONG
        //cout<<ms_st<<endl; success output:__
          if(readObj["act"]== "arm")//check type what is done: if arm is done, do move
          {
            //If in charging mode
            
            if(charge=="1")
            {
              sos_trigger_response(2);
            }
            else
            {//before do next move, check pick/drop success or not

              lt = msstptxy [ms_cnt-1]["arm"]["cmd"].as<string>();

              first_four = lt.substr(0, 4);

              // if(first_four == "pick" || first_four == "drop")
              // {
              //   string tsid = lt.substr(4, sizeof(lt));
              //   traynum = atoi(tsid.c_str());

              //   if(first_four== "pick")
              //   {   
              //       cout<<"Checking tray AFTER pick...."<<endl;
              //       switch(traynum)
              //       {
              //         case 1:{
              //           if(Tray1=="1")
              //           {
              //             cout<<"Action "<<lt<<" Failed, Tray1 is still empty"<<endl;
              //             PickFail = true;
              //           }
              //           else
              //           {
              //             cout<<"Tray1 is occupied. Picking to Tray"<<traynum<<" success"<<endl;
              //           }
              //           break;
              //         }
              //         case 2:{
              //           if(Tray2=="1")
              //           {
              //             cout<<"Action "<<lt<<" Failed, Tray2 is still empty"<<endl;
              //             PickFail= true;
              //           }
              //           else
              //           {
              //             cout<<"Tray2 is occupied. Picking to Tray"<<traynum<<" success"<<endl;
              //           }
              //           break;
              //         }
              //         case 3:{
              //           if(Tray3=="1")
              //           {
              //             cout<<"Action "<<lt<<" Failed, Tray3 is still empty"<<endl;
              //             PickFail = true;
              //           }
              //           else
              //           {
              //             cout<<"Tray3 is occupied. Picking to Tray"<<traynum<<" success"<<endl;
              //           }
              //           break;
              //         }
              //         case 4:{
              //           if(Tray4=="1")
              //           {
              //             cout<<"Action "<<lt<<" Failed, Tray4 is still empty"<<endl;
              //             PickFail = true;
              //           }
              //           else
              //           {
              //             cout<<"Tray4 is occupied. Picking to Tray"<<traynum<<" success"<<endl;
              //           }
              //           break;
              //         }
              //       }           
              //   }
              //   else if(first_four=="drop")
              //   {
              //       cout<<"Checking tray AFTER drop...."<<endl;
              //       switch(traynum)
              //       {
              //         case 1:{
              //           if(Tray1=="0")
              //           {
              //             cout<<"Action "<<lt<<" Failed, Tray1 is still occupied"<<endl;
              //             DropFail = true;
              //           }
              //           else
              //           {
              //             cout<<"Tray1 is empty. Droping from Tray"<<traynum<<" success"<<endl;
              //           }
              //           break;
              //         }
              //         case 2:{
              //           if(Tray2=="0")
              //           {
              //             cout<<"Action "<<lt<<" Failed, Tray2 is still occupied"<<endl;
              //             DropFail = true;
              //           }
              //           else
              //           {
              //             cout<<"Tray2 is empty. Droping from Tray"<<traynum<<" success"<<endl;
              //           }
              //           break;
              //         }
              //         case 3:{
              //           if(Tray3=="0")
              //           {
              //             cout<<"Action "<<lt<<" Failed, Tray3 is still occupied"<<endl;
              //             DropFail = true;
              //           }
              //           else
              //           {
              //             cout<<"Tray3 is empty. Droping from Tray"<<traynum<<" success"<<endl;
              //           }
              //           break;
              //         }
              //         case 4:{
              //           if(Tray4=="0")
              //           {
              //             cout<<"Action "<<lt<<" Failed, Tray4 is still occupied"<<endl;
              //             DropFail = true;
              //           }
              //           else
              //           {
              //             cout<<"Tray4 is empty. Droping from Tray"<<traynum<<" success"<<endl;
              //           }
              //           break;
              //         }
              //       }
              //   }
              // }

              if(STATE != "DELIVERY" && cur_state != "DELIVERY")
              {
                callstate("DELIVERY");
              }
              string mx = msstptxy [ms_cnt]["x"].as<string>();
              string my = msstptxy [ms_cnt]["y"].as<string>();

              if(mx!=ms_x && my != ms_y)
              {
                ms_act="move";
                ms_seq = msstptxy [ms_cnt]["Seq"].as<string>();
                ms_pt = msstptxy [ms_cnt]["pt"].as<string>();
                ms_x_1 = ms_x;
                ms_y_1 = ms_y;
                ms_x = msstptxy [ms_cnt]["x"].as<string>();
                ms_y = msstptxy [ms_cnt]["y"].as<string>();
                ms_z = msstptxy [ms_cnt]["z"].as<string>();
                ms_w = msstptxy [ms_cnt]["w"].as<string>();
                //cout<<"in"<<endl; failure cause:__=="1" is wrong
                // cout<<nobj["map"]<<endl; success output:__ since msstObj output is []
                nobj["map"] = msstObj["fname"];
                nobj ["to_pt"] = ms_pt;
                nobj ["to_x"] = ms_x;
                nobj ["to_y"] = ms_y;
                nobj ["to_z"] = ms_z;
                nobj ["to_w"] = ms_w;

                string root2;
                nobj.printTo(root2);
                // cout<<root2<<endl; success output:{"map":,"to_x":"","to_y":""}
                String s;
                s.data = root2;
                // cout<<s; success output: data: {"map":,"to_x":"","to_y":""}
                cout<<"Go To: "<<nobj["to_pt"]<<endl;
                nav_pub.publish(s);
              }
              else
              {
                ms_act="skip move";
                cout<<"Skipping Moving at same location.."<<endl;
                ms_seq = msstptxy [ms_cnt]["Seq"].as<string>();
                ms_act = "move";
                move_skip = true;
              }
            }
          }//if(action arm done)
          //Done move: do action pick/drop/wait/find_charger/suspend
          if(readObj["act"]== "move" || move_skip == true)
          {
            if(charge=="1")//if in charging mode do find charger
            {
              if(STATE != "CHARGING" && cur_state != "CHARGING")
              {
                callstate("CHARGING");
              }
              if(cur_obj_detected=="2.0")
              {
                cout<<"Charger Found: Docking to charger now.."<<endl;
                atlas80evo_msgs::SetONOFF srvfc;
                srvfc.request.data = true;
                findchargerClient.call(srvfc);//service call to find charger
              }
              else
              {
                NoChargerError = true;
                cout<<"Charger Not Found"<<endl;
              }
            }//if charge == "1"
            else
            {
              ms_a = msstptxy [ms_cnt]["arm"].as<string>();
              if(ms_a=="gohome")
              {
                ms_act="gohome";
                cout<<"Entering Charger House"<<endl;
                
                //uncomment this for in charger ending
                // if(STATE != "CHARGING" && cur_state != "CHARGING")
                // {
                //   callstate("CHARGING");
                // }
                // atlas80evo_msgs::SetONOFF srvfc;
                // srvfc.request.data = true;
                // findchargerClient.call(srvfc);//service call to find charger

                ms_cnt ++;
                move_skip = false;

                //comment this for in charger ending
                String s;
                s.data = "{\"st\":\"0\",\"act\":\"arm\"}";
                sched_pub.publish(s);
              }
              else if(ms_a=="wait")
              {
                ms_act="wait";
                cout<<"Waiting at: "<<msstptxy [ms_cnt]["pt"].as<string>()<<endl;
                String zw;
                zw.data="wait";
                skip_pub.publish(zw);
                ms_cnt ++;
                move_skip = false;
              }
              else if(ms_a=="suspend")
              {
                ms_act="suspend";
                cout<<"Suspending at: "<<msstptxy [ms_cnt]["pt"].as<string>()<<endl;
                String zs;
                suspend = true;
                zs.data="suspend";
                skip_pub.publish(zs);
                ms_cnt ++;
                move_skip = false;
              }
              else//arm action
              {
                if(STATE != "PICKDROP" && cur_state != "PICKDROP")
                {
                  callstate("PICKDROP");
                }
                //check find table or not
                if(found_table)
                {
                  DynamicJsonBuffer meReadBuffer;
                  JsonObject& armObj = meReadBuffer.parseObject(ms_a);

                  lt = armObj["cmd"].as<string>();
                  ms_act=lt;
                  string first_four = lt.substr(0, 4);
                  string tsid = lt.substr(4, sizeof(lt));
                  traynum = atoi(tsid.c_str());

                  // if(first_four == "pick")
                  // {
                  //     cout<<"Checking tray BEFORE pick...."<<endl;
                  //     switch(traynum)
                  //     {
                  //       case 1:{
                  //         if(Tray1=="1")
                  //         {
                  //           cout<<"Pick Error:Unable to "<<lt<<", Tray1 is occupied"<<endl;
                  //           PickError = true;
                  //         }
                  //         else
                  //         {
                  //           cout<<"Tray1 is empty. Picking to Tray"<<traynum<<" at: "<<msstptxy [ms_cnt]["pt"].as<string>()<<endl;
                  //         }
                  //         break;
                  //       }
                  //       case 2:{
                  //         if(Tray2=="1")
                  //         {
                  //           cout<<"Pick Error:Unable to "<<lt<<", Tray2 is occupied"<<endl;
                  //           PickError = true;
                  //         }
                  //         else
                  //         {
                  //           cout<<"Tray2 is empty. Picking to Tray"<<traynum<<" at: "<<msstptxy [ms_cnt]["pt"].as<string>()<<endl;
                  //         }
                  //         break;
                  //       }
                  //       case 3:{
                  //         if(Tray3=="1")
                  //         {
                  //           cout<<"Pick Error:Unable to "<<lt<<", Tray3 is occupied"<<endl;
                  //           PickError = true;
                  //         }
                  //         else
                  //         {
                  //           cout<<"Tray3 is empty. Picking to Tray"<<traynum<<" at: "<<msstptxy [ms_cnt]["pt"].as<string>()<<endl;
                  //         }
                  //         break;
                  //       }
                  //       case 4:{
                  //         if(Tray4=="1")
                  //         {
                  //           cout<<"Pick Error:Unable to "<<lt<<", Tray4 is occupied"<<endl;
                  //           PickError = true;
                  //         }
                  //         else
                  //         {
                  //           cout<<"Tray4 is empty. Picking to Tray"<<traynum<<" at: "<<msstptxy [ms_cnt]["pt"].as<string>()<<endl;
                  //         }
                  //         break;
                  //       }
                  //     }           
                  // }
                  // else if(first_four=="drop")
                  // {
                  //     cout<<"Checking tray BEFORE drop...."<<endl;
                  //     switch(traynum)
                  //     {
                  //       case 1:{
                  //         if(Tray1=="0")
                  //         {
                  //           cout<<"Drop Error:Unable to "<<lt<<", Tray1 is empty"<<endl;
                  //           DropError = true;
                  //         }
                  //         else
                  //         {
                  //           cout<<"Tray1 is occupied. Droping from Tray"<<traynum<<"at: "<<msstptxy [ms_cnt]["pt"].as<string>()<<endl;
                  //         }
                  //         break;
                  //       }
                  //       case 2:{
                  //         if(Tray2=="0")
                  //         {
                  //           cout<<"Drop Error:Unable to "<<lt<<", Tray2 is empty"<<endl;
                  //           DropError = true;
                  //         }
                  //         else
                  //         {
                  //           cout<<"Tray2 is occupied. Droping from Tray"<<traynum<<"at: "<<msstptxy [ms_cnt]["pt"].as<string>()<<endl;
                  //         }
                  //         break;
                  //       }
                  //       case 3:{
                  //         if(Tray3=="0")
                  //         {
                  //           cout<<"Drop Error:Unable to "<<lt<<", Tray3 is empty"<<endl;
                  //           DropError = true;
                  //         }
                  //         else
                  //         {
                  //           cout<<"Tray3 is occupied. Droping from Tray"<<traynum<<"at: "<<msstptxy [ms_cnt]["pt"].as<string>()<<endl;
                  //         }
                  //         break;
                  //       }
                  //       case 4:{
                  //         if(Tray4=="0")
                  //         {
                  //           cout<<"Drop Error:Unable to "<<lt<<", Tray4 is empty"<<endl;
                  //           DropError = true;
                  //         }
                  //         else
                  //         {
                  //           cout<<"Tray4 is occupied. Droping from Tray"<<traynum<<"at: "<<msstptxy [ms_cnt]["pt"].as<string>()<<endl;
                  //         }
                  //         break;
                  //       }
                  //     }
                  // }

                  //if(PickError==false && DropError==false)//if no error at all
                  //{
                    String ss;
                    ss.data = ms_a;
                    // cout<<ss<<endl; success //output: data:{"agv_ID":"1","s":"2"}
                    arm_pub.publish(ss);
                    cur_arm = ms_a;
                    ms_cnt ++;
                    move_skip = false;
                    found_table = false;
                  //}
                  // else
                  // {//repeat action
                  //   String s;
                  //   s.data = "{\"st\":\"0\",\"act\":\"navi\"}";
                  //   sched_pub.publish(s);
                  // }
                }
                else
                {
                  // findPickDrop();
                  if(move_skip == false)
                  {
                    findTable();
                  }
                  else
                  {
                    cout<<"Delaying the next PICKDROP..."<<endl;
                    same_loc_delay = Time::now();

                    String xms;
                    xms.data = "{\"st\":\"1\",\"act\":\"bypass\"}";
                    sched_pub.publish(xms);
                    
                  }
                }                

              }
            }//if charge NOT "1"
            
          }//if(move DONE)
      }
      else if (ms_cnt == ms_max)//reset all when mission finished
      {
        // cout<<"in"<<endl; total failure no output
        cout<<"ALL DONE"<<endl;
        cur_arm = "";
        ms_cnt = 0;
        ms_id = "";
        ms_seq = "";
        ms_pt = "";
        ss_id = "";
        sched_id = "";
        ms_act = "done";
        msstObj = 0;
        ms_x="";
        ms_y="";
        ms_x_1 = "";
        ms_y_1 = "";
        firstNavi = true;
        found_table = false;
        if(STATE != "STANDBY" && cur_state != "STANDBY")
        {
          callstate("STANDBY");
        }

      }//check current mission number

  }//Check task done
  else //if st == 1
  {
    //do loop of this callback till certain amount of time
    Duration diff_dly=Time::now()-same_loc_delay;
    cout<<"Delay Time:"<<diff_dly.toSec()<<endl;   
    if(diff_dly.toSec() > 5)
    {
      found_table = true;
      DynamicJsonBuffer dftBuffer;
      JsonObject& dftobj = dftBuffer.createObject();
      dftobj["st"] = "0";
      dftobj ["act"] = "move";
      string root2;
      dftobj.printTo(root2);
      String s;
      s.data = root2;
      cout<<"Same Point Using Old Arm parameter and PickDrop"<<endl;
      sched_pub.publish(s);
    }
    else
    {
      String xm;
      xm.data = "{\"st\":\"1\",\"act\":\"bypass\"}";
      sched_pub.publish(xm);
    }
  }

}

void srCallback(const std_msgs::String::ConstPtr &msg)//FMS SR
{
  //{"agv_ID":"1", "cmd" :"1"}
  string Data = msg->data.c_str();
  DynamicJsonBuffer jsonReadBufferSr;
  JsonObject& readObj = jsonReadBufferSr.parseObject(Data);
  cout << "FMS SR REQ" << endl; //success output:FMS SR REQ
  // cout<<readObj["agv_ID"]<<endl; success output: 1
  if(readObj["agv_ID"]==AGV_ID)
  {
    // cout<<"in"<<endl; success output:in
    string text = readObj["cmd"];
    if (text == "1")
    {
      //cout<<text<<endl; //success output:1
      suspend = !suspend;
    }
    else if(text == "0")
    {
      suspend = !suspend;
      callstate(before_state);
    }
  }
}

void obstacleCallback(const std_msgs::String::ConstPtr &msg)
{
  string Data = msg->data.c_str();
  if(Data == "Obstacle")
  {
    if(stopobs==false)
    {
      obs_t = Time::now();
    }
  }
}

void ledpub()
{
  String strt;
  strt.data = sr_led;
  led_pub.publish(strt);

}

void state_Callback(const atlas80evo_msgs::FSMState &msg)
{
 STATE = msg.state;
 // cout<<"--------------------------------"<<STATE<<endl;
}




int main(int argc, char **argv)
{

  ros::init(argc, argv, "fms_handler");
  ros::NodeHandle nh;

  sr_break.linear.x = sr_break.linear.y = sr_break.linear.z = 0.0;
  sr_break.angular.x = sr_break.angular.y = sr_break.angular.z = 0.0;

  //mission start (/f2a/ms/st) is listen directly by mission handler package;
  //arm movement data (/f2a/sys/arm) is listen directly by arm;
  //arm movement data (/f2a/sys/mv) is listen directly by agv;
  ros::Subscriber state = nh.subscribe("/fsm_node/state", 1000, state_Callback);
  ros::Subscriber sys_sos = nh.subscribe("/f2a/sys/sos", 1000, sys_sos_Callback);
  ros::Subscriber sys_off = nh.subscribe("/f2a/sys/off", 1000, sys_off_Callback);
  ros::Subscriber sys_diag = nh.subscribe("/f2a/sys/diag", 1000, sys_diag_Callback);
  ros::Subscriber healthSubs = nh.subscribe("/low_level/status", 1000, agv_health_pub);
  ros::Subscriber moveSubs = nh.subscribe("/f2a/sys/mv", 1000, mvSysCallback);
  ros::Subscriber srtSubs = nh.subscribe("/led/status", 1000, nanocallback);
  ros::Subscriber srSubs = nh.subscribe("/f2a/ms/sr", 1000, srCallback);
  ros::Subscriber msstSubs = nh.subscribe("/f2a/ms/st", 1000, msstCallback);
  ros::Subscriber navactSubs = nh.subscribe("/a2a/ms/hdl", 1000, navactCallback);
  ros::Subscriber gpsSubs = nh.subscribe("/vectornav/GPS", 5, gpsCallback);
  ros::Subscriber traySubs = nh.subscribe("/a2f/arm/tray", 1000, trayCallback);
  ros::Subscriber tcamSubs = nh.subscribe("/a2a/tcam/raw", 1000, tcamCallback);
  ros::Subscriber obstacleSubs = nh.subscribe("obstacle_stop/debug", 1000, obstacleCallback);

  ros::Subscriber lidar3Subs = nh.subscribe("/os1_cloud_node/points", 1000, lidar3Callback);//listen 3D lidar
  ros::Subscriber lidar2fSubs = nh.subscribe("/cloud1", 1000, lidar2fCallback);//listen 2D Lidar f
  ros::Subscriber lidar2rSubs = nh.subscribe("/cloud2", 1000, lidar2rCallback);//listen 2D Lidar r
  ros::Subscriber camSubs = nh.subscribe("/camera/color/image_raw", 1000, camCallback);//listen camera
  ros::Subscriber obuSubs = nh.subscribe("/a2a/obu", 1000, obuCallback);//listen obu
  ros::Subscriber armSubs = nh.subscribe("/a2a/arm/live", 1000, armCallback);//listen arm location
  ros::Subscriber Manstart = nh.subscribe("/f2a/sys/man", 1000, TeleOPCallback); //Teleopstart
  ros::Subscriber objrstart = nh.subscribe("/a2a/object/detection", 1000, ObjectRecogCallback); //Status from tower object detection
  ros::Subscriber ptsSubs = nh.subscribe("/amcl_pose", 1000, ptsCallback);

  shutdownClient = nh.serviceClient<std_srvs::Empty>("/shutdown");
  findpickdropClient = nh.serviceClient<atlas80evo_msgs::SetONOFF>("/find_pickdrop");
  findchargerClient = nh.serviceClient<atlas80evo_msgs::SetONOFF>("/charging/call");
  playsound = nh.serviceClient<atlas80evo_msgs::SetSound>("/sound/call");
  findoutchargerClient = nh.serviceClient<atlas80evo_msgs::SetONOFF>("/leaving/call");
  stateClient = nh.serviceClient<atlas80evo_msgs::SetFSMState>("/fsm_node/set_state");
  findtableClient = nh.serviceClient<atlas80evo_msgs::SetONOFF>("/aligning/call");
  checkposeClient = nh.serviceClient<atlas80evo_msgs::SetPose2D>("/check/pose");

  ServiceServer donecheckpose_serv = nh.advertiseService("/check/reply", DoneCheckPoseCallback);
  ServiceServer donepickdrop_serv = nh.advertiseService("/find_pickdrop/goal", DoneFindPickupCallback);
  ServiceServer donecharger_serv = nh.advertiseService("charging/done", DoneFindChargingDockCallback);
  ServiceServer doneoutcharger_serv = nh.advertiseService("/leaving/done", DoneFindOutChargingDockCallback);
  ServiceServer donetable_serv = nh.advertiseService("/aligning/done", DoneFindTableCallback);


  reg_pub = nh.advertise<String>("/pd_state", 1000);
  sched_pub = nh.advertise<String>("/a2a/ms/hdl", 1000);
  sos_pub = nh.advertise<String>("/a2f/sys/sos", 1000);
  skip_pub = nh.advertise<String>("/a2a/ms/skip", 1000);
  arm_pub = nh.advertise<String>("/f2a/arm/mv", 1000);
  nav_pub = nh.advertise<String>("/a2a/nav", 1000);
  led_pub = nh.advertise<String>("/a2a/led", 1000);
  health_pub = nh.advertise<String>("/a2f/ms/ht", 1000);
  diag_pub = nh.advertise<String>("/a2f/sys/diag", 1000);
  arm_param_pub = nh.advertise<String>("/a2a/agv/loc", 1000);

  to_obu_pub = nh.advertise<String>("/a2a/to_obu", 1000);
  sr_break_pub = nh.advertise<geometry_msgs::Twist>("/twist_cmd_mux/input/suspend", 10);
  mv_pub = nh.advertise<geometry_msgs::Twist>("/twist_cmd_mux/input/webop", 1000);
  sus_pub = nh.advertise<String>("/a2a/led",1000);
  armxtra_pub = nh.advertise<String>("/a2a/arm/xtra",10);
  sysled_pub = nh.advertise<String>("/a2a/sysled",10);

  
  // srv.request.path2file = "/home/endstruct2/catkin_ws/src/atlas80evo/sounds/stand.wav";
  // srv.request.channel = 1;
  // srv.request.volume = 1.0;
  // srv.request.loop = -1;
  // srv.request.interval = 0;
  // playsound.call(srv);

  Rate loop_rate(20);
  
  cout << ver << endl;

  health_t = Time::now();
  wait_t = Time::now();
  sus_t = Time::now();
  waita_t = Time::now();
  alert_t = Time::now();
  cam_time = Time::now();
  same_loc_delay = Time::now();

  while(ok())
  {
    sos_check();
    generic_check();
    spinOnce();
    loop_rate.sleep();
  }
  return 0;
}