#include <list>
#include <map>
#include <sstream>


#include <aspect/blackboard.h>
#include <blackboard/interface_listener.h>
#include <blackboard/remote.h>
#include <config/netconf.h>
#include <navgraph/navgraph.h>
#include <netcomm/fawkes/client.h>
#include <protobuf_comm/server.h>
#include <protobuf_comm/frame_header.h>
#include <protobuf_comm/client.h>
#include <protobuf_comm/server.h>
#include <protobuf_comm/peer.h>


#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/format.hpp>
#include "boost/assign.hpp" //TODO check


#include "ros/ros.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"


#include <interfaces/NavGraphGeneratorInterface.h>
#include <interfaces/NavGraphWithMPSGeneratorInterface.h>

#include <interfaces/Position3DInterface.h>
#include <interfaces/RobotinoLightInterface.h>
#include <interfaces/SkillerInterface.h>
#include <interfaces/TagVisionInterface.h>
#include <interfaces/ZoneInterface.h>

#include <rcll_adapter/Position3D.h>
#include <rcll_adapter/RobotinoLight.h>
#include <rcll_adapter/SkillStatus.h>
#include <rcll_adapter/TagVision.h>
#include <rcll_adapter/ZoneSearch.h>

using namespace fawkes;
using namespace std;
using namespace boost::assign;

/*
  To add another Interface Listener:
  1- Make sure the Class is in the make file to be found components
  2- (optional) Create a new msg. (.msg file and global object)
  3- Include both the interface and the new msg in this class
  4- Add interface info in createInterfacesMap and create a Handler Method , similar to the ones in bb_interface_data_changed
*/

typedef struct {
    string type;
    ros::Publisher publisher;
    fawkes::Interface *interface;
} InterfaceInfo;

bool contains(const std::vector<string> &vec, const string &value)
{
    return std::find(vec.begin(), vec.end(), value) != vec.end();
}
  class FawkesConnector  : public fawkes::BlackBoardInterfaceListener , public fawkes::BlackBoardAspect
  {
  public:
    FawkesConnector(int argc, char **argv);
    ~FawkesConnector();
    void openFawkesConnection(std::string fawkes_host,int fawkes_port);
    map<string, InterfaceInfo>  createInterfacesMap();
    virtual void bb_interface_data_changed(fawkes::Interface *interface) throw();
    void handleRobotinoLight(fawkes::Interface *interface, std::string id);
    void handleSkillStatus(fawkes::Interface *interface, std::string id);
    void handleZoneSearch(fawkes::Interface *interface, std::string id);
    void handlePosition3D(fawkes::Interface *interface, std::string id);
    void handleTagVision(fawkes::Interface *interface, std::string id);
  private:
    rcll_adapter::Position3D getPosition3DMessage(Position3DInterface *i,std_msgs::Header oldHeader);
  public:
    RemoteBlackBoard *g_blackboard = NULL;
    map<string, InterfaceInfo> interface_id_to_info;
    ////////////////////////////////////////////////Rcll Messages////////////////////////////////////////////////
    rcll_adapter::SkillStatus msg_skill_status;
    rcll_adapter::RobotinoLight msg_robotino_light;
    rcll_adapter::ZoneSearch msg_zone_search;
    rcll_adapter::Position3D msg_position_3D;
    rcll_adapter::TagVision msg_tag_vision;
  };

  map<string, InterfaceInfo> FawkesConnector::createInterfacesMap()
  {
// strMap["/navgraph-generator-mps"] = {"NavGraphWithMPSGeneratorInterface", n.advertise<rcll_adapter::Position3DInterface>("Position3D", bfr), NULL};
    ros::NodeHandle n;
    int bfr=1000;//buffer
    map<string, InterfaceInfo> interface_id_to_info_tmp;
   
  //interface_id_to_info[id]                            = {type,            Publisher                              TopicName        , interface}; 
    interface_id_to_info_tmp["/explore-zone/info"]      = {"ZoneInterface", n.advertise<rcll_adapter::ZoneSearch>("rcll_ZoneSearch", bfr), NULL};
    interface_id_to_info_tmp["/explore-zone/pose"]      = {"Position3DInterface", n.advertise<rcll_adapter::Position3D>("rcll_Position3D", bfr), NULL};
    interface_id_to_info_tmp["Light determined"]        = {"RobotinoLightInterface", n.advertise<rcll_adapter::RobotinoLight>("rcll_RobotinoLight", bfr), NULL}; 
    interface_id_to_info_tmp["Skiller"]                 = {"SkillerInterface", n.advertise<rcll_adapter::SkillStatus>("rcll_SkillStatus", bfr), NULL};
    
   
    ros::Publisher tagVisionPublisher    = n.advertise<rcll_adapter::TagVision>("rcll_TagVision", bfr);
    InterfaceInfo tagVisionStruct = {"Position3DInterface", tagVisionPublisher, NULL};

    interface_id_to_info_tmp["/tag-vision/info"]        = {"TagVisionInterface", tagVisionPublisher, NULL}; 
    interface_id_to_info_tmp["/tag-vision/0"]           = tagVisionStruct;
    interface_id_to_info_tmp["/tag-vision/1"]           = tagVisionStruct;
    interface_id_to_info_tmp["/tag-vision/2"]           = tagVisionStruct;
    interface_id_to_info_tmp["/tag-vision/3"]           = tagVisionStruct;
    interface_id_to_info_tmp["/tag-vision/4"]           = tagVisionStruct;
    interface_id_to_info_tmp["/tag-vision/5"]           = tagVisionStruct;
    interface_id_to_info_tmp["/tag-vision/6"]           = tagVisionStruct;
    interface_id_to_info_tmp["/tag-vision/7"]           = tagVisionStruct;
    interface_id_to_info_tmp["/tag-vision/8"]           = tagVisionStruct;
    interface_id_to_info_tmp["/tag-vision/9"]           = tagVisionStruct;
    interface_id_to_info_tmp["/tag-vision/10"]          = tagVisionStruct;
    interface_id_to_info_tmp["/tag-vision/11"]          = tagVisionStruct;
    interface_id_to_info_tmp["/tag-vision/12"]          = tagVisionStruct;
    interface_id_to_info_tmp["/tag-vision/13"]          = tagVisionStruct;
    interface_id_to_info_tmp["/tag-vision/14"]          = tagVisionStruct;
    interface_id_to_info_tmp["/tag-vision/15"]          = tagVisionStruct;

    return interface_id_to_info_tmp;
  }

  FawkesConnector::FawkesConnector(int argc, char **argv):BlackBoardInterfaceListener("")
  {
    ros::init(argc, argv, "talker");
    interface_id_to_info=createInterfacesMap();
  }
 
  void FawkesConnector::openFawkesConnection(std::string fawkes_host,int fawkes_port)
  {
    FawkesNetworkClient *c =  new FawkesNetworkClient(fawkes_host.c_str(),(unsigned short) fawkes_port);
    c->connect();
    g_blackboard = new RemoteBlackBoard(c);
    try {
       for (map<string, InterfaceInfo>::iterator p = interface_id_to_info.begin();
          p != interface_id_to_info.end( ); ++p ) {
          std::string id = p->first;
          InterfaceInfo interfaceInfo=p->second;
          std::string type=interfaceInfo.type;
          Interface *iface = g_blackboard->open_for_reading(type.c_str(), id.c_str());
          bbil_add_data_interface(iface);
          interfaceInfo.interface=iface;
        }
        g_blackboard->register_listener(this, BlackBoard::BBIL_FLAG_DATA);
      }
      catch (Exception &e) {
        fprintf(stderr, "Failed to listen to all Interfaces %s:", e.what_no_backtrace());
      }
  }

  void FawkesConnector::bb_interface_data_changed(fawkes::Interface *interface) throw()
  {
    std::string id = interface->id();
    if(id.compare("/explore-zone/info")==0){
      handleZoneSearch(interface,id);
    } else if(id.compare("Light determined")==0){
      handleRobotinoLight(interface,id);
    } else if(id.compare("Skiller")==0){
      handleSkillStatus(interface,id);
    }else if(id.compare("/explore-zone/pose")==0){
      handlePosition3D(interface,id);
    }else if(id.find_last_of("//")!=string::npos && id.substr(0,id.find_last_of("//")).compare("/tag-vision")==0){
      handleTagVision(interface,id);
    } else {
      //Something really unexpected happend if we come here
    }
    //contains({ "","",.. },id) could be also used if more than one interface publishes same message
  }
  void FawkesConnector::handleZoneSearch(fawkes::Interface *interface, std::string id)
  {
    ZoneInterface *i = static_cast<ZoneInterface *>(interface);
    i->read();
    msg_zone_search.search_state=i->search_state();
    msg_zone_search.tag_id=i->tag_id();
    ros::Publisher p= ((InterfaceInfo)interface_id_to_info[id]).publisher;
    p.publish(msg_zone_search);
  }

  void FawkesConnector::handleSkillStatus(fawkes::Interface *interface, std::string id)
  {
    
    SkillerInterface *i = static_cast<SkillerInterface *>(interface);
    i->read();
    msg_skill_status.error=i->error();
    msg_skill_status.exclusive_controller=i->exclusive_controller();
    msg_skill_status.status=i->status();
    msg_skill_status.skill_string=i->skill_string();
    msg_skill_status.stamp=  ros::Time::now();
    ros::Publisher p= ((InterfaceInfo)interface_id_to_info[id]).publisher;
    p.publish(msg_skill_status);
  }

  void FawkesConnector::handleRobotinoLight(fawkes::Interface *interface, std::string id)
  {
    RobotinoLightInterface *i = static_cast<RobotinoLightInterface *>(interface);
    i->read();
    msg_robotino_light.red=i->red();
    msg_robotino_light.yellow=i->yellow();
    msg_robotino_light.green=i->green();
    msg_robotino_light.visibility_history=i->visibility_history();
    msg_robotino_light.ready=i->is_ready();
    ros::Publisher p= ((InterfaceInfo)interface_id_to_info[id]).publisher;
    p.publish(msg_robotino_light);
  }

  rcll_adapter::Position3D FawkesConnector::getPosition3DMessage(Position3DInterface *i,std_msgs::Header oldHeader)
  {
    rcll_adapter::Position3D msg_pos_3D;
    geometry_msgs::PoseWithCovarianceStamped poseWithCovarianceStamped;
    std_msgs::Header header=oldHeader; //used this way for the sequence number
    //////////////////assigning the pose ///////////////////////////////
    geometry_msgs::PoseWithCovariance poseWithCovariance;
    geometry_msgs::Pose pose;
    geometry_msgs::Point point;
    geometry_msgs::Quaternion quaternion;
    point.x=i->translation(0);
    point.y=i->translation(1);
    point.z=i->translation(2);
    quaternion.x=i->rotation(0);
    quaternion.y=i->rotation(1);
    quaternion.z=i->rotation(2);
    quaternion.w=i->rotation(3);
    pose.position=point;
    pose.orientation=quaternion;
    poseWithCovariance.pose=pose;

    //////////////////assigning the covariance ///////////////////////////////
    boost::array<double,36ul>  covariance;
    for(uint j=0;j<covariance.size();j++)covariance[j]=i->covariance(j);
    poseWithCovariance.covariance=covariance;
    poseWithCovarianceStamped.pose=poseWithCovariance;

    //////////////////assigning the header ///////////////////////////////
    header.seq=header.seq+1;
    header.stamp= ros::Time::now();
    header.frame_id=i->frame();
    poseWithCovarianceStamped.header=header;

    msg_pos_3D.pose_with_cov_stamped=poseWithCovarianceStamped;
    msg_pos_3D.visibility_history=i->visibility_history();
    return msg_pos_3D;
  }

  void FawkesConnector::handlePosition3D(fawkes::Interface *interface, std::string id)
  {
    Position3DInterface *i = static_cast<Position3DInterface *>(interface);
    i->read();
    msg_position_3D=getPosition3DMessage(i,msg_position_3D.pose_with_cov_stamped.header);
    ros::Publisher p= ((InterfaceInfo)interface_id_to_info[id]).publisher;
    p.publish(msg_position_3D);
  }

  void FawkesConnector::handleTagVision(fawkes::Interface *interface, std::string id)
  {
    if(id.compare("/tag-vision/info")==0)
    {
      TagVisionInterface *i = static_cast<TagVisionInterface *>(interface);
      i->read();
      std_msgs::Header header=msg_tag_vision.header;
      header.seq=header.seq+1;
      header.stamp= ros::Time::now();
      header.frame_id=i->frame();
      msg_tag_vision.header=header;
      msg_tag_vision.tags_visible=i->tags_visible();
      boost::array<int, 12ul> tag_id=msg_tag_vision.tag_id;
      for(uint j=0;j<tag_id.size();j++)tag_id[j]=i->tag_id(j);
      msg_tag_vision.tag_id=tag_id;
    }
    else
    {
      Position3DInterface *i = static_cast<Position3DInterface *>(interface);
      i->read();
      string idOfPosition = id.substr(id.find_last_of("//")+1,id.size());
      uint position = atoi(idOfPosition.c_str());
      rcll_adapter::Position3D msg_tag=getPosition3DMessage(i,msg_tag_vision.header);
      vector<rcll_adapter::Position3D> allPositions=msg_tag_vision.position;
      if(allPositions.size()>position)//TODO change
      {
        allPositions.at(position)=msg_tag;
      }
      else
      {
        allPositions.push_back(msg_tag);
      }
      msg_tag_vision.position=allPositions;
    }
    ros::Publisher p= ((InterfaceInfo)interface_id_to_info[id]).publisher;
    p.publish(msg_tag_vision);
  }

  FawkesConnector::~FawkesConnector()
  {
   for (map<string, InterfaceInfo>::iterator p = interface_id_to_info.begin();
          p != interface_id_to_info.end( ); ++p ) {
               InterfaceInfo interfaceInfo=p->second;
             g_blackboard->close(interfaceInfo.interface);
        }
  }

int main(int argc, char **argv)
{

  std::string fawkes_host = "localhost";
  int fawkes_port=1921;
  FawkesConnector *connector = new FawkesConnector(argc,argv);
  connector->openFawkesConnection(fawkes_host,fawkes_port);
  while (true)
  {
    sleep(1000);
  }
  return 0;
}