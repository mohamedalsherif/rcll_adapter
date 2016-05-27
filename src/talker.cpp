


//Wie soll ich mein CMakeLists.txt andern so dass das da kompiliert. 
#include <blackboard/remote.h>





#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}


/*


#include <navgraph/navgraph.h>
+#include <interfaces/NavGraphWithMPSGeneratorInterface.h>
+#include <interfaces/SkillerInterface.h>

void
+Ros_Faw::init()
+{
+  std::string    fawkes_host = "localhost";
+  int fawkes_port=1921;
+  c =  new FawkesNetworkClient(fawkes_host.c_str(),(unsigned short) fawkes_port);
+  c->connect();
+  g_blackboard = new RemoteBlackBoard(c);
+   fprintf(stderr, "Arrived in INIT\n");
+  try
+    {
+      std::string type="SkillerInterface";
+      std::string id = "Skiller";
+      Interface *iface = g_blackboard->open_for_reading(type.c_str(), id.c_str());
+   //navgen_if_=g_blackboard->open_for_reading<fawkes::SkillerInterface>("Skiller");
+      bbil_add_data_interface(iface);
+      g_blackboard->register_listener(this, BlackBoard::BBIL_FLAG_DATA);
+    }
+    catch (Exception &e) {
+      fprintf(stderr, "Failed to execute skill %s:", e.what_no_backtrace());
+    }
+     fprintf(stderr, "left in INIT\n");
+}

+  void
+  RobyProtobuf::bb_interface_data_changed(fawkes::Interface *interface) throw()
+  {
+    fprintf(stderr, "Data changed \n");
+  }


*/