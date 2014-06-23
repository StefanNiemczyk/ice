#include "ros/ros.h"
#include "std_msgs/String.h"

#include "ice/ICEngine.h"
#include "ice/information/InformationElement.h"
//#include "ice/InformationContainer.h"

#include "boost/shared_ptr.hpp"
#include "boost/uuid/uuid.hpp"
#include "boost/uuid/uuid_generators.hpp"

#include <exception>
#include <iostream>
#include <memory>


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback2(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  std::cout << "Start" << std::endl;
  std::shared_ptr<int> ip;
  std::shared_ptr<int> ip2;
/*
  std::cout << "Create Engine" << std::endl;
  boost::shared_ptr<ice::ICEngine> engine(new ice::ICEngine());
  std::cout << "Create Engine Done" << std::endl;

  std::cout << "Create Buffer" << std::endl;
  boost::shared_ptr<ice::InformationContainer<int> > buffer = engine->registerInformation<int>("info 1");
  std::cout << "Create Buffer Done, size " << buffer->getMaxBuffer() << std::endl;

  std::cout << "Create Information" << std::endl;
  for (int i = 0; i < 7; ++i)
  {
    boost::shared_ptr<ice::InformationElement<int> > information(new ice::InformationElement<int>(buffer->getUUID(), i));
    int identifier = buffer->add(information);
    std::cout << "\tInformation " << identifier << " : " << i << std::endl;
  }
  std::cout << "Done Information" << std::endl;

  std::cout << "Start reading" << std::endl;
  boost::shared_ptr<ice::InformationElement<int> > information;
  int i = 0;
  do
  {
    std::cout << "\tRead " << i << std::endl;
    information = buffer->getLast(i);

    if (information)
      std::cout << "\tInformation " << i++ << ", " << information->getInformation() << std::endl;
    else
      std::cout << "\tInformation " << i++ << ", null" << std::endl;
  } while (information);
  std::cout << "End reading" << std::endl;

  std::cout << "Start reset" << std::endl;
  buffer->reset(true);
  std::cout << "\tLast Information " << (buffer->getLast() ? "exists (bad)" : "not exists (ok)")  << std::endl;
  std::cout << "End reset" << std::endl;

  std::cout << "Start Buffer vector test" << std::endl;
  boost::shared_ptr< ice::InformationContainer<int> > buffer2 = engine->getContainer<int>("info 1");
  std::cout << "\tBuffer " << (buffer2 ? "found (ok)" : "not found (bad)")  << std::endl;

  boost::shared_ptr< ice::InformationContainer<int> > buffer3 = engine->getContainer<int>(1);
  std::cout << "\tBuffer " << (buffer3 ? "found (bad)" : "not found (ok)")  << std::endl;


  boost::shared_ptr<ice::InformationContainer<double> > buffer4 = engine->registerInformation<double>("info2");
  boost::shared_ptr< ice::InformationContainer<double> > buffer5 = engine->getContainer<double>(1);
  std::cout << "\tBuffer " << (buffer5 ? "found (ok)" : "not found (bad)")  << std::endl;

  try {
    boost::shared_ptr< ice::InformationContainer<int> > buffer6 = engine->getContainer<int>(1);
    std::cout << "\tBuffer " << (buffer6 ? "found (bad)" : "not found (ok)")  << std::endl;
  } catch (const std::bad_cast & e) {
    std::cout << "\tBuffer throws exception: " << e.what() << " (ok)" << std::endl;
  }
  std::cout << "Done Buffer vector test" << std::endl;
*/
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  //ros::init(argc, argv, "listener");
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  //ros::NodeHandle n;
  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  // ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback2);
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  // ros::spin();
  return 0;
}
