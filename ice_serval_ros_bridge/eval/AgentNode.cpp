/*
 * KnowledgeBaseNode.cpp
 *
 *  Created on: Apr 11, 2016
 *      Author: sni
 */


#include <chrono>
#include <ros/ros.h>
#include <signal.h>
#include "easylogging++.h"
#include "IceServalBridge.h"
#include <ice/information/InformationStore.h>
#include <ice/communication/jobs/InformationRequest.h>

INITIALIZE_EASYLOGGINGPP


std::chrono::time_point<std::chrono::system_clock> start;
std::shared_ptr<ice::IceServalBridge> node;
ofstream file;
std::shared_ptr<ice::Entity> mops;

void callback(std::shared_ptr<ice::InformationRequest> request)
{
  static int count = 0;

  auto end = std::chrono::system_clock::now();
  auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

  sleep(10);

  auto &traffic = node->getCommunicationInterface()->getTraffic();
  unsigned long long trafficSum = traffic.sendBytes + traffic.receivedBytes;

  if (count++ < 100)
  {
    fprintf(stderr, "%3d: %7d ms, %5d lost, %12d b, %5d send, %5d received\n",
            count, dur, request->getResendCount(), trafficSum, traffic.messageSendCount, traffic.messageReceivedCount);
//    std::cerr << count << ":\t\t" << dur << "\t" << request->getResendCount() << std::endl;

    file << dur << "\t" << request->getResendCount() << "\t"
        << trafficSum << "\t" << traffic.messageSendCount << "\t" << traffic.messageReceivedCount << std::endl;

    auto spec = std::make_shared<ice::InformationSpecification>("*",
                                                                  "http://vs.uni-kassel.de/TurtleBot#ChargeStation",
                                                                  "http://vs.uni-kassel.de/Ice#Position",
                                                                  "http://vs.uni-kassel.de/TurtleBot#RelativeToLandmark",
                                                                  "http://vs.uni-kassel.de/TurtleBot#Landmark");

    auto r = std::make_shared<ice::InformationRequest>(node, mops);
    r->setTimeout(3000);
    r->getRequests().push_back(spec);
    r->setCallbackFinished(&callback);

    node->getCommunicationInterface()->resetTraffic();
    start = std::chrono::system_clock::now();
    node->getCommunicationInterface()->addComJob(r);
  }
  else
  {
    std::cerr << "#########################################################################" << std::endl;
    std::cerr << "Finished " << dur << std::endl;
    std::cerr << "#########################################################################" << std::endl;
    file.flush();
    file.close();
  }
}

void mySigintHandler(int sig)
{
  node->cleanUp();
  ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "agent_eval");

  ros::NodeHandle nh_("");
  ros::NodeHandle pnh_("~");

  node = std::make_shared<ice::IceServalBridge>(nh_, pnh_);
  node->init();
  signal(SIGINT, mySigintHandler);

  // stuff
  sleep(5);
  mops = node->getEntityDirector()->lookup(ice::EntityDirectory::ID_ONTOLOGY, "http://vs.uni-kassel.de/TurtleBot#Mops", false);

  if(mops == nullptr)
  {
    std::cerr << "Mops not found!" << std::endl;
    node->cleanUp();
    ros::shutdown();
    return 1;
  }

  file.open("/tmp/time.log");

  auto spec = std::make_shared<ice::InformationSpecification>("*",
                                                              "http://vs.uni-kassel.de/TurtleBot#ChargeStation",
                                                              "http://vs.uni-kassel.de/Ice#Position",
                                                              "http://vs.uni-kassel.de/TurtleBot#RelativeToLandmark",
                                                              "http://vs.uni-kassel.de/TurtleBot#Landmark");

  auto request = std::make_shared<ice::InformationRequest>(node, mops);
  request->setTimeout(3000);
  request->getRequests().push_back(spec);
  request->setCallbackFinished(&callback);


  std::cerr << "#########################################################################" << std::endl;
  std::cerr << "Start" << std::endl;
  std::cerr << "#########################################################################" << std::endl;

  node->getCommunicationInterface()->resetTraffic();
  start = std::chrono::system_clock::now();
  node->getCommunicationInterface()->addComJob(request);

  ros::spin();

  return 0;
}
