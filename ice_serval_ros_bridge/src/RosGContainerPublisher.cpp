/*
 * RosGMessagePublisher.cpp
 *
 *  Created on: Apr 22, 2016
 *      Author: sni
 */

#include "RosGContainerPublisher.h"

namespace ice
{

RosGContainerPublisher::RosGContainerPublisher()
{
  // TODO Auto-generated constructor stub

}

RosGContainerPublisher::~RosGContainerPublisher()
{
  // TODO Auto-generated destructor stub
}

bool RosGContainerPublisher::publish(std::shared_ptr<RequiredInfo> const &reqInfo, std::shared_ptr<GContainer> constainer)
{
  // TOTO gcontainer to message
  std::string message = "";

  return this->publish(reqInfo->topic, reqInfo->message, message);
}

bool RosGContainerPublisher::publish(std::string const &topic, std::string const &messageName, std::string const &message) {
    std::stringstream output;
    std::string command = "rostopic pub " + topic + " " + messageName + " \"" + message + "\"";
    std::shared_ptr<FILE> pipe(popen(command.c_str(), "r"), pclose);
    if (!pipe) return -1;
    char buffer[128];

    while (!feof(pipe.get())) {
        if (fgets(buffer, 128, pipe.get()) != NULL)
          output << buffer;
    }

    // todo

    output.seekg(0, std::ios::end);
    int size = output.tellg();
    output.seekg(0, std::ios::beg);

    return size;
}

} /* namespace ice */
