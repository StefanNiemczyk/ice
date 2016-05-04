/*
 * RosGMessagePublisher.cpp
 *
 *  Created on: Apr 22, 2016
 *      Author: sni
 */

#include "RosGContainerPublisher.h"

#include <locale>

#include "IceServalBridge.h"

namespace ice
{

RosGContainerPublisher::RosGContainerPublisher()
{
  _log = el::Loggers::getLogger("RosGContainerPublisher");
  _log->info("Constructor");

  setlocale(LC_ALL, "C");// TODO
}

RosGContainerPublisher::~RosGContainerPublisher()
{
}

bool RosGContainerPublisher::publish(std::shared_ptr<RequiredInfo> const &reqInfo, std::shared_ptr<GContainer> &container)
{
  // todo gcontainer to message
  std::string message = "x: {o2_XCoordinate}\ny: {o2_YCoordinate}\nz: {o2_ZCoordinate}";

  bool result = this->transformToMessage(message, container);

  if (false == result)
    return false;

  return this->publish(reqInfo->topic, reqInfo->message, message);
}

bool RosGContainerPublisher::transformToMessage(std::string &message, std::shared_ptr<GContainer> &container)
{
  int index = message.find("{");
  int index2 = 0;
  std::string pathStr, valueStr;

  while (index != std::string::npos)
  {
    index2 = message.find("}");
    pathStr = message.substr(index + 1, index2 - index - 1);

    auto path = container->representation->accessPath(pathStr);

    if (path == nullptr)
    {
      _log->error("Unknown path '%v' in container of representation '%v'", pathStr, container->representation->name);
      return false;
    }

    auto pair = container->getPair(path);

    switch (pair.first)
    {
      case UNSET:
        _log->error("Unknown path '%v' in container of representation '%v'", pathStr, container->representation->name);
        return false;
        break;
      case BOOL:
        valueStr = *((bool*) pair.second) ? "true" : "false";
        break;
      case BYTE:
        valueStr = std::to_string(*((int8_t*) pair.second));
        break;
      case UNSIGNED_BYTE:
        valueStr = std::to_string(*((uint8_t*) pair.second));
        break;
      case SHORT:
        valueStr = std::to_string(*((short*) pair.second));
        break;
      case INT:
        valueStr = std::to_string(*((int*) pair.second));
        break;
      case LONG:
        valueStr = std::to_string(*((long*) pair.second));
        break;
      case UNSIGNED_SHORT:
        valueStr = std::to_string(*((unsigned short*) pair.second));
        break;
      case UNSIGNED_INT:
        valueStr = std::to_string(*((unsigned int*) pair.second));
        break;
      case UNSIGNED_LONG:
        valueStr = std::to_string(*((unsigned long*) pair.second));
        break;
      case FLOAT:
        valueStr = std::to_string(*((float*) pair.second));
        break;
      case DOUBLE:
        valueStr = std::to_string(*((double*) pair.second));
        break;
      case STRING:
        valueStr = *((std::string*) pair.second);
        break;
    }

    message = message.replace(index, index2 - index + 1, valueStr);
    index = message.find("{");
  }

  return true;
}

bool RosGContainerPublisher::publish(std::string const &topic, std::string const &messageName, std::string const &message) {
    std::stringstream output;
    std::string command = "rostopic pub --once " + topic + " " + messageName + " \"" + message + "\"";
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
