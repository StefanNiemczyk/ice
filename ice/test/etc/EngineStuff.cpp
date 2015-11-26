#include <iostream>
#include <memory>
#include <vector>

#include "ice/container/Position.h"
#include "ice/information/BaseInformationStream.h"
#include "ice/information/InformationElement.h"
#include "ice/information/InformationStream.h"
#include "ice/information/StreamFactory.h"
#include "ice/processing/Node.h"

class TestFactory : public ice::StreamFactory
{
  std::shared_ptr<ice::BaseInformationStream> createStream(const std::string& className,
                                                           std::shared_ptr<ice::StreamDescription> streamDescription,
                                                           std::shared_ptr<ice::EventHandler> eventHandler,
                                                           int streamSize,
                                                           int sharingMaxCount) const
  {
    auto stream = ice::StreamFactory::createStream(className, streamDescription, eventHandler, streamSize,
                                                   sharingMaxCount);
    if (stream)
      return stream;

    if (className == "")
      return stream;

    if ("Position" == className)
    {
      stream = std::make_shared<ice::InformationStream<ice::Position>>(streamDescription, eventHandler, streamSize,
          sharingMaxCount);
    }
    else if ("http://vs.uni-kassel.de/IceTest#TestRepresentation1" == className)
    {
      stream = std::make_shared<ice::InformationStream<ice::Position>>(streamDescription, eventHandler, streamSize,
          sharingMaxCount);
    }
    else if ("http://vs.uni-kassel.de/IceTest#TestRepresentation2" == className)
    {
      stream = std::make_shared<ice::InformationStream<ice::Position>>(streamDescription, eventHandler, streamSize,
          sharingMaxCount);
    }
    else if ("List[Position]" == className)
    {
      stream = std::make_shared<ice::InformationStream<std::vector<ice::Position>>>(streamDescription, eventHandler, streamSize,
          sharingMaxCount);
    }

    return stream;
  }
};

class ObstacleFusion : public ice::Node
{
public:
  static std::shared_ptr<ice::Node> createNode()
  {
    return std::make_shared<ObstacleFusion>();
  }

  ObstacleFusion() :
      Node()
  {

  }

  virtual std::string getClassName()
  {
    return "ObstacleFusing";
  }

  virtual int performNode()
  {
    return 0;
  }
};

class SmothingNode : public ice::Node
{
public:

  std::shared_ptr<ice::InformationStream<ice::Position>> inputStream;
  std::shared_ptr<ice::InformationStream<ice::Position>> outputStream;

  static std::shared_ptr<ice::Node> createNode()
  {
    return std::make_shared<SmothingNode>();
  }

  SmothingNode() :
      Node()
  {

  }

  virtual std::string getClassName()
  {
    return "smothing";
  }

  int init()
  {
    if (this->inputs.empty() || this->outputs.empty()){
      return 1;
    }

    this->inputStream = std::static_pointer_cast<ice::InformationStream<ice::Position>>(this->inputs[0]);
    this->outputStream = std::static_pointer_cast<ice::InformationStream<ice::Position>>(this->outputs[0]);

    return 0;
  }

  virtual int performNode()
  {
    auto infoEle = this->inputStream->getLast();
    std::unique_ptr<ice::Position> posNew(new ice::Position());
    auto pos = infoEle->getInformation();

    posNew->x = pos.x - 1;
    posNew->y = pos.y - 1;
    posNew->z = pos.z - 1;

    this->outputStream->add(std::move(posNew));

    return 0;
  }
};

class SimpleSourceNode : public ice::Node
{
public:
  static std::shared_ptr<ice::Node> createNode()
  {
    return std::make_shared<SimpleSourceNode>();
  }

  SimpleSourceNode() :
      Node()
  {

  }

  virtual std::string getClassName()
  {
    return "SimpleSourceNode";
  }

  virtual int performNode()
  {
    return 0;
  }
};
