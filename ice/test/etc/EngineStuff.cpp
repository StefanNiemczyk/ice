#include <iostream>
#include <memory>
#include <vector>

#include "ice/container/Position.h"
#include "ice/information/BaseInformationStream.h"
#include "ice/information/InformationElement.h"
#include "ice/information/InformationStream.h"
#include "ice/information/StreamFactory.h"
#include "ice/processing/Node.h"
#include "ice/representation/GContainer.h"
#include "ice/representation/Transformation.h"
#include "ice/ICEngine.h"

class TestFactory : public ice::StreamFactory
{
public:
  TestFactory(std::weak_ptr<ice::ICEngine> engine) : ice::StreamFactory(engine) {}

  std::shared_ptr<ice::BaseInformationStream> createStream(const std::string& className,
                                                           std::shared_ptr<ice::CollectionDescription> streamDescription,
                                                           std::shared_ptr<ice::EventHandler> eventHandler,
                                                           int streamSize) const
  {
    if (className == "")
      return nullptr;

    std::shared_ptr<ice::BaseInformationStream> stream;

    if ("Position" == className)
    {
      stream = std::make_shared<ice::InformationStream<ice::Position>>(streamDescription, eventHandler, streamSize);
    }
    else if ("http://vs.uni-kassel.de/IceTest#TestRepresentation1" == className)
    {
      stream = std::make_shared<ice::InformationStream<ice::Position>>(streamDescription, eventHandler, streamSize);
    }
    else if ("http://vs.uni-kassel.de/IceTest#TestRepresentation2" == className)
    {
      stream = std::make_shared<ice::InformationStream<ice::Position>>(streamDescription, eventHandler, streamSize);
    }
    else if ("List[Position]" == className)
    {
      stream = std::make_shared<ice::InformationStream<std::vector<ice::Position>>>(streamDescription, eventHandler, streamSize);
    }

    if (stream == nullptr)
      stream = ice::StreamFactory::createStream(className, streamDescription, eventHandler, streamSize);

    if (stream)
      return stream;

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

    posNew->x = pos->x - 1;
    posNew->y = pos->y - 1;
    posNew->z = pos->z - 1;

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

class TestTransformation : public ice::Transformation
{
public:
  TestTransformation(std::weak_ptr<ice::GContainerFactory> factory) : Transformation(factory, "TestTransformation", "http://vs.uni-kassel.de/Ice#Position")
  {
    auto f = this->factory.lock();
    targetRepresentation = f->getRepresentation("http://vs.uni-kassel.de/IceTest#Pos3D");
    inputs.push_back(f->getRepresentation("http://vs.uni-kassel.de/IceTest#Pos2D"));
  }

  ~TestTransformation() {}

  virtual std::shared_ptr<ice::GContainer> transform(std::shared_ptr<ice::GContainer>* inputs)
  {
    auto output = this->factory.lock()->makeInstance(this->targetRepresentation);

    auto p2dX = this->inputs[0]->accessPath({"http://vs.uni-kassel.de/Ice#XCoordinate"});
    auto p2dY = this->inputs[0]->accessPath({"http://vs.uni-kassel.de/Ice#YCoordinate"});

    auto p3dX = targetRepresentation->accessPath({"http://vs.uni-kassel.de/Ice#XCoordinate"});
    auto p3dY = targetRepresentation->accessPath({"http://vs.uni-kassel.de/Ice#YCoordinate"});
    auto p3dZ = targetRepresentation->accessPath({"http://vs.uni-kassel.de/Ice#ZCoordinate"});

    double defaultZ = 500;

    output->set(p3dX, inputs[0]->get(p2dX));
    output->set(p3dY, inputs[0]->get(p2dY));
    output->set(p3dZ, &defaultZ);

    return output;
  }
};
