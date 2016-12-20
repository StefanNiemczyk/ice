#include <ice/information/CollectionFactory.h>
#include <iostream>
#include <memory>
#include <vector>

#include "ice/container/Position.h"
#include "ice/information/BaseInformationStream.h"
#include "ice/information/InformationElement.h"
#include "ice/information/InformationSet.h"
#include "ice/information/InformationStream.h"
#include "ice/processing/Node.h"
#include "ice/representation/GContainer.h"
#include "ice/representation/Transformation.h"
#include "ice/ICEngine.h"

#include "Pos3D.h"

class SimpleTask : public ice::AsynchronousTask
{
public:
  SimpleTask()
  {
    value = false;
  }

  int performTask()
  {
    value = true;

    return 0;
  }

  bool value;
};

class SimpleListener : public ice::AbstractInformationListener<int>
{
public:
  SimpleListener()
  {
    value = 0;
  }

  ~SimpleListener()
  {

  }

  int value;

  const int newEvent(std::shared_ptr<ice::InformationElement<int>> element,
                     std::shared_ptr<ice::InformationCollection> stream)
  {
    value = *element->getInformation();

    return 0;
  }
};

class SimpleListener2 : public ice::AbstractInformationListener<int>
{
public:
  SimpleListener2()
  {
    value = 0;
  }

  ~SimpleListener2()
  {

  }

  int value;

  const int newEvent(std::shared_ptr<ice::InformationElement<int> > element,
                     std::shared_ptr<ice::InformationCollection> stream)
  {
    value = *element->getInformation() + 3;

    return 0;
  }
};

class TestFactory : public ice::CollectionFactory
{
public:
  TestFactory(std::weak_ptr<ice::ICEngine> engine) : ice::CollectionFactory(engine) {}

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
      stream = std::make_shared<ice::InformationStream<ice::Pos3D>>(streamDescription, eventHandler, streamSize);
      stream->setGContainer(true);
    }
    else if ("http://vs.uni-kassel.de/IceTest#TestRepresentation1" == className)
    {
      stream = std::make_shared<ice::InformationStream<ice::Pos3D>>(streamDescription, eventHandler, streamSize);
      stream->setGContainer(true);
    }
    else if ("http://vs.uni-kassel.de/IceTest#TestRepresentation2" == className)
    {
      stream = std::make_shared<ice::InformationStream<ice::Pos3D>>(streamDescription, eventHandler, streamSize);
      stream->setGContainer(true);
    }

    if (stream == nullptr)
      stream = ice::CollectionFactory::createStream(className, streamDescription, eventHandler, streamSize);

    if (stream)
      return stream;

    return stream;
  }

  std::shared_ptr<ice::BaseInformationSet> createSet(const std::string& className,
                                                             std::shared_ptr<ice::CollectionDescription> streamDescription,
                                                             std::shared_ptr<ice::EventHandler> eventHandler) const
    {
      if (className == "")
        return nullptr;

      std::shared_ptr<ice::BaseInformationSet> set;

      if ("Position" == className)
      {
        set = std::make_shared<ice::InformationSet<ice::Pos3D>>(streamDescription, eventHandler);
        set->setGContainer(true);
      }
      else if ("http://vs.uni-kassel.de/IceTest#TestRepresentation1" == className)
      {
        set = std::make_shared<ice::InformationSet<ice::Pos3D>>(streamDescription, eventHandler);
        set->setGContainer(true);
      }
      else if ("http://vs.uni-kassel.de/IceTest#TestRepresentation2" == className)
      {
        set = std::make_shared<ice::InformationSet<ice::Pos3D>>(streamDescription, eventHandler);
        set->setGContainer(true);
      }

      if (set == nullptr)
        set = ice::CollectionFactory::createSet(className, streamDescription, eventHandler);

      if (set)
        return set;

      return set;
    }
};

class GContainerTestFactory : public ice::CollectionFactory
{
public:
  GContainerTestFactory(std::weak_ptr<ice::ICEngine> engine) : ice::CollectionFactory(engine) {}

  std::shared_ptr<ice::BaseInformationStream> createStream(const std::string& className,
                                                           std::shared_ptr<ice::CollectionDescription> streamDescription,
                                                           std::shared_ptr<ice::EventHandler> eventHandler,
                                                           int streamSize) const
  {
    if (className == "")
      return nullptr;

    std::shared_ptr<ice::BaseInformationStream> stream;

    if ("http://vs.uni-kassel.de/IceTest#Pos3D" == className)
    {
      stream = std::make_shared<ice::InformationStream<ice::Pos3D>>(streamDescription, eventHandler, streamSize);
      stream->setGContainer(true);
    }

    if (stream == nullptr)
      stream = ice::CollectionFactory::createStream(className, streamDescription, eventHandler, streamSize);

    if (stream)
      return stream;

    return stream;
  }

  std::shared_ptr<ice::BaseInformationSet> createSet(const std::string& className,
                                                             std::shared_ptr<ice::CollectionDescription> streamDescription,
                                                             std::shared_ptr<ice::EventHandler> eventHandler) const
    {
      if (className == "")
        return nullptr;

      std::shared_ptr<ice::BaseInformationSet> set;

      if ("http://vs.uni-kassel.de/IceTest#Pos3D" == className)
      {
        set = std::make_shared<ice::InformationSet<ice::Pos3D>>(streamDescription, eventHandler);
        set->setGContainer(true);
      }

      if (set == nullptr)
        set = ice::CollectionFactory::createSet(className, streamDescription, eventHandler);

      if (set)
        return set;

      return set;
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

  virtual const int newEvent(std::shared_ptr<ice::InformationElement<ice::GContainer>> element,
                             std::shared_ptr<ice::InformationCollection> collection)
  {
    this->performNode();
    return 0;
  }

  virtual int performNode()
  {
    return 0;
  }
};

class SmothingNode : public ice::Node
{
public:

  std::shared_ptr<ice::InformationStream<ice::Pos3D>> inputStream;
  std::shared_ptr<ice::InformationStream<ice::Pos3D>> outputStream;

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

    this->inputStream = std::static_pointer_cast<ice::InformationStream<ice::Pos3D>>(this->inputs[0]);
    this->outputStream = std::static_pointer_cast<ice::InformationStream<ice::Pos3D>>(this->outputs[0]);

    return 0;
  }

  virtual const int newEvent(std::shared_ptr<ice::InformationElement<ice::GContainer>> element,
                             std::shared_ptr<ice::InformationCollection> collection)
  {
    this->performNode();
    return 0;
  }

  virtual int performNode()
  {
    auto infoEle = this->inputStream->getLast();
    auto rep = this->gcontainerFactory->getRepresentation("http://vs.uni-kassel.de/Ice#CoordinatePositionRep");
    std::unique_ptr<ice::Pos3D> posNew(new ice::Pos3D(rep));
    auto pos = infoEle->getInformation();

    posNew->x = pos->x - 1;
    posNew->y = pos->y - 1;
    posNew->z = pos->z - 1;

    this->outputStream->add(std::move(posNew));

    return 0;
  }
};

class SetTestNode : public ice::Node
{
public:

  std::shared_ptr<ice::InformationSet<ice::Pos3D>> inputSet;
  std::shared_ptr<ice::InformationSet<ice::Pos3D>> outputSet;

  static std::shared_ptr<ice::Node> createNode()
  {
    auto node = std::make_shared<SetTestNode>();
    auto desc = std::make_shared<ice::NodeDescription>(ice::NodeType::SET, "SetTestNode", "SetTestNode", "", "");
    node->setNodeDescription(desc);

    return node;
  }

  SetTestNode() : Node()
  {

  }

  virtual std::string getClassName()
  {
    return "setTest";
  }

  int init()
  {
    if (this->inputSets.empty() || this->outputSets.empty()){
      return 1;
    }

    this->inputSet = std::static_pointer_cast<ice::InformationSet<ice::Pos3D>>(this->inputSets[0]);
    this->outputSet = std::static_pointer_cast<ice::InformationSet<ice::Pos3D>>(this->outputSets[0]);

    return 0;
  }

  virtual const int newEvent(std::shared_ptr<ice::InformationElement<ice::GContainer>> element,
                             std::shared_ptr<ice::InformationCollection> collection)
  {
    this->performNode();
    return 0;
  }

  virtual int performNode()
  {
    auto infoEle = this->inputSet->get("muh");
    std::unique_ptr<ice::Pos3D> posNew(new ice::Pos3D(nullptr));
    auto pos = infoEle->getInformation();

    posNew->x = pos->x - 1;
    posNew->y = pos->y - 1;
    posNew->z = pos->z - 1;

    this->outputSet->add(infoEle->getSpecification()->getEntity(), std::move(posNew));

    return 0;
  }
};

class SetSourceNode : public ice::Node
{
public:
  std::shared_ptr<ice::InformationSet<ice::Pos3D>> outputSet;

  static std::shared_ptr<ice::Node> createNode()
  {
    auto node = std::make_shared<SetSourceNode>();

    return node;
  }

  SetSourceNode() : Node()
  {

  }

  virtual std::string getClassName()
  {
    return "setTest";
  }

  int init()
  {
    if (this->outputSets.empty()){
      return 1;
    }

    this->outputSet = std::static_pointer_cast<ice::InformationSet<ice::Pos3D>>(this->outputSets[0]);

    return 0;
  }

  virtual const int newEvent(std::shared_ptr<ice::InformationElement<ice::GContainer>> element,
                             std::shared_ptr<ice::InformationCollection> collection)
  {
    this->performNode();
    return 0;
  }

  virtual int performNode()
  {
    auto rep = this->gcontainerFactory->getRepresentation("http://vs.uni-kassel.de/Ice#CoordinatePositionRep");
    std::unique_ptr<ice::Pos3D> posNew(new ice::Pos3D(rep));

    posNew->x = 1;
    posNew->y = 21;
    posNew->z = 31;

    this->outputSet->add("muh", std::move(posNew));

    return 0;
  }
};

class TestSetNode : public ice::Node
{
public:
  std::shared_ptr<ice::InformationSet<ice::Pos3D>> outputSet;

  static std::shared_ptr<ice::Node> createNode()
  {
    auto node = std::make_shared<TestSetNode>();

    return node;
  }

  TestSetNode() : Node()
  {

  }

  virtual std::string getClassName()
  {
    return "testSetNode";
  }

  int init()
  {
    if (this->outputSets.empty() || this->inputs.empty()){
      return 1;
    }

    this->outputSet = std::static_pointer_cast<ice::InformationSet<ice::Pos3D>>(this->outputSets[0]);

    return 0;
  }

  virtual const int newEvent(std::shared_ptr<ice::InformationElement<ice::GContainer>> element,
                             std::shared_ptr<ice::InformationCollection> collection)
  {
    this->performNode();
    return 0;
  }

  virtual int performNode()
  {
    for (auto &in : this->inputs)
    {
      auto stream = std::static_pointer_cast<ice::InformationStream<ice::Pos3D>>(in);
      auto info = stream->getLast();
      if (info != nullptr)
      {
        this->outputSet->add(info->getSpecification()->getEntity(), info->getInformation());
      }
    }

    return 0;
  }
};

class TestSetTransformNode : public ice::Node
{
public:
  std::shared_ptr<ice::InformationSet<ice::Pos3D>> inputSet;
  std::shared_ptr<ice::InformationSet<ice::Pos3D>> outputSet;

  static std::shared_ptr<ice::Node> createNode()
  {
    auto node = std::make_shared<TestSetTransformNode>();

    return node;
  }

  TestSetTransformNode() : Node()
  {

  }

  virtual std::string getClassName()
  {
    return "testSetNode";
  }

  int init()
  {
    if (this->outputSets.empty() || this->inputSets.empty()){
      std::cout << "ERROR ################################################" << std::endl;
      std::cout << this->inputSets.empty() << std::endl;
      std::cout << this->outputSets.empty() << std::endl;
      return 1;
    }

    this->inputSet = std::static_pointer_cast<ice::InformationSet<ice::Pos3D>>(this->inputSets[0]);
    this->outputSet = std::static_pointer_cast<ice::InformationSet<ice::Pos3D>>(this->outputSets[0]);

    return 0;
  }

  virtual const int newEvent(std::shared_ptr<ice::InformationElement<ice::GContainer>> element,
                             std::shared_ptr<ice::InformationCollection> collection)
  {
    this->performNode();
    return 0;
  }

  virtual int performNode()
  {
    auto info = this->inputSet->get("muh");
    auto pos = info->getInformation();
    auto rep = this->gcontainerFactory->getRepresentation("http://vs.uni-kassel.de/Ice#CoordinatePositionRep");
    auto posNew = std::make_shared<ice::Pos3D>(rep);
    posNew->x = 100 + pos->x;
    posNew->y = 100 + pos->y;
    posNew->z = 100 + pos->z;
    this->outputSet->add(info->getSpecification()->getEntity(), posNew);

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

  virtual const int newEvent(std::shared_ptr<ice::InformationElement<ice::GContainer>> element,
                             std::shared_ptr<ice::InformationCollection> collection)
  {
    return 0;
  }

  virtual int performNode()
  {
    return 0;
  }
};

class SimpleSetSourceNode : public ice::Node
{
public:
  static std::shared_ptr<ice::Node> createNode()
  {
    return std::make_shared<SimpleSetSourceNode>();
  }

  SimpleSetSourceNode() :
      Node()
  {

  }

  virtual std::string getClassName()
  {
    return "SimpleSetSourceNode";
  }

  virtual const int newEvent(std::shared_ptr<ice::InformationElement<ice::GContainer>> element,
                             std::shared_ptr<ice::InformationCollection> collection)
  {
    return 0;
  }

  virtual int performNode()
  {
    return 0;
  }
};

class SimpleSourceNodeAlternative : public ice::Node
{
public:
  static std::shared_ptr<ice::Node> createNode()
  {
    return std::make_shared<SimpleSourceNodeAlternative>();
  }

  SimpleSourceNodeAlternative() : Node()
  {

  }

  virtual std::string getClassName()
  {
    return "SimpleSourceNodeAlternative";
  }

  virtual const int newEvent(std::shared_ptr<ice::InformationElement<ice::GContainer>> element,
                             std::shared_ptr<ice::InformationCollection> collection)
  {
    this->performNode();
    return 0;
  }

  virtual int performNode()
  {
    this->nodeStore->handleNodeFailure(this->getCreatorName());
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
