/*
 * StreamFactory.cpp
 *
 *  Created on: May 30, 2014
 *      Author: sni
 */

#include "ice/information/StreamFactory.h"

namespace ice
{

StreamFactory::StreamFactory()
{
  //
}

StreamFactory::~StreamFactory()
{
  //
}

std::shared_ptr<BaseInformationStream> StreamFactory::createStream(
    const std::string& className, const std::string name, std::weak_ptr<InformationType> informationType,
    std::shared_ptr<EventHandler> eventHandler, std::shared_ptr<InformationSpecification> specification, int streamSize,
    std::string provider, std::string description, bool shared) const
{
  std::shared_ptr<BaseInformationStream> stream;

  if (className == "")
    return stream;

  if ("int" == className)
  {
    stream = std::make_shared<InformationStream<int>>(name, informationType, eventHandler, specification, streamSize, provider,
                                        description, shared);
  }
  else if ("long" == className)
  {
    stream = std::make_shared<InformationStream<long>>(name, informationType, eventHandler, specification, streamSize, provider,
                                         description, shared);
  }
  else if ("double" == className)
  {
    stream = std::make_shared<InformationStream<double>>(name, informationType, eventHandler, specification, streamSize, provider,
                                           description, shared);
  }
  else if ("float" == className)
  {
    stream = std::make_shared<InformationStream<float>>(name, informationType, eventHandler, specification, streamSize, provider,
                                          description, shared);
  }
  else if ("bool" == className)
  {
    stream = std::make_shared<InformationStream<float>>(name, informationType, eventHandler, specification, streamSize, provider,
                                          description, shared);
  }
  else if ("List[int]" == className)
  {
    stream = std::make_shared<InformationStream<std::vector<int>>>(name, informationType, eventHandler, specification, streamSize, provider,
                                          description, shared);
  }
  else if ("List[long]" == className)
  {
    stream = std::make_shared<InformationStream<std::vector<long>>>(name, informationType, eventHandler, specification, streamSize, provider,
                                          description, shared);
  }
  else if ("List[double]" == className)
  {
    stream = std::make_shared<InformationStream<std::vector<double>>>(name, informationType, eventHandler, specification, streamSize, provider,
                                          description, shared);
  }
  else if ("List[float]" == className)
  {
    stream = std::make_shared<InformationStream<std::vector<float>>>(name, informationType, eventHandler, specification, streamSize, provider,
                                          description, shared);
  }
  else if ("List[bool]" == className)
  {
    stream = std::make_shared<InformationStream<std::vector<bool>>>(name, informationType, eventHandler, specification, streamSize, provider,
                                          description, shared);
  }

  return stream;
}

} /* namespace ice */
