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

std::shared_ptr<BaseInformationStream> StreamFactory::createStream(const std::string& className,
                                                                   std::shared_ptr<StreamDescription> streamDescription,
                                                                   std::shared_ptr<EventHandler> eventHandler,
                                                                   int streamSize, int sharingMaxCount) const
{
  std::shared_ptr<BaseInformationStream> stream;

  if (className == "")
    return stream;

//  std::shared_ptr<StreamDescription> streamDescription,
//  std::shared_ptr<EventHandler> eventHandler,
//  int streamSize,
//  int sharingMaxCount = 0

  if ("int" == className)
  {
    stream = std::make_shared<InformationStream<int>>(streamDescription, eventHandler, streamSize, sharingMaxCount);
  }
  else if ("long" == className)
  {
    stream = std::make_shared<InformationStream<long>>(streamDescription, eventHandler, streamSize, sharingMaxCount);
  }
  else if ("double" == className)
  {
    stream = std::make_shared<InformationStream<double>>(streamDescription, eventHandler, streamSize, sharingMaxCount);
  }
  else if ("float" == className)
  {
    stream = std::make_shared<InformationStream<float>>(streamDescription, eventHandler, streamSize, sharingMaxCount);
  }
  else if ("bool" == className)
  {
    stream = std::make_shared<InformationStream<float>>(streamDescription, eventHandler, streamSize, sharingMaxCount);
  }
  else if ("List[int]" == className)
  {
    stream = std::make_shared<InformationStream<std::vector<int>>>(streamDescription, eventHandler, streamSize,
    sharingMaxCount);
  }
  else if ("List[long]" == className)
  {
    stream = std::make_shared<InformationStream<std::vector<long>>>(streamDescription, eventHandler, streamSize,
    sharingMaxCount);
  }
  else if ("List[double]" == className)
  {
    stream = std::make_shared<InformationStream<std::vector<double>>>(streamDescription, eventHandler, streamSize,
    sharingMaxCount);
  }
  else if ("List[float]" == className)
  {
    stream = std::make_shared<InformationStream<std::vector<float>>>(streamDescription, eventHandler, streamSize,
    sharingMaxCount);
  }
  else if ("List[bool]" == className)
  {
    stream = std::make_shared<InformationStream<std::vector<bool>>>(streamDescription, eventHandler, streamSize,
    sharingMaxCount);
  }

  return stream;
}

} /* namespace ice */
