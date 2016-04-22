/*
 * RosGMessagePublisher.h
 *
 *  Created on: Apr 22, 2016
 *      Author: sni
 */

#ifndef ROSGMESSAGEPUBLISHER_H_
#define ROSGMESSAGEPUBLISHER_H_

#include <memory>

#include <ice/representation/GContainer.h>

#include "IceServalBridge.h"

namespace ice
{

class RosGContainerPublisher
{
public:
  RosGContainerPublisher();
  virtual ~RosGContainerPublisher();

  bool publish(std::shared_ptr<RequiredInfo> const &reqInfo, std::shared_ptr<GContainer> constainer);

private:
  bool publish(std::string const &topic, std::string const &messageName, std::string const &message);
};

} /* namespace ice */

#endif /* ROSGMESSAGEPUBLISHER_H_ */
