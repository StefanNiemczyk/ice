/*
 * ServalInformationSender.h
 *
 *  Created on: 27.08.2016
 *      Author: sni
 */

#ifndef INCLUDE_SERVALINFORMATIONSENDER_H_
#define INCLUDE_SERVALINFORMATIONSENDER_H_

#include <memory>

#include <ice/communication/InformationSender.h>
#include <ice/representation/GContainer.h>

namespace ice
{
class Entity;
class GContainer;
class ServalCommunication;

class ServalInformationSender : public InformationSender<GContainer>
{
public:
  ServalInformationSender(std::shared_ptr<InformationCollection> collection,
                          std::shared_ptr<ServalCommunication> communication);
  virtual ~ServalInformationSender();

  virtual void init();
  virtual void cleanUp();

  virtual void sendInformationElement(std::vector<std::shared_ptr<Entity>> &sendTo,
                                      std::shared_ptr<InformationElement<GContainer>> informationElement);

private:
  bool                                          isSet;
  unsigned char                                 buffer[1024];
  uint32_t                                      collectionHash;
  std::shared_ptr<ServalCommunication>          communication;
};

} /* namespace ice */

#endif /* INCLUDE_SERVALINFORMATIONSENDER_H_ */
