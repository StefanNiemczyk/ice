/*
 * ServalInformationReceiver.h
 *
 *  Created on: 27.08.2016
 *      Author: sni
 */

#ifndef INCLUDE_SERVALINFORMATIONRECEIVER_H_
#define INCLUDE_SERVALINFORMATIONRECEIVER_H_

#include <memory>

#include <ice/communication/InformationReceiver.h>
#include <ice/Time.h>
#include <ice/TypeDefs.h>

namespace ice
{
class GContainer;
class InformationCollection;
class ServalCommunication;

class ServalInformationReceiver : public InformationReceiver, public std::enable_shared_from_this<ServalInformationReceiver>
{
public:
  ServalInformationReceiver(std::shared_ptr<InformationCollection> const &collection,
                            std::shared_ptr<TimeFactory> const &timeFactory,
                            std::shared_ptr<ServalCommunication> const &communication);
  virtual ~ServalInformationReceiver();

  virtual void init();
  virtual void cleanUp();

  const uint32_t getHash() const;
  std::shared_ptr<InformationCollection> getCollection() const;
  bool isSet();
  void insertInformation(std::shared_ptr<GContainer> container, ont::entity entity, time timeValidity,
                         time timeObservation, time timeProcessed);

private:
  bool                                          isASet;
  uint32_t                                      collectionHash;
  std::shared_ptr<ServalCommunication>          communication;
};

} /* namespace ice */

#endif /* INCLUDE_SERVALINFORMATIONRECEIVER_H_ */
