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

namespace ice
{

class BaseInformationStream;
class ServalCommunication;

class ServalInformationReceiver : InformationReceiver
{
public:
  ServalInformationReceiver(std::shared_ptr<BaseInformationStream> const &stream,
                            std::shared_ptr<ServalCommunication> const &communication);
  virtual ~ServalInformationReceiver();

  const uint32_t getStreamHash() const;
  std::shared_ptr<BaseInformationStream> getStream() const;

private:
  std::shared_ptr<BaseInformationStream>        stream;
  uint32_t                                      streamHash;
  std::shared_ptr<ServalCommunication>          communication;
};

} /* namespace ice */

#endif /* INCLUDE_SERVALINFORMATIONRECEIVER_H_ */
