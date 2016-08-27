/*
 * InformationReceiver.h
 *
 *  Created on: Jun 16, 2014
 *      Author: sni
 */

#ifndef INFORMATIONRECEIVER_H_
#define INFORMATIONRECEIVER_H_

#include <memory>

namespace ice
{

class BaseInformationStream;

class InformationReceiver
{
public:
  InformationReceiver(std::shared_ptr<BaseInformationStream> const &stream);
  virtual ~InformationReceiver();

  /*!
   * \brief Returns the type_info of the template type.
   *
   * Returns the type_info of the template type.
   */
  virtual const std::type_info* getTypeInfo();

private:
  std::shared_ptr<BaseInformationStream>        stream;
};

} /* namespace ice */

#endif /* INFORMATIONRECEIVER_H_ */
