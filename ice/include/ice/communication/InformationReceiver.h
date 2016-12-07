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

class InformationCollection;

class InformationReceiver
{
public:
  InformationReceiver(std::shared_ptr<InformationCollection> const &collection);
  virtual ~InformationReceiver();

  virtual void init() = 0;
  virtual void cleanUp() = 0;

  /*!
   * \brief Returns the type_info of the template type.
   *
   * Returns the type_info of the template type.
   */
  virtual const std::type_info* getTypeInfo();

protected:
  std::shared_ptr<InformationCollection>        collection;
};

} /* namespace ice */

#endif /* INFORMATIONRECEIVER_H_ */
