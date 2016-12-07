/*
 * BaseInformationSender.h
 *
 *  Created on: Jun 17, 2014
 *      Author: sni
 */

#ifndef BASEINFORMATIONSENDER_H_
#define BASEINFORMATIONSENDER_H_

#include <memory>
#include <typeinfo>

#include "ice/information/InformationCollection.h"

namespace ice
{

class BaseInformationSender
{
public:
  BaseInformationSender(std::shared_ptr<InformationCollection> collection)
  {
    this->collection = collection;
  };
  virtual ~BaseInformationSender() {};

  virtual void init() = 0;
  virtual void cleanUp() = 0;

  /*!
   * \brief Returns the type_info of the template type.
   *
   * Returns the type_info of the template type.
   */
  virtual const std::type_info* getTypeInfo() const = 0;

protected:
  std::shared_ptr<InformationCollection> collection;
};

} /* namespace ice */

#endif /* BASEINFORMATIONSENDER_H_ */
