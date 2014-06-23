/*
 * BaseInformationSender.h
 *
 *  Created on: Jun 17, 2014
 *      Author: sni
 */

#ifndef BASEINFORMATIONSENDER_H_
#define BASEINFORMATIONSENDER_H_

#include <typeinfo>

namespace ice
{

class BaseInformationSender
{
public:
  BaseInformationSender() {};
  virtual ~BaseInformationSender() {};

  /*!
   * \brief Returns the type_info of the template type.
   *
   * Returns the type_info of the template type.
   */
  virtual const std::type_info* getTypeInfo() const = 0;
};

} /* namespace ice */

#endif /* BASEINFORMATIONSENDER_H_ */
