/*
 * InformationReceiver.h
 *
 *  Created on: Jun 16, 2014
 *      Author: sni
 */

#ifndef INFORMATIONRECEIVER_H_
#define INFORMATIONRECEIVER_H_

namespace ice
{

class InformationReceiver
{
public:
  InformationReceiver() {};
  virtual ~InformationReceiver() {};

  /*!
   * \brief Returns the type_info of the template type.
   *
   * Returns the type_info of the template type.
   */
  virtual const std::type_info* getTypeInfo() const = 0;
};

} /* namespace ice */

#endif /* INFORMATIONRECEIVER_H_ */
