/*
 * InformationSender.h
 *
 *  Created on: Jun 16, 2014
 *      Author: sni
 */

#ifndef INFORMATIONSENDER_H_
#define INFORMATIONSENDER_H_

#include <ice/communication/BaseInformationSender.h>
#include <ice/Entity.h>
#include <memory>
#include <vector>

namespace ice
{
template<typename T>
  class InformationElement;
} /* namespace ice */

namespace ice
{

template<typename T>
  class InformationSender : public BaseInformationSender
  {
  public:
    InformationSender() {};
    virtual ~InformationSender() {};

    virtual void sendInformationElement(std::vector<std::shared_ptr<Entity>> &sendTo,
                                        std::shared_ptr<InformationElement<T>> informationElement) = 0;

    /*!
     * \brief Returns the type_info of the template type.
     *
     * Returns the type_info of the template type.
     */
    virtual const std::type_info* getTypeInfo() const = 0;
  };

} /* namespace ice */

#endif /* INFORMATIONSENDER_H_ */
