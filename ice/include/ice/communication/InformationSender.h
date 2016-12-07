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
    InformationSender(std::shared_ptr<InformationCollection> collection) : BaseInformationSender(collection) {};
    virtual ~InformationSender() {};

    virtual void init() = 0;
    virtual void cleanUp() = 0;

    virtual void sendInformationElement(std::vector<std::shared_ptr<Entity>> &sendTo,
                                        std::shared_ptr<InformationElement<T>> informationElement) = 0;

    /*!
     * \brief Returns the type_info of the template type.
     *
     * Returns the type_info of the template type.
     */
    const std::type_info* getTypeInfo() const
    {
      return &typeid(T);
    }
  };

} /* namespace ice */

#endif /* INFORMATIONSENDER_H_ */
