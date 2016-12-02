/*
 * AbstractInformationListener.h
 *
 *  Created on: May 23, 2014
 *      Author: sni
 */

#ifndef ABSTRACTINFORMATIONLISTENER_H_
#define ABSTRACTINFORMATIONLISTENER_H_

#include <memory>

#include "ice/information/InformationElement.h"
#include "ice/information/InformationCollection.h"

namespace ice
{

//Forward declaration

//* InformationListener
/**
 * Abstract listener which can be registered at a information stream and is triggered if
 * a new information element is added to the stream.
 *
 */
template<typename T>
  class AbstractInformationListener
  {
  public:
    /*!
     * \brief Default destructor
     *
     * Default destructor
     */
    virtual ~AbstractInformationListener() {};

    /*!
     * \brief This method will be triggered if a new element is added to the information stream.
     *
     * This method will be triggered if a new element is added to the information stream.
     *
     * \param element The new information element.
     * \param stream The stream which received the new information element.
     */
    virtual const int newEvent(std::shared_ptr<InformationElement<T>> element,
                               std::shared_ptr<InformationCollection> stream) = 0;
  };

} /* namespace ice */


#endif /* ABSTRACTINFORMATIONLISTENER_H_ */
