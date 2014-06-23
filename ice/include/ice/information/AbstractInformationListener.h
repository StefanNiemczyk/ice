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
#include "ice/information/InformationStream.h"

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
    virtual ~AbstractInformationListener();

    /*!
     * \brief This method will be triggered if a new element is added to the information stream.
     *
     * This method will be triggered if a new element is added to the information stream.
     *
     * \param element The new information element.
     * \param stream The stream which received the new information element.
     */
    virtual const int newEvent(std::shared_ptr<InformationElement<T>> element,
                               std::shared_ptr<InformationStream<T>> stream);
  };

} /* namespace ice */

//Include after forward declaration

//Implementing methods here

template<typename T>
  inline ice::AbstractInformationListener<T>::~AbstractInformationListener()
  {
  }

template<typename T>
  inline const int ice::AbstractInformationListener<T>::newEvent(std::shared_ptr<InformationElement<T>> element,
                                                            std::shared_ptr<InformationStream<T>> stream)
  {
  }

#endif /* ABSTRACTINFORMATIONLISTENER_H_ */
