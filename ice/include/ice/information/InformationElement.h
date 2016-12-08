/*
 * InformationElement.h
 *
 *  Created on: May 13, 2014
 *      Author: sni
 */

#ifndef INFORMATIONELEMENT_H_
#define INFORMATIONELEMENT_H_

#include <memory>

#include "ice/Time.h"
#include "ice/information/InformationSpecification.h"

namespace ice
{

//* InformationElement
/**
 * Information element stores one information and provides required meta
 * data. The information can not be changed.
 *
 */
template<typename T>
  class InformationElement
  {
  public:
    /*!
     * \brief Default constructor
     *
     * Default constructor
     *
     * \param specification The specification of the stored information.
     * \param information The information to store.
     * \param timeValidity validity time of the information.
     * \param timeObservation observation time of the information.
     * \param timeProcessed time of the processing of the information.
     */
    InformationElement(std::shared_ptr<InformationSpecification> specification, std::shared_ptr<T> information,
                       time timeValidity = NO_TIME, time timeObservation = NO_TIME, time timeProcessed = NO_TIME) :
        specification(specification), information(std::move(information))
    {
      this->timeValidity = timeValidity;
      this->timeObservation = timeObservation;
      this->timeProcessed = timeProcessed;
    }

    /*!
     * \brief Default destructor
     *
     * Default destructor
     */
    virtual ~InformationElement()
    {
      //
    }

    /*!
     * \brief Returns the information stored in this container.
     *
     * Returns the information stored in this container.
     */
    const std::shared_ptr<T> getInformation() const
    {
      return this->information;
    }

//    const std::shared_ptr<T> getSharedInformation() const
//    {
//      return this->information;
//    }

    /*!
     * \brief Returns true if the information is still valid, else false.
     *
     * Checks the timeValidity time of the current information. Returns true if the
     * timeValidity is before current time, else false.
     */
    bool isValid() const
    {
      if (false == this->timeValidity)
        return true;

      return (this->timeValidity->isBeforeCurrent());
    }

    /*!
     * \brief Returns the specification of the information stored in this container.
     *
     * Returns the specification of the information stored in this container.
     */
    std::shared_ptr<InformationSpecification> getSpecification()
    {
      return this->specification;
    }

    time getTimeValidity()
    {
      return timeValidity;
    }

    time getTimeObservation()
    {
      return timeObservation;
    }

    time getTimeProcessed()
    {
      return timeProcessed;
    }

  private:
    time                                        timeValidity;           /**< validity time of the information */
    time                                        timeObservation;        /**< observation time of the information */
    time                                        timeProcessed;          /**< time of the processing of the information */
    std::shared_ptr<InformationSpecification>   specification;          /**< Specification of the information stored in this container */
    const std::shared_ptr<T>                    information;            /**< the stored information */
  };

} /* namespace ice */

#endif /* INFORMATIONELEMENT_H_ */
