/*
 * CooperationRequest.h
 *
 *  Created on: Jun 12, 2014
 *      Author: sni
 */

#ifndef COOPERATIONREQUEST_H_
#define COOPERATIONREQUEST_H_

#include <memory>
#include <vector>

#include "ice/Identifier.h"

namespace ice
{
class StreamDescription;
class StreamTemplateDescription;
} /* namespace ice */

namespace ice
{

//* CooperationRequest
/**
 * This class describes the cooperation request between two engines and contains:
 * - streams requested from an other engine or offered to this engine
 * - similar elements within the information processing are contained as well.
 *
 */
class CooperationRequest
{
public:
  /*!
   * \brief The constructor sets the identifier of the receiving engine.
   *
   * The constructor sets the identifier of the receiving engine.
   *
   * \param receivingEngine The identifier of the receiving engine.
   */
  CooperationRequest(identifier receivingEngine);

  /*!
   * \brief Default destructor
   *
   * Default destructor
   */
  virtual ~CooperationRequest();

  /*!
   * \brief Returns true if the request is empty, else false.
   *
   * Returns true if the request is empty, else false.
   */
  bool isEmpty() const;

  /*!
   * \brief Returns the streams offered to the receiving engine.
   *
   * Returns the streams offered to the receiving engine.
   */
  const std::shared_ptr<std::vector<std::shared_ptr<StreamDescription>> > getOffers() const;

  /*!
   * \brief Returns the streams requested from the receiving engine.
   *
   * Returns the streams requested from the receiving engine.
   */
  const std::shared_ptr<std::vector<std::shared_ptr<StreamTemplateDescription>>>getRequests() const;

private:
  identifier receivingEngine; /**< Identifier of the receiving engine of this request */
  std::shared_ptr<std::vector<std::shared_ptr<StreamDescription>> > offers; /**< List of offers */
  std::shared_ptr<std::vector<std::shared_ptr<StreamTemplateDescription>>> requests; /**< List of requests */
};

}
/* namespace ice */

#endif /* COOPERATIONREQUEST_H_ */
