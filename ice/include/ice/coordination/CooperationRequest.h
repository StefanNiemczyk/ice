/*
 * CooperationRequest.h
 *
 *  Created on: Jun 12, 2014
 *      Author: sni
 */

#ifndef COOPERATIONREQUEST_H_
#define COOPERATIONREQUEST_H_

#include <ice/Identifier.h>
#include <memory>
#include <vector>

namespace ice
{
class StreamDescription;
class StreamTemplateDescription;
} /* namespace ice */

namespace ice
{

class CooperationRequest
{
public:
  CooperationRequest(identifier requestingEngine);
  virtual ~CooperationRequest();

  const std::shared_ptr<std::vector<std::shared_ptr<StreamDescription>> > getOffers() const;

  void setOffers(const std::shared_ptr<std::vector<std::shared_ptr<StreamDescription>> > offers);

  const std::shared_ptr<std::vector<std::shared_ptr<StreamTemplateDescription>>>getRequests() const;

  void setRequests(const std::shared_ptr<std::vector<std::shared_ptr<StreamTemplateDescription>>> requests);

private:
  identifier requestingEngine;
  std::shared_ptr<std::vector<std::shared_ptr<StreamDescription>> > offers; /**< List of offers */
  std::shared_ptr<std::vector<std::shared_ptr<StreamTemplateDescription>>> requests; /**< List of requests */
};

}
/* namespace ice */

#endif /* COOPERATIONREQUEST_H_ */
