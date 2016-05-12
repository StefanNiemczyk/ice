/*
 * InformationStore.h
 *
 *  Created on: May 6, 2016
 *      Author: sni
 */

#ifndef INFORMATIONSTORE_H_
#define INFORMATIONSTORE_H_

#include <map>
#include <memory>
#include <mutex>
#include <vector>

#include "easylogging++.h"

namespace ice
{


class GContainer;
template<typename T>
class InformationElement;
class InformationSpecification;
class OntologyInterface;

typedef InformationElement<GContainer> GElement;

struct Information
{
  std::shared_ptr<InformationSpecification> infoSpec;
  std::shared_ptr<InformationElement<GContainer>> info;
};

class InformationStore
{
public:
  InformationStore(std::shared_ptr<OntologyInterface> ontology);
  virtual ~InformationStore();

  bool init();
  bool cleanUp();

  void addInformation(std::shared_ptr<InformationSpecification> infoSpec,
                      std::shared_ptr<GElement> info);
  int getInformation(std::shared_ptr<InformationSpecification> request,
                     std::vector<std::shared_ptr<GElement>> &outInfo);

private:
  std::shared_ptr<OntologyInterface> ontology; /**< Interface to access the ontology */
  std::map<std::shared_ptr<InformationSpecification>,std::shared_ptr<GElement>> information;
  std::mutex _mtx; /**< Mutex */
  el::Logger* _log; /**< Logger */
};

} /* namespace ice */

#endif /* INFORMATIONSTORE_H_ */
