/*
 * InformationStore.h
 *
 *  Created on: May 6, 2016
 *      Author: sni
 */

#ifndef INFORMATIONSTORE_H_
#define INFORMATIONSTORE_H_

#include <memory>
#include <mutex>

#include "easylogging++.h"

namespace ice
{

class OntologyInterface;

class InformationStore
{
public:
  InformationStore(std::shared_ptr<OntologyInterface> ontology);
  virtual ~InformationStore();

  bool init();
  bool cleanUp();

private:
  std::shared_ptr<OntologyInterface> ontology; /**< Interface to access the ontology */
  std::mutex _mtx; /**< Mutex */
  el::Logger* _log; /**< Logger */
};

} /* namespace ice */

#endif /* INFORMATIONSTORE_H_ */
