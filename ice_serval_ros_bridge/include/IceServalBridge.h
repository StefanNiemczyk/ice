/*
 * ice_serval_bridge.h
 *
 *  Created on: Apr 11, 2016
 *      Author: sni
 */

#ifndef ICE_SERVAL_BRIDGE_H_
#define ICE_SERVAL_BRIDGE_H_

#include <memory>

#include "ice/ontology/OntologyInterface.h"


namespace ice
{

class ice_serval_bridge
{
public:
  ice_serval_bridge();
  virtual ~ice_serval_bridge();

private:
  std::shared_ptr<OntologyInterface> ontologyInterface;
};

} /* namespace ice */

#endif /* ICE_SERVAL_BRIDGE_H_ */
