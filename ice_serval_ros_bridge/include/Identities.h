/*
 * identities.h
 *
 *  Created on: Apr 11, 2016
 *      Author: sni
 */

#ifndef IDENTITIES_H_
#define IDENTITIES_H_

#include <string>
#include <Identity.h>

namespace ice
{

typedef std::string serval_id;
typedef std::string ontology_iri;

class Identities
{
public:
  Identities();
  virtual ~Identities();

  Identity registerIdentity(std::string key, std::string value);
};

} /* namespace ice */

#endif /* IDENTITIES_H_ */
