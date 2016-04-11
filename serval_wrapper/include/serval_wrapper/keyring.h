/*
 * keyring.h
 *
 *  Created on: Apr 8, 2016
 *      Author: sni
 */

#ifndef KEYRING_H_
#define KEYRING_H_

#include <memory>
#include <vector>

namespace ice
{

class serval_interface;
struct serval_identity;

namespace serval_wrapper {

class keyring
{
private:
  const std::string SERVAL_REST_GET_IDENTITIES = "/restful/keyring/identities.json";
  const std::string SERVAL_REST_ADD_IDENTITY = "/restful/keyring/add";
  const std::string SERVAL_REST_GET_IDENTITY_SET = "/restful/keyring/$SID/set";

public:
  keyring(serval_interface* const interface);
  virtual ~keyring();

  std::unique_ptr<std::vector<serval_identity>> getIdentities();
  std::unique_ptr<serval_identity> addIdentity(std::string password = "");

private:
  serval_interface *interface;
};
}

} /* namespace ice */

#endif /* KEYRING_H_ */
