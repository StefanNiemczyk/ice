/*
 * Identity.h
 *
 *  Created on: Apr 12, 2016
 *      Author: sni
 */

#ifndef IDENTITY_H_
#define IDENTITY_H_

#include <map>
#include <string>
#include <memory>

namespace ice
{

enum identity_match {
  FULL_MATCH,
  PARTIAL_MATCH,
  INCLUDED,
  INCLUDING,
  NO_MATCH,
  CONFLICTING
};

struct Id {
    template <typename KeyType, typename ValueType>
    Id(KeyType&& key, ValueType&& value)
            : key{key}, value{value} {}

    std::string key;
    std::string value;
};

class Identity
{
public:
  Identity(const std::initializer_list<Id>& id);
  virtual ~Identity();

  identity_match checkMatching(std::shared_ptr<Identity> &identity);
  void fuse(std::shared_ptr<Identity> &identity);

private:
  std::map<std::string, std::string> ids;
};

} /* namespace ice */

#endif /* IDENTITY_H_ */
