/*
 * Identity.h
 *
 *  Created on: Apr 12, 2016
 *      Author: sni
 */

#ifndef IDENTITY_H_
#define IDENTITY_H_

#include <chrono>
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
  Identity(const std::initializer_list<Id>& ids);
  virtual ~Identity();

  identity_match checkMatching(std::shared_ptr<Identity> &identity);
  identity_match checkMatching(const std::initializer_list<Id>& ids);
  identity_match checkMatching(std::string &key, std::string &value);
  void fuse(std::shared_ptr<Identity> &identity);
  void fuse(const std::initializer_list<Id>& ids);

  bool isIceIdentity();
  void setIceIdentity(bool value);
  bool isTimeout();

  std::chrono::steady_clock::time_point getActiveTimestamp();
  void setActiveTimestamp(std::chrono::steady_clock::time_point = std::chrono::steady_clock::now());

  void addMetadata(std::string &key, std::string &value);
  bool getMetadata(std::string &key, std::string &outValue);

  void addConnectionQuality(std::string &key, double &value);
  bool getConnectionQuality(std::string &key, double &outValue);

private:
  bool                                                  iceIdentity;
  std::chrono::steady_clock::time_point                 timestamp;
  std::chrono::milliseconds                             timeoutDuration;
  std::map<std::string, std::string>                    ids;
  std::map<std::string, std::string>                    metadata;
  std::map<std::string, double>                         connectionQuality;
};

} /* namespace ice */

#endif /* IDENTITY_H_ */
