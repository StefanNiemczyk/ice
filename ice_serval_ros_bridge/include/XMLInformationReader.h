/*
 * XMLInformationReader.h
 *
 *  Created on: Apr 22, 2016
 *      Author: sni
 */

#ifndef XMLINFORMATIONREADER_H_
#define XMLINFORMATIONREADER_H_

#include <memory>
#include <vector>

#include <easylogging++.h>
#include <tinyxml.h>
#include <ice/information/InformationSpecification.h>

#include "IceServalBridge.h"

namespace ice
{

class XMLInformationReader
{
public:
  XMLInformationReader();
  virtual ~XMLInformationReader();

  bool readFile(const std::string& fileName);
  std::vector<std::shared_ptr<OfferedInfo>> getOffered();
  std::vector<std::shared_ptr<RequiredInfo>> getRequired();

private:
  bool readOffered(TiXmlElement* element);
  bool readRequired(TiXmlElement* element);

private:
  std::vector<std::shared_ptr<OfferedInfo>> offered;
  std::vector<std::shared_ptr<RequiredInfo>> required;
  el::Logger* _log; /**< Logger */
};

} /* namespace ice */

#endif /* XMLINFORMATIONREADER_H_ */
