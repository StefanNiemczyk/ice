/*
 * XMLInformationReader.h
 *
 *  Created on: Apr 22, 2016
 *      Author: sni
 */

#ifndef XMLINFORMATIONREADER_H_
#define XMLINFORMATIONREADER_H_

#include <easylogging++.h>
#include <memory>
#include <tinyxml.h>
#include <vector>

#include <ice/information/InformationSpecification.h>

namespace ice
{

class XMLInformationReader
{
public:
  XMLInformationReader();
  virtual ~XMLInformationReader();

  bool readFile(const std::string& fileName);
  std::vector<std::shared_ptr<InformationSpecification>> getOffered();
  std::vector<std::shared_ptr<InformationSpecification>> getRequired();

private:
  bool readInformation(TiXmlElement* element, std::vector<std::shared_ptr<InformationSpecification>> &infos);

private:
  std::vector<std::shared_ptr<InformationSpecification>> offered;
  std::vector<std::shared_ptr<InformationSpecification>> required;
  el::Logger* _log; /**< Logger */
};

} /* namespace ice */

#endif /* XMLINFORMATIONREADER_H_ */
