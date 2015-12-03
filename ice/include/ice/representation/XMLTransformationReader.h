/*
 * XMLTransformationReader.h
 *
 *  Created on: Nov 26, 2015
 *      Author: sni
 */

#ifndef XMLTRANSFORMATIONREADER_H_
#define XMLTRANSFORMATIONREADER_H_

#include <memory>
#include <string>
#include <vector>

#include "tinyxml.h"
#include "easylogging++.h"

namespace ice
{

enum XMLDimensionOperations {
  XML_USE,
  XML_DEFAULT,
  XML_FORMULA,
  XML_COMPLEX
};

struct TransInput {
  int id;
  std::string representation;
};

struct DimensionDesc {
  std::string name;
  XMLDimensionOperations type;

  // use
  int sourceId;
  std::string path;

  // default
  std::string value;

  // formula
  std::string formula;
  std::string varname;

  // compelex
  std::vector<DimensionDesc> dims;
};

struct TransDesc {
  std::string name;
  std::string scope;
  std::string output;

  std::vector<TransInput> inputs;
  std::vector<DimensionDesc> ops;
};

class XMLTransformationReader
{
public:
  XMLTransformationReader();
  virtual ~XMLTransformationReader();

  bool readFile(const std::string& fileName);

  void clear();
  std::vector<TransDesc*>& getTransformations();

private:
  TransDesc* readTransformation(TiXmlElement* element);
  bool readOperations(TiXmlElement* element, std::vector<DimensionDesc>& dimensions);

private:
  el::Logger* _log; /**< Logger */
  std::vector<TransDesc*> transformationDescs; /**< The descriptions of transformations */
};

} /* namespace ice */

#endif /* XMLTRANSFORMATIONREADER_H_ */
