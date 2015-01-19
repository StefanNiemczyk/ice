/*
 * XMLReader.h
 *
 *  Created on: May 27, 2014
 *      Author: sni
 */

#ifndef XMLREADER_H_
#define XMLREADER_H_

#include <iostream>
#include <string>
#include <vector>
#include <map>

#include "tinyxml.h"

#include "ice/processing/Node.h"
#include "easylogging++.h"

namespace ice
{

struct XMLInformation {
  std::string uuid;
  std::string topic;
  std::string type;
  std::string _desc;

  std::vector<XMLInformation*> nested;
};

struct XMLStream {
  std::string name;
  std::string informationUuid;
  std::string provider;
  std::string sharingState = "inactive";
  int sharingMaxCount = 0;
  int size = -1;
  std::string _desc;
  bool trigger = false;
};

struct XMLStreamTeamplate {
  std::string name;
  std::string informationUuid;
  std::string provider;
  int maxStreamCount = 0;
  int size = -1;
  std::string _desc;
  bool trigger = false;
};

struct XMLNode {
  std::string name;
  NodeType type;
  std::string source;
  std::string className;
  long trigger = -1;
  std::string _desc;
  std::vector<XMLStream*> inputs;
  std::vector<XMLStreamTeamplate*> inputTemplates;
  std::vector<XMLStream*> outputs;
  std::map<std::string, std::string> configs;
};

class XMLReader
{
public:
  XMLReader();
  virtual ~XMLReader();

  bool readFile(const std::string& fileName);
  bool readFiles(std::initializer_list<std::string> fileNameList);
  std::vector<XMLInformation*>* getInformations();
  std::vector<XMLStream*>* getStreams();
  std::vector<XMLStreamTeamplate*>* getStreamTemplates();
  std::vector<XMLNode*>* getNodes();
  void clear();

private:
  XMLInformation* readInformation(TiXmlElement* element);
  XMLStream* readStream(TiXmlElement* element, bool checkName);
  bool readStreamSharing(TiXmlElement* element, XMLStream* stream);
  XMLStreamTeamplate* readStreamTemplate(TiXmlElement* element, bool checkName);
  XMLNode* readNode(TiXmlElement* element);
  void clearInformation(XMLInformation* information);

private:
  std::vector<XMLInformation*> informations;
  std::vector<XMLStream*> streams;
  std::vector<XMLStreamTeamplate*> streamTemplates;
  std::vector<XMLNode*> nodes;
  el::Logger* _log;
};

} /* namespace ice */

#endif /* XMLREADER_H_ */
