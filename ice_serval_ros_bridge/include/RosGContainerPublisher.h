/*
 * RosGMessagePublisher.h
 *
 *  Created on: Apr 22, 2016
 *      Author: sni
 */

#ifndef ROSGMESSAGEPUBLISHER_H_
#define ROSGMESSAGEPUBLISHER_H_

#include <memory>
#include <vector>

#include <easylogging++.h>
#include <ice/representation/GContainer.h>
#include <tinyxml.h>


namespace ice
{

class RequiredInfo;
class OntologyInterface;

struct MessageTemplate
{
  std::string message;
  std::string representation;
  std::string messageTemplate;
};

class RosGContainerPublisher
{
public:
  RosGContainerPublisher(std::shared_ptr<OntologyInterface> ontology, std::string templateXMLFile);
  virtual ~RosGContainerPublisher();
  bool init();
  bool cleanUp();

  bool findTemplate(std::string const &message, std::string const &representation, MessageTemplate &msgTemplate);
  bool publish(std::shared_ptr<RequiredInfo> const &reqInfo, std::shared_ptr<GContainer> &container);
  bool transformToMessage(std::string &message, std::shared_ptr<GContainer> &container);

private:
  bool publish(std::string const &topic, std::string const &messageName, std::string const &message);
  bool readXMLFile(const std::string& fileName);
  bool readTemplate(TiXmlElement* element);

private:
  std::vector<MessageTemplate> messageTemplates;
  std::shared_ptr<OntologyInterface> ontology;
  std::string templateXMLFile;
  el::Logger *_log;
};

} /* namespace ice */

#endif /* ROSGMESSAGEPUBLISHER_H_ */
