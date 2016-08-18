/*
 * InformationMap.h
 *
 *  Created on: 18.08.2016
 *      Author: sni
 */

#ifndef INCLUDE_INFORMATIONMAP_H_
#define INCLUDE_INFORMATIONMAP_H_

#include <memory>
#include <vector>

namespace ice
{

class ICEngine;
class CommunicationInterface;
class EntityDirectory;

class InformationMap
{
public:
  InformationMap(std::weak_ptr<ICEngine> engine);
  virtual ~InformationMap();

private:
  std::weak_ptr<ICEngine>                       engine;
  std::shared_ptr<CommunicationInterface>       communication;
  std::shared_ptr<EntityDirectory>              directory;
};

} /* namespace ice */

#endif /* INCLUDE_INFORMATIONMAP_H_ */
