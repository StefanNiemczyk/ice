/*
 * TBKnowledgeBase.h
 *
 *  Created on: Dec 8, 2016
 *      Author: sni
 */

#ifndef INCLUDE_TBKNOWLEDGEBASE_H_
#define INCLUDE_TBKNOWLEDGEBASE_H_

#include <memory>

#include <ice/ICEngine.h>
#include <easylogging++.h>

namespace ice
{
class GContainer;
template <typename T>
class InformationStream;
template <typename T>
class InformationSet;
class PositionOrientation3D;
class RTLandmark;

class TBKnowledgeBase : public ICEngine
{
public:
  TBKnowledgeBase(std::string robotName);
  virtual ~TBKnowledgeBase();

  virtual void init();
  virtual void start();

public:
  std::shared_ptr<InformationStream<PositionOrientation3D>>     positionOwn;
  std::shared_ptr<InformationSet<RTLandmark>>                   positionAll;
  std::shared_ptr<InformationSet<RTLandmark>>                   positionVictims;

private:
  std::string const                                     robotName;
  el::Logger                                            *_log;
};

} /* namespace ice */

#endif /* INCLUDE_TBKNOWLEDGEBASE_H_ */
