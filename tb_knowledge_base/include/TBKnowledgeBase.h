/*
 * TBKnowledgeBase.h
 *
 *  Created on: Dec 8, 2016
 *      Author: sni
 */

#ifndef INCLUDE_TBKNOWLEDGEBASE_H_
#define INCLUDE_TBKNOWLEDGEBASE_H_

#include <ice/ICEngine.h>
#include <easylogging++.h>

namespace ice
{

class TBKnowledgeBase : public ICEngine
{
public:
  TBKnowledgeBase();
  virtual ~TBKnowledgeBase();

  virtual void init();

private:
  el::Logger                                    *_log;
};

} /* namespace ice */

#endif /* INCLUDE_TBKNOWLEDGEBASE_H_ */
