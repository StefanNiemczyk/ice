/*
 * TBLocalization.h
 *
 *  Created on: Dec 8, 2016
 *      Author: sni
 */

#ifndef INCLUDE_NODE_TBLOCALIZATION_H_
#define INCLUDE_NODE_TBLOCALIZATION_H_

#include <memory>

#include <ice/processing/Node.h>

namespace ice
{
class GContainer;
template <typename T>
class InformationStream;

class TBLocalization : public Node
{
public:
  static std::shared_ptr<ice::Node> createNode();

public:
  TBLocalization();
  virtual ~TBLocalization();

  int init();

  virtual std::string getClassName();
  virtual int performNode();

private:
  static std::string POS_REP;

private:
  std::shared_ptr<ice::InformationStream<GContainer>>   output;

};

} /* namespace ice */

#endif /* INCLUDE_NODE_TBLOCALIZATION_H_ */
