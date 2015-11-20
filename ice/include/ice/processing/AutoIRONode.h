/*
 * AutoIRONode.h
 *
 *  Created on: Nov 20, 2015
 *      Author: sni
 */

#ifndef AUTOIRONODE_H_
#define AUTOIRONODE_H_

#include "ice/processing/Node.h"

namespace ice
{

class AutoIRONode : public Node
{
public:
  AutoIRONode();
  virtual ~AutoIRONode();

  virtual int init();

  virtual int cleanUp();

  virtual int performNode();
};

} /* namespace ice */

#endif /* AUTOIRONODE_H_ */
