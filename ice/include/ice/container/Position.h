/*
 * Position.h
 *
 *  Created on: Jun 18, 2014
 *      Author: sni
 */

#ifndef POSITION_H_
#define POSITION_H_

namespace ice
{

class Position
{
public:
  Position();
  virtual ~Position();

  int x;
  int y;
  int z;
};

} /* namespace ice */

#endif /* POSITION_H_ */
