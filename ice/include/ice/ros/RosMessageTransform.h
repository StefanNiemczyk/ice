/*
 * RosMessageTransform.h
 *
 *  Created on: Jun 18, 2014
 *      Author: sni
 */

#ifndef ROSMESSAGETRANSFORM_H_
#define ROSMESSAGETRANSFORM_H_

#include <iostream>
#include <memory>
#include <vector>

#include "boost/shared_ptr.hpp"

#include "ice/container/Position.h"
#include "ice/information/InformationElement.h"

#include "ice_msgs/Position.h"
#include "ice_msgs/Positions.h"

namespace ice
{

class RosMessageTransform
{
public:
  // ice::Position <-> ice_msgs::Position
  static std::unique_ptr<ice_msgs::Position> transformC2MPosition(
      std::shared_ptr<InformationElement<Position>> informationElement);
  static std::unique_ptr<Position> transformM2CPosition(const boost::shared_ptr<ice_msgs::Position const> msg);


  // ice::Position[] <-> ice_msgs::Positions
  static std::unique_ptr<ice_msgs::Positions> transformC2MPositions(
      std::shared_ptr<InformationElement<std::vector<Position>>> informationElement);
  static std::unique_ptr<std::vector<Position>> transformM2CPositions(const boost::shared_ptr<ice_msgs::Positions const> msg);

};

} /* namespace ice */

#endif /* ROSMESSAGETRANSFORM_H_ */
