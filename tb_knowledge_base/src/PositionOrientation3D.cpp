/*
 * PositionOrientation3D.cpp
 *
 *  Created on: Dec 13, 2016
 *      Author: sni
 */

#include "container/PositionOrientation3D.h"

#include <iomanip>
#include <limits>

namespace ice
{

int PositionOrientation3D::ALPHA_PATH = 0;
int PositionOrientation3D::X_COORDINATE_PATH = 1;
int PositionOrientation3D::Y_COORDINATE_PATH = 2;
int PositionOrientation3D::Z_COORDINATE_PATH = 3;

PositionOrientation3D::PositionOrientation3D(std::shared_ptr<Representation> const &representation) :
    GContainer(representation), alpha(0), x(0), y(0), z(0)
{
  //
}

PositionOrientation3D::~PositionOrientation3D()
{
  //
}

GContainer* PositionOrientation3D::clone()
{
  auto pos = new PositionOrientation3D(this->representation);

  pos->x = this->x;
  pos->y = this->y;
  pos->z = this->z;

  return pos;
}

void PositionOrientation3D::print(int level, std::string dimension)
{
  // TODO
}

std::pair<BasicRepresentationType, void*> PositionOrientation3D::getPair(std::vector<int> *indices, int index)
{
  if (indices->at(index) == ALPHA_PATH)
  {
    return std::make_pair(BasicRepresentationType::DOUBLE, &this->ALPHA_PATH);
  }
  else if (indices->at(index) == X_COORDINATE_PATH)
  {
    return std::make_pair(BasicRepresentationType::DOUBLE, &this->x);
  }
  else if (indices->at(index) == Y_COORDINATE_PATH)
  {
    return std::make_pair(BasicRepresentationType::DOUBLE, &this->y);
  }
  else if (indices->at(index) == Z_COORDINATE_PATH)
  {
    return std::make_pair(BasicRepresentationType::DOUBLE, &this->z);
  }

  return std::make_pair(BasicRepresentationType::UNSET, nullptr);
}

void* PositionOrientation3D::get(std::vector<int> *indices, int index)
{
  if (indices->at(index) == ALPHA_PATH)
  {
    return &this->alpha;
  }
  else if (indices->at(index) == X_COORDINATE_PATH)
  {
    return &this->x;
  }
  else if (indices->at(index) == Y_COORDINATE_PATH)
  {
    return &this->y;
  }
  else if(indices->at(index) == Z_COORDINATE_PATH)
  {
    return &this->z;
  }

  return nullptr;
}

bool PositionOrientation3D::set(std::vector<int> *indices, int index, const void* value)
{
  if (indices->at(index) == ALPHA_PATH)
  {
    this->alpha = *((double*) value);
    return true;
  }
  else if(indices->at(index) == Z_COORDINATE_PATH)
  {
    this->z = *((double*) value);
    return true;
  }
  else if (indices->at(index) == X_COORDINATE_PATH)
  {
    this->x = *((double*) value);
    return true;
  }
  else if (indices->at(index) == Y_COORDINATE_PATH)
  {
    this->y = *((double*) value);
    return true;
  }

  return false;
}

std::string PositionOrientation3D::toJSON()
{
  std::stringstream ss;
  ss << std::setprecision (std::numeric_limits<double>::digits10 + 1);
  ss << "{\"http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D\":{\"http://vs.uni-kassel.de/Ice#Alpha\":{\"http://vs.uni-kassel.de/Ice#DoubleRep\":"
     << this->alpha
     << "},\"http://vs.uni-kassel.de/Ice#XCoordinate\":{\"http://vs.uni-kassel.de/Ice#DoubleRep\":"
     << this->x
     << "},\"http://vs.uni-kassel.de/Ice#YCoordinate\":{\"http://vs.uni-kassel.de/Ice#DoubleRep\":"
     << this->y
     << "},\"http://vs.uni-kassel.de/Ice#ZCoordinate\":{\"http://vs.uni-kassel.de/Ice#DoubleRep\":"
     << this->z
     << "}}}";

  return ss.str();
}

Value PositionOrientation3D::toJSONValue(Document &d)
{
  // TODO
  Value value;
  return value;
}

bool PositionOrientation3D::readFromJSON(Value& value)
{
  if (false == value.IsObject())
    return false;

  for (auto it = value.MemberBegin(); it != value.MemberEnd(); ++it)
  {
    if (false == it->name.IsString() || false == it->value.IsObject())
    {
      _log->error("Payload could not be parsed: Id is not a string");
      return false;
    }

    auto name = std::string(it->name.GetString());
    auto value = it->value.GetObject();

    if (name == "http://vs.uni-kassel.de/Ice#Alpha")
    {
      auto &num = value.MemberBegin()->value;
      if (false == num.IsDouble())
      {
        _log->error("Payload could not be parsed: Alpha is not a double value");
        return false;
      }
      this->alpha = num.GetDouble();
    }
    else if (name == "http://vs.uni-kassel.de/Ice#XCoordinate")
    {
      auto &num = value.MemberBegin()->value;
      if (false == num.IsDouble())
      {
        _log->error("Payload could not be parsed: XCoordinate is not a double value");
        return false;
      }
      this->x = num.GetDouble();
    }
    else if (name == "http://vs.uni-kassel.de/Ice#YCoordinate")
    {
      auto &num = value.MemberBegin()->value;
      if (false == num.IsDouble())
      {
        _log->error("Payload could not be parsed: YCoordinate is not a double value");
        return false;
      }
      this->y = num.GetDouble();
    }
    else if (name == "http://vs.uni-kassel.de/Ice#ZCoordinate")
    {
      auto &num = value.MemberBegin()->value;
      if (false == num.IsDouble())
      {
        _log->error("Payload could not be parsed: ZCoordinate is not a double value");
        return false;
      }
      this->z = num.GetDouble();
    }
    else
    {
      _log->error("Payload could not be parsed: '%v' is unknown", name);
      return false;
    }
  }

  return true;
}

bool PositionOrientation3D::isGeneric()
{
  return false;
}

} /* namespace ice */
