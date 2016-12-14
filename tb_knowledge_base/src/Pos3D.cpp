/*
 * GPSPosition.cpp
 *
 *  Created on: 14.09.2016
 *      Author: sni
 */

#include "container/Pos3D.h"

#include <iomanip>
#include <limits>

namespace ice
{

int Pos3D::X_COORDINATE_PATH = 0;
int Pos3D::Y_COORDINATE_PATH = 1;
int Pos3D::Z_COORDINATE_PATH = 2;

Pos3D::Pos3D(std::shared_ptr<Representation> const &representation) :
    GContainer(representation), x(0), y(0), z(0)
{
  //
}

Pos3D::~Pos3D()
{
  //
}

GContainer* Pos3D::clone()
{
  auto pos = new Pos3D(this->representation);

  pos->x = this->x;
  pos->y = this->y;
  pos->z = this->z;

  return pos;
}

void Pos3D::print(int level, std::string dimension)
{
  // TODO
}

std::pair<BasicRepresentationType, void*> Pos3D::getPair(std::vector<int> *indices, int index)
{
  if (indices->at(index) == X_COORDINATE_PATH)
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

void* Pos3D::get(std::vector<int> *indices, int index)
{
  if (indices->at(index) == X_COORDINATE_PATH)
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

bool Pos3D::set(std::vector<int> *indices, int index, const void* value)
{
  if(indices->at(index) == Z_COORDINATE_PATH)
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

std::string Pos3D::toJSON()
{
  std::stringstream ss;
  ss << std::setprecision (std::numeric_limits<double>::digits10 + 1);
  ss << "{\"http://vs.uni-kassel.de/Ice#CoordinatePositionRep\":{\"http://vs.uni-kassel.de/Ice#XCoordinate\":{\"http://vs.uni-kassel.de/Ice#DoubleRep\":"
     << this->x
     << "},\"http://vs.uni-kassel.de/Ice#YCoordinate\":{\"http://vs.uni-kassel.de/Ice#DoubleRep\":"
     << this->y
     << "},\"http://vs.uni-kassel.de/Ice#ZCoordinate\":{\"http://vs.uni-kassel.de/Ice#DoubleRep\":"
     << this->z
     << "}}}";

  return ss.str();
}

Value Pos3D::toJSONValue(Document &d)
{
  // TODO
  Value value;
  return value;
}

bool Pos3D::readFromJSON(Value& value)
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
    if (name == "http://vs.uni-kassel.de/Ice#XCoordinate")
    {
      auto &num = value.MemberBegin()->value;
      if (false == num.IsDouble())
      {
        _log->error("Payload could not be parsed: XCoordinate is not a double value");
        return false;
      }
      this->x = num.GetDouble();
    }
    else if(name == "http://vs.uni-kassel.de/Ice#YCoordinate")
    {
      auto &num = value.MemberBegin()->value;
      if (false == num.IsDouble())
      {
        _log->error("Payload could not be parsed: YCoordinate is not a double value");
        return false;
      }
      this->y = num.GetDouble();
    }
    else if(name == "http://vs.uni-kassel.de/Ice#ZCoordinate")
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

bool Pos3D::isGeneric()
{
  return false;
}

} /* namespace ice */
