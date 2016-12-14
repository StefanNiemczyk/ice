/*
 * GPSPosition.cpp
 *
 *  Created on: 14.09.2016
 *      Author: sni
 */

#include "container/WGS84.h"

#include <iomanip>
#include <limits>

namespace ice
{

int WGS84::ALTITUDE_PATH = 0;
int WGS84::LATITUDE_PATH = 1;
int WGS84::LONGITUDE_PATH = 2;

WGS84::WGS84(std::shared_ptr<Representation> const &representation) :
    GContainer(representation), altitude(0), latitude(0), longitude(0)
{
  //
}

WGS84::~WGS84()
{
  //
}

GContainer* WGS84::clone()
{
  auto pos = new WGS84(this->representation);

  pos->altitude = this->altitude;
  pos->latitude = this->latitude;
  pos->longitude = this->longitude;

  return pos;
}

void WGS84::print(int level, std::string dimension)
{
  // TODO
}

std::pair<BasicRepresentationType, void*> WGS84::getPair(std::vector<int> *indices, int index)
{
  if (indices->at(index) == ALTITUDE_PATH)
  {
    return std::make_pair(BasicRepresentationType::DOUBLE, &this->altitude);
  }
  else if (indices->at(index) == LATITUDE_PATH)
  {
    return std::make_pair(BasicRepresentationType::DOUBLE, &this->latitude);
  }
  else if (indices->at(index) == LONGITUDE_PATH)
  {
    return std::make_pair(BasicRepresentationType::DOUBLE, &this->longitude);
  }

  return std::make_pair(BasicRepresentationType::UNSET, nullptr);
}

void* WGS84::get(std::vector<int> *indices, int index)
{
  if (indices->at(index) == ALTITUDE_PATH)
  {
    return &this->altitude;
  }
  else if (indices->at(index) == LATITUDE_PATH)
  {
    return &this->latitude;
  }
  else if(indices->at(index) == LONGITUDE_PATH)
  {
    return &this->longitude;
  }

  return nullptr;
}

bool WGS84::set(std::vector<int> *indices, int index, const void* value)
{
  if(indices->at(index) == ALTITUDE_PATH)
  {
    this->altitude = *((double*) value);
    return true;
  }
  else if (indices->at(index) == LATITUDE_PATH)
  {
    this->latitude = *((double*) value);
    return true;
  }
  else if (indices->at(index) == LONGITUDE_PATH)
  {
    this->longitude = *((double*) value);
    return true;
  }

  return false;
}

std::string WGS84::toJSON()
{
  std::stringstream ss;
  ss << std::setprecision (std::numeric_limits<double>::digits10 + 1);
  ss << "{\"http://vs.uni-kassel.de/Ice#WGS84Rep\":{\"http://vs.uni-kassel.de/Ice#Altitude\":{\"http://vs.uni-kassel.de/Ice#DoubleRep\":"
     << this->altitude
     << "},\"http://vs.uni-kassel.de/Ice#Latitude\":{\"http://vs.uni-kassel.de/Ice#DoubleRep\":"
     << this->latitude
     << "},\"http://vs.uni-kassel.de/Ice#Longitude\":{\"http://vs.uni-kassel.de/Ice#DoubleRep\":"
     << this->longitude
     << "}}}";

  return ss.str();
}

Value WGS84::toJSONValue(Document &d)
{
  // TODO
  Value value;
  return value;
}

bool WGS84::readFromJSON(Value& value)
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
    if (name == "http://vs.uni-kassel.de/Ice#Altitude")
    {
      auto &num = value.MemberBegin()->value;
      if (false == num.IsDouble())
      {
        _log->error("Payload could not be parsed: XCoordinate is not a double value");
        return false;
      }
      this->altitude = num.GetDouble();
    }
    else if(name == "http://vs.uni-kassel.de/Ice#Latitude")
    {
      auto &num = value.MemberBegin()->value;
      if (false == num.IsDouble())
      {
        _log->error("Payload could not be parsed: YCoordinate is not a double value");
        return false;
      }
      this->latitude = num.GetDouble();
    }
    else if(name == "http://vs.uni-kassel.de/Ice#Longitude")
    {
      auto &num = value.MemberBegin()->value;
      if (false == num.IsDouble())
      {
        _log->error("Payload could not be parsed: ZCoordinate is not a double value");
        return false;
      }
      this->longitude = num.GetDouble();
    }
    else
    {
      _log->error("Payload could not be parsed: '%v' is unknown", name);
      return false;
    }
  }

  return true;
}

bool WGS84::isGeneric()
{
  return false;
}

} /* namespace ice */
