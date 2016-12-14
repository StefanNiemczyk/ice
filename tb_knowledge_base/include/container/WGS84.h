/*
 * GPSPosition.h
 *
 *  Created on: 14.09.2016
 *      Author: sni
 */

#ifndef INCLUDE_WGS84_H_
#define INCLUDE_WGS84_H_

#include <ice/representation/GContainer.h>

namespace ice
{

class WGS84 : public GContainer
{
private:
  static int ALTITUDE_PATH, LATITUDE_PATH, LONGITUDE_PATH;

public:
  WGS84(std::shared_ptr<Representation> const &representation);
  virtual ~WGS84();

  virtual GContainer* clone();
  virtual void print(int level, std::string dimension = "");
  virtual std::pair<BasicRepresentationType, void*> getPair(std::vector<int> *indices, int index);
  virtual void* get(std::vector<int> *indices, int index);

  virtual bool set(std::vector<int> *indices, const void* value) {
          return this->set(indices, 0, value);
  }
  virtual bool set(std::vector<int> *indices, int index, const void* value);
  virtual std::string toJSON();
  virtual Value toJSONValue(Document &d);

  virtual bool readFromJSON(Value& value);
  virtual bool isGeneric();

public:
  double altitude;
  double latitude;
  double longitude;
};

} /* namespace ice */

#endif /* INCLUDE_POS3D_H_ */
