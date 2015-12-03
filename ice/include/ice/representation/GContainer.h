#ifndef REPRESENTATION_INSTANCE_H
#define REPRESENTATION_INSTANCE_H

#include <iostream>
#include <vector>

#include "ice/representation/Representation.h"

#include "easylogging++.h"

namespace ice
{

class GContainer
{
public:
  static el::Logger* getLogger()
  {
    static el::Logger* log = el::Loggers::getLogger("GContainer");
    return log;
  }

public:
  GContainer(std::shared_ptr<Representation> rep)
  {
    _log = getLogger();
    this->representation = rep;
  }
  virtual ~GContainer()
  {
  }

  template<typename T>
    T getValue(std::vector<int> *indices)
    {
      return *((T*)this->get(indices));
    }

  virtual void* get(std::vector<int> *indices)
  {
    return this->get(indices, 0);
  }

  virtual bool set(std::vector<int> *indices, const void* value)
  {
    return this->set(indices, 0, value);
  }

  virtual GContainer* clone() = 0;

  void print()
  {
    this->print(0);
  }

  virtual void print(int level, std::string dimension = "") = 0;
  virtual void* get(std::vector<int> *indices, int index) = 0;
  virtual bool set(std::vector<int> *indices, int index, const void* value) = 0;

protected:
  std::shared_ptr<Representation> representation;
  el::Logger* _log;
};

class CompositeGContainer : public GContainer
{
  friend class GContainerFactory;

public:
  CompositeGContainer(std::shared_ptr<Representation> rep) :
      GContainer(rep)
  {
    //
  }

  virtual ~CompositeGContainer()
  {
    for (int i=0; i < this->subs.size(); ++i)
    {
      delete this->subs.at(i);
    }
  }

  virtual GContainer* clone()
  {
    CompositeGContainer* instance = new CompositeGContainer(this->representation);

    for (int i = 0; i < this->subs.size(); ++i)
    {
      instance->subs.push_back(this->subs.at(i)->clone());
    }

    return instance;
  }

  virtual void print(int level, std::string dimension)
  {
    if (dimension != "")
    {
      std::cout << std::string(level, ' ') << dimension << " " << this->representation->name << std::endl;
    }
    else
    {
      std::cout << std::string(level, ' ') << this->representation->name << std::endl;
    }

    for (int i = 0; i < this->subs.size(); ++i)
    {
      auto sub = this->subs.at(i);
      sub->print(level + 1, this->representation->dimensionNames.at(i));
    }
  }

  virtual void* get(std::vector<int> *indices, int index)
  {
    if (indices->size() <= index)
    {
      return this->clone();
    }

    if (this->subs.size() < indices->at(index))
    {
      _log->error("Index out of bounds in get value from CompositeGContainer '%v', index '%v'",
                  this->representation->name, index);
      return nullptr;
    }

    return this->subs.at(indices->at(index))->get(indices, index + 1);
  }

  virtual bool set(std::vector<int> *indices, int index, const void* value)
  {
    if (indices->size() <= index)
    {
      CompositeGContainer* cgc = (CompositeGContainer*) value;

      for (int i=0; i < subs.size(); ++i)
      {
        delete this->subs.at(i);
      }
// ugly
      this->subs = cgc->subs;

      return true;
    }

    if (this->subs.size() < indices->at(index))
    {
      _log->error("Index out of bounds in get value from CompositeGContainer '%v', index '%v'",
                  this->representation->name, index);
      return false;
    }

    return this->subs.at(indices->at(index))->set(indices, index + 1, value);
  }

private:
  std::vector<GContainer*> subs;
};

class BasicGContainer : public GContainer
{
public:
  BasicGContainer(std::shared_ptr<Representation> rep) :
      GContainer(rep)
  {
    this->type = BasicRepresentationType::UNSET;
  }

  virtual ~BasicGContainer()
  {
    //
  }

  virtual GContainer* clone() = 0;

  virtual void* get(std::vector<int> *indices, int index) = 0;
  virtual bool set(std::vector<int> *indices, int index, const void* value) = 0;
  virtual void print(int level, std::string dimension) = 0;

protected:
  BasicRepresentationType type;
};

//* BoolGContainer
/**
 * Boolean representation
 */
class BoolGContainer : public BasicGContainer
{
public:
  BoolGContainer(std::shared_ptr<Representation> rep) :
      BasicGContainer(rep)
  {
    type = BasicRepresentationType::BOOL;
    this->value = false;
  }

  virtual GContainer* clone()
  {
    BoolGContainer* instance = new BoolGContainer(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* get(std::vector<int> *indices, int index)
  {
    if (indices->size() != index)
    {
      _log->error("Get value of BoolGContainer '%v' at not end of path, index '%v'",
                  this->representation->name, index);
      return nullptr;
    }

    return &this->value;
  }

  virtual bool set(std::vector<int> *indices, int index, const void* value)
  {
    if (indices->size() != index)
    {
      _log->error("Set value of BoolGContainer '%v' at not end of path, index '%v'",
                  this->representation->name, index);
      return false;
    }

    this->value = *((bool*)value);

    return true;
  }

  virtual void print(int level, std::string dimension)
  {
    std::cout << std::string(level, ' ') << dimension << " " << this->value << " (bool)" << std::endl;
  }

private:
  bool value;
};

//* ByteGContainer
/**
 * Byte representation
 */
class ByteGContainer : public BasicGContainer
{
public:
  ByteGContainer(std::shared_ptr<Representation> rep) :
      BasicGContainer(rep)
  {
    type = BasicRepresentationType::BYTE;
    this->value = 0;
  }

  virtual GContainer* clone()
  {
    ByteGContainer* instance = new ByteGContainer(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* get(std::vector<int> *indices, int index)
  {
    if (indices->size() != index)
    {
      _log->error("Get value of ByteGContainer '%v' at not end of path, index '%v'",
                  this->representation->name, index);
      return nullptr;
    }

    return &this->value;
  }

  virtual bool set(std::vector<int> *indices, int index, const void* value)
  {
    if (indices->size() != index)
    {
      _log->error("Set value of ByteGContainer '%v' at not end of path, index '%v'",
                  this->representation->name, index);
      return false;
    }

    this->value = *((int8_t*)value);

    return true;
  }

  virtual void print(int level, std::string dimension)
  {
    std::cout << std::string(level, ' ') << dimension << " " << this->value << " (byte)" << std::endl;
  }

private:
  int8_t value;
};

//* UnsignedByteGContainer
/**
 * Unsigned byte representation
 */
class UnsignedByteGContainer : public BasicGContainer
{
public:
  UnsignedByteGContainer(std::shared_ptr<Representation> rep) :
      BasicGContainer(rep)
  {
    type = BasicRepresentationType::UNSIGNED_BYTE;
    this->value = 0;
  }

  virtual GContainer* clone()
  {
    UnsignedByteGContainer* instance = new UnsignedByteGContainer(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* get(std::vector<int> *indices, int index)
  {
    if (indices->size() != index)
    {
      _log->error("Get value of UnsignedByteGContainer '%v' at not end of path, index '%v'",
                  this->representation->name, index);
      return nullptr;
    }

    return &this->value;
  }

  virtual bool set(std::vector<int> *indices, int index, const void* value)
  {
    if (indices->size() != index)
    {
      _log->error("Set value of UnsignedByteGContainer '%v' at not end of path, index '%v'",
                  this->representation->name, index);
      return false;
    }

    this->value = *((uint8_t*)value);

    return true;
  }

  virtual void print(int level, std::string dimension)
  {
    std::cout << std::string(level, ' ') << dimension << " " << this->value << " (unsigned byte)" << std::endl;
  }

private:
  uint8_t value;
};

//* ShortGContainer
/**
 * Short representation
 */
class ShortGContainer : public BasicGContainer
{
public:
  ShortGContainer(std::shared_ptr<Representation> rep) :
      BasicGContainer(rep)
  {
    type = BasicRepresentationType::SHORT;
    this->value = 0;
  }

  virtual GContainer* clone()
  {
    ShortGContainer* instance = new ShortGContainer(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* get(std::vector<int> *indices, int index)
  {
    if (indices->size() != index)
    {
      _log->error("Get value of ShortGContainer '%v' at not end of path, index '%v'",
                  this->representation->name, index);
      return nullptr;
    }

    return &this->value;
  }

  virtual bool set(std::vector<int> *indices, int index, const void* value)
  {
    if (indices->size() != index)
    {
      _log->error("Set value of ShortGContainer '%v' at not end of path, index '%v'",
                  this->representation->name, index);
      return false;
    }

    this->value = *((short*)value);

    return true;
  }

  virtual void print(int level, std::string dimension)
  {
    std::cout << std::string(level, ' ') << dimension << " " << this->value << " (short)" << std::endl;
  }

private:
  short value;
};

//* IntGContainer
/**
 * Integer representation
 */
class IntGContainer : public BasicGContainer
{
public:
  IntGContainer(std::shared_ptr<Representation> rep) :
      BasicGContainer(rep)
  {
    type = BasicRepresentationType::INT;
    this->value = 0;
  }

  virtual GContainer* clone()
  {
    IntGContainer* instance = new IntGContainer(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* get(std::vector<int> *indices, int index)
  {
    if (indices->size() != index)
    {
      _log->error("Get value of IntGContainer '%v' at not end of path, index '%v'",
                  this->representation->name, index);
      return nullptr;
    }

    return &this->value;
  }

  virtual bool set(std::vector<int> *indices, int index, const void* value)
  {
    if (indices->size() != index)
    {
      _log->error("Set value of IntGContainer '%v' at not end of path, index '%v'",
                  this->representation->name, index);
      return false;
    }

    this->value = *((int*)value);

    return true;
  }

  virtual void print(int level, std::string dimension)
  {
    std::cout << std::string(level, ' ') << dimension << " " << this->value << " (int)" << std::endl;
  }

private:
  int value;
};

//* LongGContainer
/**
 * Long representation
 */
class LongGContainer : public BasicGContainer
{
public:
  LongGContainer(std::shared_ptr<Representation> rep) :
      BasicGContainer(rep)
  {
    type = BasicRepresentationType::LONG;
    this->value = 0;
  }

  virtual GContainer* clone()
  {
    LongGContainer* instance = new LongGContainer(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* get(std::vector<int> *indices, int index)
  {
    if (indices->size() != index)
    {
      _log->error("Get value of LongGContainer '%v' at not end of path, index '%v'",
                  this->representation->name, index);
      return nullptr;
    }

    return &this->value;
  }

  virtual bool set(std::vector<int> *indices, int index, const void* value)
  {
    if (indices->size() != index)
    {
      _log->error("Set value of LongGContainer '%v' at not end of path, index '%v'",
                  this->representation->name, index);
      return false;
    }

    this->value = *((long*)value);

    return true;
  }

  virtual void print(int level, std::string dimension)
  {
    std::cout << std::string(level, ' ') << dimension << " " << this->value << " (long)" << std::endl;
  }

private:
  long value;
};

//* UnsignedShortGContainer
/**
 * Unsigned short representation
 */
class UnsignedShortGContainer : public BasicGContainer
{
public:
  UnsignedShortGContainer(std::shared_ptr<Representation> rep) :
      BasicGContainer(rep)
  {
    type = BasicRepresentationType::UNSIGNED_SHORT;
    this->value = 0;
  }

  virtual GContainer* clone()
  {
    UnsignedShortGContainer* instance = new UnsignedShortGContainer(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* get(std::vector<int> *indices, int index)
  {
    if (indices->size() != index)
    {
      _log->error("Get value of UnsignedShortGContainer '%v' at not end of path, index '%v'",
                  this->representation->name, index);
      return nullptr;
    }

    return &this->value;
  }

  virtual bool set(std::vector<int> *indices, int index, const void* value)
  {
    if (indices->size() != index)
    {
      _log->error("Set value of UnsignedShortGContainer '%v' at not end of path, index '%v'",
                  this->representation->name, index);
      return false;
    }

    this->value = *((unsigned short*)value);

    return true;
  }

  virtual void print(int level, std::string dimension)
  {
    std::cout << std::string(level, ' ') << dimension << " " << this->value << " (unsigned short)" << std::endl;
  }

private:
  unsigned short value;
};

//* UnsignedIntGContainer
/**
 * Unsigned integer representation
 */
class UnsignedIntGContainer : public BasicGContainer
{
public:
  UnsignedIntGContainer(std::shared_ptr<Representation> rep) :
      BasicGContainer(rep)
  {
    type = BasicRepresentationType::UNSIGNED_INT;
    this->value = 0;
  }

  virtual GContainer* clone()
  {
    UnsignedIntGContainer* instance = new UnsignedIntGContainer(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* get(std::vector<int> *indices, int index)
  {
    if (indices->size() != index)
    {
      _log->error("Get value of UnsignedIntGContainer '%v' at not end of path, index '%v'",
                  this->representation->name, index);
      return nullptr;
    }

    return &this->value;
  }

  virtual bool set(std::vector<int> *indices, int index, const void* value)
  {
    if (indices->size() != index)
    {
      _log->error("Set value of UnsignedIntGContainer '%v' at not end of path, index '%v'",
                  this->representation->name, index);
      return false;
    }

    this->value = *((unsigned int*)value);

    return true;
  }

  virtual void print(int level, std::string dimension)
  {
    std::cout << std::string(level, ' ') << dimension << " " << this->value << " (unsigned int)" << std::endl;
  }

private:
  unsigned int value;
};

//* UnsignedLongGContainer
/**
 * unsigned long representation
 */
class UnsignedLongGContainer : public BasicGContainer
{
public:
  UnsignedLongGContainer(std::shared_ptr<Representation> rep) :
      BasicGContainer(rep)
  {
    type = BasicRepresentationType::UNSIGNED_LONG;
    this->value = 0;
  }

  virtual GContainer* clone()
  {
    UnsignedLongGContainer* instance = new UnsignedLongGContainer(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* get(std::vector<int> *indices, int index)
  {
    if (indices->size() != index)
    {
      _log->error("Get value of UnsignedLongGContainer '%v' at not end of path, index '%v'",
                  this->representation->name, index);
      return nullptr;
    }

    return &this->value;
  }

  virtual bool set(std::vector<int> *indices, int index, const void* value)
  {
    if (indices->size() != index)
    {
      _log->error("Set value of UnsignedLongGContainer '%v' at not end of path, index '%v'",
                  this->representation->name, index);
      return false;
    }

    this->value = *((unsigned long*)value);

    return true;
  }

  virtual void print(int level, std::string dimension)
  {
    std::cout << std::string(level, ' ') << dimension << " " << this->value << " (unsigned long)" << std::endl;
  }

private:
  unsigned long value;
};

//* FloatGContainer
/**
 * Float representation
 */
class FloatGContainer : public BasicGContainer
{
public:
  FloatGContainer(std::shared_ptr<Representation> rep) :
      BasicGContainer(rep)
  {
    type = BasicRepresentationType::FLOAT;
    this->value = 0;
  }

  virtual GContainer* clone()
  {
    FloatGContainer* instance = new FloatGContainer(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* get(std::vector<int> *indices, int index)
  {
    if (indices->size() != index)
    {
      _log->error("Get value of FloatGContainer '%v' at not end of path, index '%v'",
                  this->representation->name, index);
      return nullptr;
    }

    return &this->value;
  }

  virtual bool set(std::vector<int> *indices, int index, const void* value)
  {
    if (indices->size() != index)
    {
      _log->error("Set value of FloatGContainer '%v' at not end of path, index '%v'",
                  this->representation->name, index);
      return false;
    }

    this->value = *((float*)value);

    return true;
  }

  virtual void print(int level, std::string dimension)
  {
    std::cout << std::string(level, ' ') << dimension << " " << this->value << " (float)" << std::endl;
  }

private:
  float value;
};

//* DoubleGContainer
/**
 * Double representation
 */
class DoubleGContainer : public BasicGContainer
{
public:
  DoubleGContainer(std::shared_ptr<Representation> rep) :
      BasicGContainer(rep)
  {
    type = BasicRepresentationType::DOUBLE;
    this->value = 0;
  }

  virtual GContainer* clone()
  {
    DoubleGContainer* instance = new DoubleGContainer(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* get(std::vector<int> *indices, int index)
  {
    if (indices->size() != index)
    {
      _log->error("Get value of DoubleGContainer '%v' at not end of path, index '%v'",
                  this->representation->name, index);
      return nullptr;
    }

    return &this->value;
  }

  virtual bool set(std::vector<int> *indices, int index, const void* value)
  {
    if (indices->size() != index)
    {
      _log->error("Set value of DoubleGContainer '%v' at not end of path, index '%v'",
                  this->representation->name, index);
      return false;
    }

    this->value = *((double*)value);

    return true;
  }

  virtual void print(int level, std::string dimension)
  {
    std::cout << std::string(level, ' ') << dimension << " " << this->value << " (double)" << std::endl;
  }

private:
  double value;
};

//* StringGContainer
/**
 * String representation
 */
class StringGContainer : public BasicGContainer
{
public:
  StringGContainer(std::shared_ptr<Representation> rep) :
      BasicGContainer(rep)
  {
    type = BasicRepresentationType::STRING;
  }

  virtual GContainer* clone()
  {
    StringGContainer* instance = new StringGContainer(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* get(std::vector<int> *indices, int index)
  {
    if (indices->size() != index)
    {
      _log->error("Get value of StringGContainer '%v' at not end of path, index '%v'",
                  this->representation->name, index);
      return nullptr;
    }

    return &this->value;
  }

  virtual bool set(std::vector<int> *indices, int index, const void* value)
  {
    if (indices->size() != index)
    {
      _log->error("Set value of StringGContainer '%v' at not end of path, index '%v'",
                  this->representation->name, index);
      return false;
    }

    this->value = *((std::string*)value);

    return true;
  }

  virtual void print(int level, std::string dimension)
  {
    std::cout << std::string(level, ' ') << dimension << " " << this->value << " (string)" << std::endl;
  }

private:
  std::string value;
};

}

#endif // REPRESENTATION_INSTANCE_H