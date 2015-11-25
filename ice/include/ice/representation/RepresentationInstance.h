#ifndef REPRESENTATION_INSTANCE_H
#define REPRESENTATION_INSTANCE_H

#include <iostream>
#include <vector>

#include "ice/representation/Representation.h"

namespace ice
{

class RepresentationInstance
{
public:
  RepresentationInstance(std::shared_ptr<Representation> rep)
  {
    this->representation = rep;
  }
  virtual ~RepresentationInstance()
  {
  }

  virtual void* get(int *indecies) = 0;

  virtual void set(int *indecies, const void* value) = 0;

  virtual RepresentationInstance* clone() = 0;

  void print()
  {
    this->print(0);
  }

  virtual void print(int level, std::string dimension = "") = 0;

protected:
  std::shared_ptr<Representation> representation;
};

class CompositeRepresentationInstance : public RepresentationInstance
{
  friend class RepresentationFactory;

public:
  CompositeRepresentationInstance(std::shared_ptr<Representation> rep) :
      RepresentationInstance(rep)
  {
    //
  }

  virtual ~CompositeRepresentationInstance()
  {
    //
  }

  virtual RepresentationInstance* clone()
  {
    CompositeRepresentationInstance* instance = new CompositeRepresentationInstance(this->representation);

    instance->subs = this->subs;

    return instance;
  }

  virtual void* get(int *indecies)
  {
    return this->subs.at(*indecies)->get(++indecies);
  }

  virtual void set(int *indecies, const void* value)
  {
    return this->subs.at(*indecies)->set(++indecies, value);
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

private:
  std::vector<std::shared_ptr<RepresentationInstance>> subs;
};

class BasicRepresentationInstance : public RepresentationInstance
{
public:
  BasicRepresentationInstance(std::shared_ptr<Representation> rep) :
      RepresentationInstance(rep)
  {
    this->type = BasicRepresentationType::UNSET;
  }

  virtual ~BasicRepresentationInstance()
  {
    //
  }

  virtual void* get(int *indecies)
  {
    return this->getRaw();
  }

  virtual void set(int *indecies, const void* value)
  {
    return this->setRaw(value);
  }

  virtual void* getRaw() = 0;
  virtual void setRaw(const void* value) = 0;
  virtual RepresentationInstance* clone() = 0;

protected:
  virtual void print(int level, std::string dimension) = 0;

protected:
  BasicRepresentationType type;
};

//* BoolRepresentationInstance
/**
 * Boolean representation
 */
class BoolRepresentationInstance : public BasicRepresentationInstance
{
public:
  BoolRepresentationInstance(std::shared_ptr<Representation> rep) :
      BasicRepresentationInstance(rep)
  {
    type = BasicRepresentationType::BOOL;
    this->value = false;
  }

  virtual RepresentationInstance* clone()
  {
    BoolRepresentationInstance* instance = new BoolRepresentationInstance(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* getRaw()
  {
    return &this->value;
  }

  virtual void setRaw(const void* value)
  {
    this->value = *((bool*)value);
  }

  virtual void print(int level, std::string dimension)
  {
    std::cout << std::string(level, ' ') << dimension << " " << this->value << " (bool)" << std::endl;
  }

private:
  bool value;
};

//* ByteRepresentationInstance
/**
 * Byte representation
 */
class ByteRepresentationInstance : public BasicRepresentationInstance
{
public:
  ByteRepresentationInstance(std::shared_ptr<Representation> rep) :
      BasicRepresentationInstance(rep)
  {
    type = BasicRepresentationType::BYTE;
    this->value = 0;
  }

  virtual RepresentationInstance* clone()
  {
    ByteRepresentationInstance* instance = new ByteRepresentationInstance(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* getRaw()
  {
    return &this->value;
  }

  virtual void setRaw(const void* value)
  {
    this->value = *((int8_t*)value);
  }

  virtual void print(int level, std::string dimension)
  {
    std::cout << std::string(level, ' ') << dimension << " " << this->value << " (byte)" << std::endl;
  }

private:
  int8_t value;
};

//* UnsignedByteRepresentationInstance
/**
 * Unsigned byte representation
 */
class UnsignedByteRepresentationInstance : public BasicRepresentationInstance
{
public:
  UnsignedByteRepresentationInstance(std::shared_ptr<Representation> rep) :
      BasicRepresentationInstance(rep)
  {
    type = BasicRepresentationType::UNSIGNED_BYTE;
    this->value = 0;
  }

  virtual RepresentationInstance* clone()
  {
    UnsignedByteRepresentationInstance* instance = new UnsignedByteRepresentationInstance(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* getRaw()
  {
    return &this->value;
  }

  virtual void setRaw(const void* value)
  {
    this->value = *((uint8_t*)value);
  }

  virtual void print(int level, std::string dimension)
  {
    std::cout << std::string(level, ' ') << dimension << " " << this->value << " (unsigned byte)" << std::endl;
  }

private:
  uint8_t value;
};

//* ShortRepresentationInstance
/**
 * Short representation
 */
class ShortRepresentationInstance : public BasicRepresentationInstance
{
public:
  ShortRepresentationInstance(std::shared_ptr<Representation> rep) :
      BasicRepresentationInstance(rep)
  {
    type = BasicRepresentationType::SHORT;
    this->value = 0;
  }

  virtual RepresentationInstance* clone()
  {
    ShortRepresentationInstance* instance = new ShortRepresentationInstance(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* getRaw()
  {
    return &this->value;
  }

  virtual void setRaw(const void* value)
  {
    this->value = *((short*)value);
  }

  virtual void print(int level, std::string dimension)
  {
    std::cout << std::string(level, ' ') << dimension << " " << this->value << " (short)" << std::endl;
  }

private:
  short value;
};

//* IntegerRepresentationInstance
/**
 * Integer representation
 */
class IntegerRepresentationInstance : public BasicRepresentationInstance
{
public:
  IntegerRepresentationInstance(std::shared_ptr<Representation> rep) :
      BasicRepresentationInstance(rep)
  {
    type = BasicRepresentationType::INT;
    this->value = 0;
  }

  virtual RepresentationInstance* clone()
  {
    IntegerRepresentationInstance* instance = new IntegerRepresentationInstance(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* getRaw()
  {
    return &this->value;
  }

  virtual void setRaw(const void* value)
  {
    this->value = *((int*)value);
  }

  virtual void print(int level, std::string dimension)
  {
    std::cout << std::string(level, ' ') << dimension << " " << this->value << " (int)" << std::endl;
  }

private:
  int value;
};

//* LongRepresentationInstance
/**
 * Long representation
 */
class LongRepresentationInstance : public BasicRepresentationInstance
{
public:
  LongRepresentationInstance(std::shared_ptr<Representation> rep) :
      BasicRepresentationInstance(rep)
  {
    type = BasicRepresentationType::LONG;
    this->value = 0;
  }

  virtual RepresentationInstance* clone()
  {
    LongRepresentationInstance* instance = new LongRepresentationInstance(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* getRaw()
  {
    return &this->value;
  }

  virtual void setRaw(const void* value)
  {
    this->value = *((long*)value);
  }

  virtual void print(int level, std::string dimension)
  {
    std::cout << std::string(level, ' ') << dimension << " " << this->value << " (long)" << std::endl;
  }

private:
  long value;
};

//* UnsignedShortRepresentationInstance
/**
 * Unsigned short representation
 */
class UnsignedShortRepresentationInstance : public BasicRepresentationInstance
{
public:
  UnsignedShortRepresentationInstance(std::shared_ptr<Representation> rep) :
      BasicRepresentationInstance(rep)
  {
    type = BasicRepresentationType::UNSIGNED_SHORT;
    this->value = 0;
  }

  virtual RepresentationInstance* clone()
  {
    UnsignedShortRepresentationInstance* instance = new UnsignedShortRepresentationInstance(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* getRaw()
  {
    return &this->value;
  }

  virtual void setRaw(const void* value)
  {
    this->value = *((unsigned short*)value);
  }

  virtual void print(int level, std::string dimension)
  {
    std::cout << std::string(level, ' ') << dimension << " " << this->value << " (unsigned short)" << std::endl;
  }

private:
  unsigned short value;
};

//* UnsignedIntegerRepresentationInstance
/**
 * Unsigned integer representation
 */
class UnsignedIntegerRepresentationInstance : public BasicRepresentationInstance
{
public:
  UnsignedIntegerRepresentationInstance(std::shared_ptr<Representation> rep) :
      BasicRepresentationInstance(rep)
  {
    type = BasicRepresentationType::UNSIGNED_INT;
    this->value = 0;
  }

  virtual RepresentationInstance* clone()
  {
    UnsignedIntegerRepresentationInstance* instance = new UnsignedIntegerRepresentationInstance(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* getRaw()
  {
    return &this->value;
  }

  virtual void setRaw(const void* value)
  {
    this->value = *((unsigned int*)value);
  }

  virtual void print(int level, std::string dimension)
  {
    std::cout << std::string(level, ' ') << dimension << " " << this->value << " (unsigned int)" << std::endl;
  }

private:
  unsigned int value;
};

//* UnsignedLongRepresentationInstance
/**
 * unsigned long representation
 */
class UnsignedLongRepresentationInstance : public BasicRepresentationInstance
{
public:
  UnsignedLongRepresentationInstance(std::shared_ptr<Representation> rep) :
      BasicRepresentationInstance(rep)
  {
    type = BasicRepresentationType::UNSIGNED_LONG;
    this->value = 0;
  }

  virtual RepresentationInstance* clone()
  {
    UnsignedLongRepresentationInstance* instance = new UnsignedLongRepresentationInstance(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* getRaw()
  {
    return &this->value;
  }

  virtual void setRaw(const void* value)
  {
    this->value = *((unsigned long*)value);
  }

  virtual void print(int level, std::string dimension)
  {
    std::cout << std::string(level, ' ') << dimension << " " << this->value << " (unsigned long)" << std::endl;
  }

private:
  unsigned long value;
};

//* FloatRepresentationInstance
/**
 * Float representation
 */
class FloatRepresentationInstance : public BasicRepresentationInstance
{
public:
  FloatRepresentationInstance(std::shared_ptr<Representation> rep) :
      BasicRepresentationInstance(rep)
  {
    type = BasicRepresentationType::FLOAT;
    this->value = 0;
  }

  virtual RepresentationInstance* clone()
  {
    FloatRepresentationInstance* instance = new FloatRepresentationInstance(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* getRaw()
  {
    return &this->value;
  }

  virtual void setRaw(const void* value)
  {
    this->value = *((float*)value);
  }

  virtual void print(int level, std::string dimension)
  {
    std::cout << std::string(level, ' ') << dimension << " " << this->value << " (float)" << std::endl;
  }

private:
  float value;
};

//* DoubleRepresentationInstance
/**
 * Double representation
 */
class DoubleRepresentationInstance : public BasicRepresentationInstance
{
public:
  DoubleRepresentationInstance(std::shared_ptr<Representation> rep) :
      BasicRepresentationInstance(rep)
  {
    type = BasicRepresentationType::DOUBLE;
    this->value = 0;
  }

  virtual RepresentationInstance* clone()
  {
    DoubleRepresentationInstance* instance = new DoubleRepresentationInstance(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* getRaw()
  {
    return &this->value;
  }

  virtual void setRaw(const void* value)
  {
    this->value = *((double*)value);
  }

  virtual void print(int level, std::string dimension)
  {
    std::cout << std::string(level, ' ') << dimension << " " << this->value << " (double)" << std::endl;
  }

private:
  double value;
};

//* StringRepresentationInstance
/**
 * String representation
 */
class StringRepresentationInstance : public BasicRepresentationInstance
{
public:
  StringRepresentationInstance(std::shared_ptr<Representation> rep) :
      BasicRepresentationInstance(rep)
  {
    type = BasicRepresentationType::STRING;
  }

  virtual RepresentationInstance* clone()
  {
    StringRepresentationInstance* instance = new StringRepresentationInstance(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* getRaw()
  {
    return &this->value;
  }

  virtual void setRaw(const void* value)
  {
    this->value = *((std::string*)value);
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
