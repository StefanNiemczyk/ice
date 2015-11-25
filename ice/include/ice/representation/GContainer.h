#ifndef REPRESENTATION_INSTANCE_H
#define REPRESENTATION_INSTANCE_H

#include <iostream>
#include <vector>

#include "ice/representation/Representation.h"

namespace ice
{

class GContainer
{
public:
  GContainer(std::shared_ptr<Representation> rep)
  {
    this->representation = rep;
  }
  virtual ~GContainer()
  {
  }

  template <typename T>
  T getValue(int *indices)
  {
    return *((T*) this->get(indices));
  }

  virtual void* get(int *indices) = 0;

  virtual void set(int *indices, const void* value) = 0;

  virtual GContainer* clone() = 0;

  void print()
  {
    this->print(0);
  }

  virtual void print(int level, std::string dimension = "") = 0;

protected:
  std::shared_ptr<Representation> representation;
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
    //
  }

  virtual GContainer* clone()
  {
    CompositeGContainer* instance = new CompositeGContainer(this->representation);

    instance->subs = this->subs;

    return instance;
  }

  virtual void* get(int *indices)
  {
    return this->subs.at(*indices)->get(++indices);
  }

  virtual void set(int *indices, const void* value)
  {
    return this->subs.at(*indices)->set(++indices, value);
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
  std::vector<std::shared_ptr<GContainer>> subs;
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

  virtual void* get(int *indices)
  {
    return this->getRaw();
  }

  virtual void set(int *indices, const void* value)
  {
    return this->setRaw(value);
  }

  virtual void* getRaw() = 0;
  virtual void setRaw(const void* value) = 0;
  virtual GContainer* clone() = 0;

protected:
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
