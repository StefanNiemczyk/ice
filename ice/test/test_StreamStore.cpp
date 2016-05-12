#include <iostream>
#include <memory>
#include <string>
#include <time.h>
#include <vector>

#include "ros/ros.h"

#include "ice/information/AbstractInformationListener.h"
#include "ice/information/InformationElement.h"
#include "ice/information/InformationSpecification.h"
#include "ice/information/InformationStream.h"
#include "ice/information/StreamStore.h"
#include "ice/information/InformationType.h"
#include "ice/information/StreamDescription.h"
#include "ice/processing/EventHandler.h"
#include "ice/TypeDefs.h"

#include "gtest/gtest.h"

namespace
{

class SimpleTask : public ice::AsynchronousTask
{
public:
  SimpleTask()
  {
    value = false;
  }

  int performTask()
  {
    value = true;

    return 0;
  }

  bool value;
};

class SimpleListener : public ice::AbstractInformationListener<int>
{
public:
  SimpleListener()
  {
    value = 0;
  }

  ~SimpleListener()
  {

  }

  int value;

  const int newEvent(std::shared_ptr<ice::InformationElement<int>> element,
                     std::shared_ptr<ice::InformationStream<int>> stream)
  {
    value = *element->getInformation();

    return 0;
  }
};

class SimpleListener2 : public ice::AbstractInformationListener<int>
{
public:
  SimpleListener2()
  {
    value = 0;
  }

  ~SimpleListener2()
  {

  }

  int value;

  const int newEvent(std::shared_ptr<ice::InformationElement<int> > element,
                     std::shared_ptr<ice::InformationStream<int> > stream)
  {
    value = *element->getInformation() + 3;

    return 0;
  }
};

class StreamStoreTests : public ::testing::Test
{

protected:
  std::shared_ptr<ice::StreamStore> store;
//  std::shared_ptr<ice::InformationType> type;
  std::shared_ptr<ice::InformationStream<double> > stream1;
  std::shared_ptr<ice::InformationStream<int> > stream2;
  std::shared_ptr<ice::InformationSpecification> spec1;
  std::shared_ptr<ice::InformationSpecification> spec2;
  time_t start_time_;
  static std::string TOPIC_1;
  static std::string TOPIC_2;
  int* value;

  virtual void SetUp()
  {
    start_time_ = time(NULL);
    auto eh = std::make_shared<ice::EventHandler>(1, 10);
    store = std::make_shared<ice::StreamStore>(eh, nullptr, nullptr);

   // auto uuid = boost::uuids::random_generator()();
    //ont::entity entity, ont::entityType, ont::scope scope, ont::representation representation
    spec1 = std::make_shared<ice::InformationSpecification>("entity1", "entityType1", "scope1", "representation1");
    spec2 = std::make_shared<ice::InformationSpecification>("entity2", "entityType2", "scope2", "representation2");

    std::map<std::string, int> metadatas;

    stream1 = store->registerStream<double>(spec1, TOPIC_1, 1000, metadatas, "provider", "source");
    stream2 = store->registerStream<int>(spec2, TOPIC_2, 1000, metadatas, "provider", "source");
  }

  virtual void TearDown()
  {
    const time_t end_time = time(NULL);

    EXPECT_TRUE(end_time - start_time_ <= 5) << "The test took too long.";
  }
};

std::string StreamStoreTests::TOPIC_1 = "test_topic_1";
std::string StreamStoreTests::TOPIC_2 = "test_topic_2";

TEST_F(StreamStoreTests, create_store)
{
  EXPECT_TRUE((store ? true : false));
}

TEST_F(StreamStoreTests, register_streams)
{
  EXPECT_EQ(true, (stream1 ? true : false));
  EXPECT_EQ(true, (stream2 ? true : false));
}

TEST_F(StreamStoreTests, get_buffer)
{
  std::shared_ptr<ice::InformationStream<double> > stream1;
  std::shared_ptr<ice::InformationStream<int> > stream2;

  stream1 = store->getStream<double>(spec1.get(), "provider", "source");
  stream2 = store->getStream<int>(spec2.get(), "provider", "source");

  EXPECT_EQ(true, (stream1 ? true : false));
  EXPECT_EQ(true, (stream2 ? true : false));
}

TEST_F(StreamStoreTests, add_information)
{
  for (int i = 0; i < 7; ++i)
  {
    std::unique_ptr<int> ptr(new int(i));
    int identifier = stream2->add(std::move(ptr));
    EXPECT_EQ(i, identifier);
  }
}

TEST_F(StreamStoreTests, read_information)
{
  for (int i = 0; i < stream2->getStreamSize() + 5; ++i)
  {
    std::unique_ptr<int> ptr(new int(i));
    int identifier = stream2->add(std::move(ptr));
    EXPECT_EQ(i, identifier);
  }

  int i = 0;
  std::shared_ptr<ice::InformationElement<int> > information;

  do
  {
    information = stream2->getLast(i);

    if (information)
      EXPECT_EQ((stream2->getStreamSize() - 1 + 5 - i), *information->getInformation());

    ++i;
  } while (information);
}

TEST_F(StreamStoreTests, async_event)
{
  std::shared_ptr<ice::InformationStream<int> > stream;

  stream = stream2;

  std::shared_ptr<SimpleListener> listener = std::make_shared<SimpleListener>();
  std::shared_ptr<SimpleListener2> listener2 = std::make_shared<SimpleListener2>();
  listener->value = 0;

  stream->registerListenerAsync(listener);
  stream->registerListenerAsync(listener2);

  int value = 5;

  std::unique_ptr<int> ptr(new int(value));
  stream->add(std::move(ptr));

  std::this_thread::sleep_for(std::chrono::milliseconds {100});

  EXPECT_EQ(value, listener->value);
  EXPECT_EQ(value + 3, listener2->value);
  EXPECT_NE(value + 4, listener2->value);
}

TEST_F(StreamStoreTests, sync_event)
{
  std::shared_ptr<ice::InformationStream<int> > stream;

  stream = stream2;

  std::shared_ptr<SimpleListener> listener = std::make_shared<SimpleListener>();
  std::shared_ptr<SimpleListener2> listener2 = std::make_shared<SimpleListener2>();
  listener->value = 0;

  stream->registerListenerSync(listener);
  stream->registerListenerSync(listener2);

  int value = 5;

  std::unique_ptr<int> ptr(new int(value));
  stream->add(std::move(ptr));

  EXPECT_EQ(value, listener->value);
  EXPECT_EQ(value + 3, listener2->value);
  EXPECT_NE(value + 4, listener2->value);
}

TEST_F(StreamStoreTests, sync_async_event)
{
  std::shared_ptr<ice::InformationStream<int> > stream;

  stream = stream2;

  std::shared_ptr<SimpleListener> listener = std::make_shared<SimpleListener>();
  std::shared_ptr<SimpleListener2> listener2 = std::make_shared<SimpleListener2>();
  listener->value = 0;

  stream->registerListenerSync(listener);
  stream->registerListenerAsync(listener2);

  int value = 5;

  std::unique_ptr<int> ptr(new int(value));
  stream->add(std::move(ptr));

  std::this_thread::sleep_for(std::chrono::milliseconds {100});

  EXPECT_EQ(value, listener->value);
  EXPECT_EQ(value + 3, listener2->value);
  EXPECT_NE(value + 4, listener2->value);
}

TEST_F(StreamStoreTests, sync_async_task)
{
  std::shared_ptr<ice::InformationStream<int> > stream;

  stream = stream2;

  auto task1 = std::make_shared<SimpleTask>();
  auto task2 = std::make_shared<SimpleTask>();

  int rt1 = stream->registerTaskAsync(task1);
  int rt2 = stream->registerTaskSync(task2);

  EXPECT_EQ(0, rt1);
  EXPECT_EQ(0, rt2);

  std::unique_ptr<int> ptr(new int(5));
  stream->add(std::move(ptr));

  std::this_thread::sleep_for(std::chrono::milliseconds {100});

  EXPECT_TRUE(task1->value);
  EXPECT_TRUE(task2->value);

  rt1 = stream->registerTaskAsync(task1);
  rt2 = stream->registerTaskSync(task2);

  EXPECT_EQ(1, rt1);
  EXPECT_EQ(1, rt2);

  rt1 = stream->unregisterTaskAsync(task1);
  rt2 = stream->unregisterTaskSync(task2);

  EXPECT_EQ(0, rt1);
  EXPECT_EQ(0, rt2);

  rt1 = stream->unregisterTaskAsync(task1);
  rt2 = stream->unregisterTaskSync(task2);

  EXPECT_EQ(1, rt1);
  EXPECT_EQ(1, rt2);
}

TEST_F(StreamStoreTests, filtered_list)
{
  auto stream = stream2;
  auto vec = std::make_shared<std::vector<std::shared_ptr<ice::InformationElement<int>>> >();

  for (int i = 0; i < 1000; ++i)
  {
    std::unique_ptr<int> ptr(new int(i));
    stream->add(std::move(ptr));
  }

  int count = stream->getFilteredList(vec, [] (std::shared_ptr<ice::InformationElement<int>> element)
  { return true;});
  EXPECT_EQ(1000, count);

  vec->clear();

  int num = 500;

  count = stream->getFilteredList(vec, [&] (std::shared_ptr<ice::InformationElement<int>> element)
  { return *element->getInformation() < num;});
  EXPECT_EQ(500, count);
}

}
