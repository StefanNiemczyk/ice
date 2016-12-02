#include <gtest/gtest.h>

#include "ice/ICEngine.h"
#include "ice/information/CollectionDescription.h"
#include "ice/information/InformationSet.h"
#include "ice/information/InformationSpecification.h"
#include "ice/information/SetStore.h"
#include "ice/processing/EventHandler.h"

#include "etc/EngineStuff.cpp"

TEST(InformationSet, simpleTest)
{
  auto handler = std::make_shared<ice::EventHandler>(2, 100);
  auto spec = std::make_shared<ice::InformationSpecification>("", "type", "scope", "rep");
  std::map<std::string, int> metadatas;
  auto description = std::make_shared<ice::CollectionDescription>(spec, "name,", "provider", "source", metadatas);
  auto set = std::make_shared<ice::InformationSet<int>>(description, handler);

  set->add("test1", std::make_shared<int>(1));
  set->add("test2", std::make_shared<int>(2));
  ASSERT_EQ(2, set->getSize());

  set->add("test2", std::make_shared<int>(3));
  ASSERT_EQ(2, set->getSize());

  ASSERT_EQ(1, *set->get("test1")->getInformation());
  ASSERT_EQ(3, *set->get("test2")->getInformation());
}

TEST(InformationSet, store_register)
{
  auto eh = std::make_shared<ice::EventHandler>(1, 10);
  std::shared_ptr<ice::ICEngine> engine;
  auto factory = std::make_shared<TestFactory>(engine);
  auto store = std::make_shared<ice::SetStore>(eh, factory);

  auto spec1 = std::make_shared<ice::InformationSpecification>("entity1", "entityType1", "scope1", "representation1");
  auto spec2 = std::make_shared<ice::InformationSpecification>("entity2", "entityType2", "scope2", "representation2");


  std::map<std::string, int> metadatas;

  auto set1 = store->registerBaseSet("double", spec1, "topic1", metadatas, "provider", "source");
  auto set2 = store->registerBaseSet("int", spec2, "topic2", metadatas, "provider", "source");

  EXPECT_EQ(true, (set1 ? true : false));
  EXPECT_EQ(true, (set2 ? true : false));

  auto set11 = store->getSet<double>(spec1.get(), "provider", "source");
  auto set12 = store->getSet<int>(spec2.get(), "provider", "source");

  EXPECT_EQ(true, (set11 ? true : false));
  EXPECT_EQ(true, (set12 ? true : false));
}


TEST(InformationSet, async_event)
{
  auto handler = std::make_shared<ice::EventHandler>(2, 100);
  auto spec = std::make_shared<ice::InformationSpecification>("", "type", "scope", "rep");
  std::map<std::string, int> metadatas;
  auto description = std::make_shared<ice::CollectionDescription>(spec, "name,", "provider", "source", metadatas);
  auto set = std::make_shared<ice::InformationSet<int>>(description, handler);

  std::shared_ptr<SimpleListener> listener = std::make_shared<SimpleListener>();
  std::shared_ptr<SimpleListener2> listener2 = std::make_shared<SimpleListener2>();
  listener->value = 0;

  set->registerListenerAsync(listener);
  set->registerListenerAsync(listener2);

  int value = 5;

  std::unique_ptr<int> ptr(new int(value));
  set->add("muh", std::move(ptr));

  std::this_thread::sleep_for(std::chrono::milliseconds {100});

  EXPECT_EQ(value, listener->value);
  EXPECT_EQ(value + 3, listener2->value);
  EXPECT_NE(value + 4, listener2->value);
}

TEST(InformationSet, sync_event)
{
  auto handler = std::make_shared<ice::EventHandler>(2, 100);
  auto spec = std::make_shared<ice::InformationSpecification>("", "type", "scope", "rep");
  std::map<std::string, int> metadatas;
  auto description = std::make_shared<ice::CollectionDescription>(spec, "name,", "provider", "source", metadatas);
  auto set = std::make_shared<ice::InformationSet<int>>(description, handler);

  std::shared_ptr<SimpleListener> listener = std::make_shared<SimpleListener>();
  std::shared_ptr<SimpleListener2> listener2 = std::make_shared<SimpleListener2>();
  listener->value = 0;

  set->registerListenerSync(listener);
  set->registerListenerSync(listener2);

  int value = 5;

  std::unique_ptr<int> ptr(new int(value));
  set->add("muh", std::move(ptr));

  EXPECT_EQ(value, listener->value);
  EXPECT_EQ(value + 3, listener2->value);
  EXPECT_NE(value + 4, listener2->value);
}

TEST(InformationSet, sync_async_event)
{
  auto handler = std::make_shared<ice::EventHandler>(2, 100);
  auto spec = std::make_shared<ice::InformationSpecification>("", "type", "scope", "rep");
  std::map<std::string, int> metadatas;
  auto description = std::make_shared<ice::CollectionDescription>(spec, "name,", "provider", "source", metadatas);
  auto set = std::make_shared<ice::InformationSet<int>>(description, handler);

  std::shared_ptr<SimpleListener> listener = std::make_shared<SimpleListener>();
  std::shared_ptr<SimpleListener2> listener2 = std::make_shared<SimpleListener2>();
  listener->value = 0;

  set->registerListenerSync(listener);
  set->registerListenerAsync(listener2);

  int value = 5;

  std::unique_ptr<int> ptr(new int(value));
  set->add("muh", std::move(ptr));

  std::this_thread::sleep_for(std::chrono::milliseconds {100});

  EXPECT_EQ(value, listener->value);
  EXPECT_EQ(value + 3, listener2->value);
  EXPECT_NE(value + 4, listener2->value);
}

TEST(InformationSet, sync_async_task)
{
  auto handler = std::make_shared<ice::EventHandler>(2, 100);
  auto spec = std::make_shared<ice::InformationSpecification>("", "type", "scope", "rep");
  std::map<std::string, int> metadatas;
  auto description = std::make_shared<ice::CollectionDescription>(spec, "name,", "provider", "source", metadatas);
  auto set = std::make_shared<ice::InformationSet<int>>(description, handler);

  auto task1 = std::make_shared<SimpleTask>();
  auto task2 = std::make_shared<SimpleTask>();

  int rt1 = set->registerTaskAsync(task1);
  int rt2 = set->registerTaskSync(task2);

  EXPECT_EQ(0, rt1);
  EXPECT_EQ(0, rt2);

  std::unique_ptr<int> ptr(new int(5));
  set->add("muh", std::move(ptr));

  std::this_thread::sleep_for(std::chrono::milliseconds {100});

  EXPECT_TRUE(task1->value);
  EXPECT_TRUE(task2->value);

  rt1 = set->registerTaskAsync(task1);
  rt2 = set->registerTaskSync(task2);

  EXPECT_EQ(1, rt1);
  EXPECT_EQ(1, rt2);

  rt1 = set->unregisterTaskAsync(task1);
  rt2 = set->unregisterTaskSync(task2);

  EXPECT_EQ(0, rt1);
  EXPECT_EQ(0, rt2);

  rt1 = set->unregisterTaskAsync(task1);
  rt2 = set->unregisterTaskSync(task2);

  EXPECT_EQ(1, rt1);
  EXPECT_EQ(1, rt2);
}

TEST(InformationSet, filtered_list)
{
  auto handler = std::make_shared<ice::EventHandler>(2, 100);
  auto spec = std::make_shared<ice::InformationSpecification>("", "type", "scope", "rep");
  std::map<std::string, int> metadatas;
  auto description = std::make_shared<ice::CollectionDescription>(spec, "name,", "provider", "source", metadatas);
  auto set = std::make_shared<ice::InformationSet<int>>(description, handler);
  auto vec = std::make_shared<std::vector<std::shared_ptr<ice::InformationElement<int>>> >();

  for (int i = 0; i < 1000; ++i)
  {
    std::unique_ptr<int> ptr(new int(i));
    set->add(std::to_string(i), std::move(ptr));
  }

  int count = set->getFilteredList(vec, [] (std::shared_ptr<ice::InformationElement<int>> &element)
  { return true;});
  EXPECT_EQ(1000, count);

  vec->clear();

  int num = 500;

  count = set->getFilteredList(vec, [&] (std::shared_ptr<ice::InformationElement<int>> element)
  { return *element->getInformation() < num;});
  EXPECT_EQ(500, count);
}
