/*
 * test_rhizome.cpp
 *
 *  Created on: Apr 8, 2016
 *      Author: sni
 */

#include <string>
#include <limits.h>
#include <unistd.h>

#include "gtest/gtest.h"
#include "serval_interface.h"


TEST(JNITest, get_bundle_list)
{
  ice::serval_interface si("localhost", 4110, "peter", "venkman");

  auto bundles = si.getBundleList();

  ASSERT_TRUE(bundles != nullptr);

  for (auto &bundle : * bundles)
  {
    std::cout << bundle.toString() << std::endl;
  }
}

TEST(JNITest, add_bundle)
{
  char buff[ PATH_MAX ];
  ssize_t count = readlink( "/proc/self/exe", buff, PATH_MAX );
  std::string path( buff, (count > 0) ? count : 0 );

  ice::serval_interface si("localhost", 4110, "peter", "venkman");

  auto result = si.addBundle("/home/sni/pjx/catkin_ws/src/ice/serval_wrapper/tests/data/test.txt");
  ASSERT_TRUE(result);

  auto bundles = si.getBundleList();

  ASSERT_TRUE(bundles != nullptr);
  ASSERT_EQ(bundles->size(), 1);

  for (auto &bundle : * bundles)
  {
    std::cout << bundle.toString() << std::endl;
  }
}
