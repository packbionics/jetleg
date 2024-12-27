// Copyright 2024 Pack Bionics
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.


#include <gtest/gtest.h>

#include <service/finite_state_controller_service.hpp>

static int ARGC;
static char ** ARGV;

static void setup()
{
  // Create a ROS 2 node
  // rclcpp::init(argc, argv);
  // rclcpp::NodeOptions node_options;
  // node_options.automatically_declare_parameters_from_overrides(true);
}

static void tearDown()
{
  // rclcpp::shutdown();
}

TEST(finite_state_controller_service, test_constructor)
{
  setup();

  // Scenario:
  EXPECT_EQ(1, 1);

  tearDown();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  ARGC = argc;
  ARGV = argv;

  return RUN_ALL_TESTS();
}
