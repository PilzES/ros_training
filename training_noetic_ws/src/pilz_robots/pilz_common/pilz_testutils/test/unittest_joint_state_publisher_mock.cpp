/*
 * Copyright (c) 2021 Pilz GmbH & Co. KG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <future>
#include <chrono>

#include <gmock/gmock.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <pilz_testutils/joint_state_publisher_mock.h>
#include <pilz_testutils/async_test.h>
#include <pilz_utils/wait_for_message.h>

namespace pilz_testutils
{
class JointStateSubscriberMock
{
public:
  JointStateSubscriberMock()
  {
    sub_ = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 1, &JointStateSubscriberMock::cb_mock, this);
  }

  MOCK_METHOD1(cb_mock, void(const sensor_msgs::JointStateConstPtr&));

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
};

using testing::PrintToString;

static constexpr double COMPUTATION_TOLERANCE{ 0.1 };

MATCHER_P(PositionNear, p, "Position " + std::string(negation ? "is not" : "is") + " near " + PrintToString(p) + ".")
{
  return std::abs(arg->position[0] - p) < COMPUTATION_TOLERANCE;
}

class JointStatePublisherMockTest : public testing::Test, public testing::AsyncTest
{
};

TEST_F(JointStatePublisherMockTest, defaultBehaviour)
{
  JointStatePublisherMock publisher_mock;
  publisher_mock.startPublishingAsync();

  auto res = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", ros::Duration(1 /*sec*/));

  EXPECT_NE(res, nullptr) << "Timeout while waiting for expected message.";

  publisher_mock.stopPublishing();
}

TEST_F(JointStatePublisherMockTest, setVelocity)
{
  JointStatePublisherMock publisher_mock;
  JointStateSubscriberMock subscriber_mock;

  ON_CALL(subscriber_mock, cb_mock(PositionNear(3.0))).WillByDefault(ACTION_OPEN_BARRIER_VOID("position_done"));

  publisher_mock.startPublishingAsync();
  publisher_mock.setJoint1Velocity(3.0);

  BARRIER("position_done", 3000 /*timeout ms*/);

  publisher_mock.stopPublishing();
}

TEST_F(JointStatePublisherMockTest, goHome)
{
  JointStatePublisherMock publisher_mock;
  JointStateSubscriberMock subscriber_mock;

  ON_CALL(subscriber_mock, cb_mock(PositionNear(0.0))).WillByDefault(ACTION_OPEN_BARRIER_VOID("home_position_done"));

  publisher_mock.startPublishingAsync(2.0 /*start_position_joint1 */);
  publisher_mock.setJoint1Velocity(3);
  publisher_mock.goHome();

  BARRIER("home_position_done", 3000 /*timeout ms*/);

  publisher_mock.stopPublishing();
}

/* This tests is merly done as a preparation for a possible refactoring
   of the JointStatePublisherMock. As shown the JointStatePublisherMock has
   implemented a rather suprising behaviour concerning its limit.
   If the joint is at or above the limit set in the JointStatePublisherMock the
   Velocity is reverted and thus the joint bounces and heads into the oposite direction.
   This seems unexpected and poses some questions regarding initialization outside the
   limit where this gets the joint "stuck" since the velocity will flip on each cycle. */
TEST_F(JointStatePublisherMockTest, limitBounce)
{
  JointStatePublisherMock publisher_mock;
  testing::NiceMock<JointStateSubscriberMock> subscriber_mock;

  ON_CALL(subscriber_mock, cb_mock(PositionNear(3.0))).WillByDefault(ACTION_OPEN_BARRIER_VOID("upper_limit_done"));
  ON_CALL(subscriber_mock, cb_mock(PositionNear(-3.0))).WillByDefault(ACTION_OPEN_BARRIER_VOID("lower_limit_done"));

  publisher_mock.startPublishingAsync(0.0 /*start_position_joint1 */);  // start outside limit
  publisher_mock.setJoint1Velocity(3);

  BARRIER("upper_limit_done", 5000 /*timeout ms*/);
  BARRIER("lower_limit_done", 10000 /*timeout ms*/);

  publisher_mock.stopPublishing();
}

TEST_F(JointStatePublisherMockTest, nextMessageOnStoppedRobot)
{
  JointStatePublisherMock publisher_mock;

  publisher_mock.startPublishingAsync(0.4 /*start_position_joint1 */);  // start outside limit
  auto res = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", ros::Duration(1 /*sec*/));

  ASSERT_NE(res, nullptr) << "Timeout while waiting for expected message.";

  auto next_message = publisher_mock.getNextMessage();

  EXPECT_DOUBLE_EQ(next_message->position[0], 0.4);

  publisher_mock.stopPublishing();
}

}  // namespace pilz_testutils

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "unittest_joint_state_publisher_mock");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner{ 1 };
  spinner.start();

  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
