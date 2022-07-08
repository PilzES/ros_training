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
#include <thread>
#include <chrono>

#include <gmock/gmock.h>

#include <pilz_testutils/async_test.h>

namespace pilz_testutils
{
void asyncCall(const std::function<void()>& fun)
{
  std::thread t(fun);
  t.detach();
}

class AsyncTestTest : public testing::Test, public testing::AsyncTest
{
public:
  MOCK_METHOD0(myMethod, void());
};

TEST_F(AsyncTestTest, defaultBehaviour)
{
  EXPECT_CALL(*this, myMethod()).Times(1).WillOnce(ACTION_OPEN_BARRIER_VOID("myMethod"));
  const int timeout_ms{ 100 };
  asyncCall(std::bind(&AsyncTestTest::myMethod, this));
  BARRIER("myMethod", timeout_ms) << "Timed-out waiting for myMethod call.";
}

TEST_F(AsyncTestTest, handlingOfUnexpectedEvent)
{
  EXPECT_CALL(*this, myMethod())
      .Times(1)
      .WillOnce(::testing::DoAll(ACTION_OPEN_BARRIER_VOID("unexpected_event"), ACTION_OPEN_BARRIER_VOID("myMethod")));
  const int timeout_ms{ 100 };
  asyncCall(std::bind(&AsyncTestTest::myMethod, this));
  BARRIER("myMethod", timeout_ms) << "Timed-out waiting for myMethod call.";
}

TEST_F(AsyncTestTest, negativeTimeout)
{
  EXPECT_CALL(*this, myMethod()).Times(1).WillOnce(ACTION_OPEN_BARRIER_VOID("myMethod"));
  const int timeout_ms{ -1 };
  asyncCall(std::bind(&AsyncTestTest::myMethod, this));
  BARRIER("myMethod", timeout_ms) << "Timed-out waiting for myMethod call.";
}

TEST_F(AsyncTestTest, openBeforeBarrier)
{
  this->triggerClearEvent("clear_event");
  const int timeout_ms{ 100 };
  BARRIER("clear_event", timeout_ms) << "Timed-out waiting for clear_event.";
}

TEST_F(AsyncTestTest, triggerTimeout)
{
  const int timeout_ms{ 100 };
  EXPECT_FALSE(barricade("myMethod", timeout_ms));
}

}  // namespace pilz_testutils

int main(int argc, char* argv[])
{
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
