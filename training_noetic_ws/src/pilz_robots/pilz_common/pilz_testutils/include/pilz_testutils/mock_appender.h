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

#ifndef MOCK_APPENDER_H
#define MOCK_APPENDER_H

#include <gmock/gmock.h>

#include "ros/console_impl.h"
#include <ros/console.h>

#include "log4cxx/appenderskeleton.h"
#include "log4cxx/level.h"

namespace pilz_testutils
{
/**
 * @brief Mocked version of a log4cxx::Appender to be used by gtest
 *
 * @related LoggerMock
 *
 * @note Not intended for direct use. Please use LoggerMock inside the tests.
 *
 */
class MockAppender : public log4cxx::AppenderSkeleton
{
public:
  ~MockAppender()
  {
    close();
  }

public:
  // Indirection via "internal_append" needed to avoid "override" warning
  // message from compiler
  void append(const log4cxx::spi::LoggingEventPtr& a, log4cxx::helpers::Pool& b) override
  {
    internal_append(a, b);
  }

  MOCK_METHOD2(internal_append, void(const log4cxx::spi::LoggingEventPtr&, log4cxx::helpers::Pool&));

  log4cxx::LogString getName() const override
  {
    return "MockAppender";
  }

  void close() override
  {
  }

  // LCOV_EXCL_START required by Base class
  bool requiresLayout() const override
  {
    return false;
  }
  // LCOV_EXCL_STOP
};

#define GENERATE_LOGMESSAGE_MATCHER_P(level)                                                                           \
  MATCHER_P(Is##level, msg, std::string(#level " \"") + msg + "\"")                                                    \
  {                                                                                                                    \
    return arg->getLevel()->toInt() == log4cxx::Level::level##_INT && std::string(msg) == arg->getMessage();           \
  }

GENERATE_LOGMESSAGE_MATCHER_P(DEBUG)
GENERATE_LOGMESSAGE_MATCHER_P(INFO)
GENERATE_LOGMESSAGE_MATCHER_P(WARN)
GENERATE_LOGMESSAGE_MATCHER_P(ERROR)
GENERATE_LOGMESSAGE_MATCHER_P(FATAL)

#define EXPECT_LOG(logger, level, msg)                                                                                 \
  EXPECT_EQ((logger).getName(), "MockAppender");                                                                       \
  EXPECT_CALL(logger, internal_append(Is##level(msg), ::testing::_))

}  // namespace pilz_testutils

namespace log4cxx
{
namespace spi
{
// LCOV_EXCL_START
void PrintTo(const LoggingEventPtr& logging_event, std::ostream* os)
{
  *os << logging_event->getLevel()->toString() << " \"" << logging_event->getMessage() << "\"";
}
// LCOV_EXCL_STOP
}  // namespace spi
}  // namespace log4cxx

#endif  // MOCK_APPENDER_H
