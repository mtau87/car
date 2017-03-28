/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
	 
#include "cartographer_android/ros_log_sink.h"

#include <chrono>
#include <cstring>
#include <string>
#include <thread>
#include <sstream>
#include "glog/log_severity.h"

namespace cartographer_android {

namespace {

const char* GetBasename(const char* filepath) {
  const char* base = std::strrchr(filepath, '/');
  return base ? (base + 1) : filepath;
}

}  // namespace

ScopedRosLogSink::ScopedRosLogSink() : will_die_(false) { AddLogSink(this); }
ScopedRosLogSink::~ScopedRosLogSink() { RemoveLogSink(this); }

void ScopedRosLogSink::send(const ::google::LogSeverity severity,
                            const char* const filename,
                            const char* const base_filename, const int line,
                            const struct std::tm* const tm_time,
                            const char* const message,
                            const size_t message_len) {
  std::stringstream ss;
  ss << "[" << severity << "]"
  	<< "[" << GetBasename(filename) << "]"
  	<< "[" << line << "]"
  	<< "[" << tm_time << "]"
  	<< "[" << message << "]";
  const std::string message_string = ss.str();
  switch (severity) {
    case GLOG_INFO:
      std::cout << message_string;
      break;

    case GLOG_WARNING:
		std::cout << message_string;
      break;

    case GLOG_ERROR:
		std::cout << message_string;
      break;

    case GLOG_FATAL:
		std::cout << message_string;
      will_die_ = true;
      break;
  }
}

void ScopedRosLogSink::WaitTillSent() {
  if (will_die_) {
    // Give ROS some time to actually publish our message.
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

}  // namespace cartographer_android
