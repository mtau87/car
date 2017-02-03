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

#include "cartographer/io/ply_writing_points_processor.h"

#include <iomanip>
#include <sstream>
#include <string>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/io/points_batch.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {

namespace {

// Writes the PLY header claiming 'num_points' will follow it into
// 'output_file'.
void WriteBinaryPlyHeader(const bool has_color, const int64 num_points,
                          FileWriter* const file_writer) {
  string color_header = !has_color ? "" : "property uchar red\n"
                                          "property uchar green\n"
                                          "property uchar blue\n";
  std::ostringstream stream;
  stream << "ply\n"
         << "format binary_little_endian 1.0\n"
         << "comment generated by Cartographer\n"
         << "element vertex " << std::setw(15) << std::setfill('0')
         << num_points << "\n"
         << "property float x\n"
         << "property float y\n"
         << "property float z\n"
         << color_header << "end_header\n";
  const string out = stream.str();
  CHECK(file_writer->WriteHeader(out.data(), out.size()));
}

void WriteBinaryPlyPointCoordinate(const Eigen::Vector3f& point,
                                   FileWriter* const file_writer) {
  char buffer[12];
  memcpy(buffer, &point[0], sizeof(float));
  memcpy(buffer + 4, &point[1], sizeof(float));
  memcpy(buffer + 8, &point[2], sizeof(float));
  CHECK(file_writer->Write(buffer, 12));
}

void WriteBinaryPlyPointColor(const Color& color,
                              FileWriter* const file_writer) {
  CHECK(file_writer->Write(reinterpret_cast<const char*>(color.data()),
                           color.size()));
}

}  // namespace

std::unique_ptr<PlyWritingPointsProcessor>
PlyWritingPointsProcessor::FromDictionary(
    FileWriterFactory file_writer_factory,
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  return common::make_unique<PlyWritingPointsProcessor>(
      file_writer_factory(dictionary->GetString("filename")), next);
}

PlyWritingPointsProcessor::PlyWritingPointsProcessor(
    std::unique_ptr<FileWriter> file_writer, PointsProcessor* const next)
    : next_(next),
      num_points_(0),
      has_colors_(false),
      file_(std::move(file_writer)) {}

PointsProcessor::FlushResult PlyWritingPointsProcessor::Flush() {
  WriteBinaryPlyHeader(has_colors_, num_points_, file_.get());
  CHECK(file_->Close()) << "Closing PLY file_writer failed.";

  switch (next_->Flush()) {
    case FlushResult::kFinished:
      return FlushResult::kFinished;

    case FlushResult::kRestartStream:
      LOG(FATAL) << "PLY generation must be configured to occur after any "
                    "stages that require multiple passes.";
  }
  LOG(FATAL);
}

void PlyWritingPointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {
  if (batch->points.empty()) {
    next_->Process(std::move(batch));
    return;
  }

  if (num_points_ == 0) {
    has_colors_ = !batch->colors.empty();
    WriteBinaryPlyHeader(has_colors_, 0, file_.get());
  }
  if (has_colors_) {
    CHECK_EQ(batch->points.size(), batch->colors.size())
        << "First PointsBatch had colors, but encountered one without. "
           "frame_id: "
        << batch->frame_id;
  }

  for (size_t i = 0; i < batch->points.size(); ++i) {
    WriteBinaryPlyPointCoordinate(batch->points[i], file_.get());
    if (has_colors_) {
      WriteBinaryPlyPointColor(batch->colors[i], file_.get());
    }
    ++num_points_;
  }
  next_->Process(std::move(batch));
}

}  // namespace io
}  // namespace cartographer
