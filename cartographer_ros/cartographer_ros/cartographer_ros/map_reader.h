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

#ifndef CARTOGRAPHER_ROS_MAP_READER_H_
#define CARTOGRAPHER_ROS_MAP_READER_H_

#include <string>
#include <iostream>
#include <memory>

#include "yaml-cpp/yaml.h"

#include "cartographer/mapping_2d/probability_grid.h"

namespace cartographer_ros {
struct Vec3{
 float x;
 float y;
 float z;
};

struct GridConfig{
	std::string image;
	float resolution;
	Vec3 origin;
	float occupied_thresh;
	float free_thresh;
	int negate;
};

void ReadPgmAndYamlToProbabilityGrid(std::shared_ptr<cartographer::mapping_2d::ProbabilityGrid>& probability_grid, const std::string& stem);
}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_MAP_READER_H_
