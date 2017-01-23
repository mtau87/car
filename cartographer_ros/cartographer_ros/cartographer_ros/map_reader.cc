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

#include "cartographer_ros/map_reader.h"

#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>
#include "glog/logging.h"
#include <iomanip> 
namespace cartographer_ros {

namespace {
void operator>>(const YAML::Node& node, Vec3& v) {  
   v.x = node[0].as<float>();  
   v.y = node[1].as<float>();  
   v.z = node[2].as<float>();  
}  

void operator>>(const YAML::Node& node, GridConfig& grid) { 
   grid.image = node["image"].as<std::string>();
   grid.resolution = node["resolution"].as<float>();  
   node["origin"] >> grid.origin;  
   grid.occupied_thresh = node["occupied_thresh"].as<float>();  
   grid.free_thresh = node["free_thresh"].as<float>();  
   grid.negate = node["negate"].as<int>();  
}  

}  // namespace

void ReadPgmAndYamlToProbabilityGrid(std::shared_ptr<cartographer::mapping_2d::ProbabilityGrid>& probability_grid, const std::string& stem) {
  const std::string yaml_filename = stem + ".yaml";
  GridConfig grid;

  std::vector<YAML::Node> vNode = YAML::LoadAllFromFile(yaml_filename);

  for(YAML::Node& doc : vNode) {
    doc >> grid;
  }

  const std::string pgm_filename = grid.image;
  LOG(INFO) << "read map from '" << pgm_filename << "'...";
  std::ifstream pgm_file(pgm_filename, std::ios::binary);
  std::string s("");
  int line_number = 1;
  int width = 0;
  int height = 0;
  double resolution = 0.f;
  std::vector<double> vGrid;

  while (getline( pgm_file, s ) )
  {
    if(s.empty())
    {
      continue;
    }
    switch(line_number)
    {
      case 1:
      break;
      case 2:
      {
        std::stringstream ss(s);
        ss.ignore(256, ';');
        ss >> resolution;
      }
      break;
      case 3:
      {
        std::stringstream ss(s);
        ss >> width >> height;
      }
      break;
      case 4:
      break;
      default:
      break;
    }
    line_number++;
    if (line_number > 4)
    {
      break;
    }
  }
  int pos = pgm_file.tellg();
  pgm_file.seekg(0, pgm_file.end);
  int length = (int)(pgm_file.tellg()) - pos;
  pgm_file.seekg(pos);
  char * buffer = new char [length];
  pgm_file.read (buffer,length);
  for (int i = 0; i < length; ++i)
  {
    unsigned short  d = (unsigned short)(unsigned char)buffer[i];
    int occupancy_probability = -1;
    double probability = 0;
    if (d != 128)
    {
      occupancy_probability = 100-100.f/255*d;
      probability = occupancy_probability*(cartographer::mapping::kMaxProbability - cartographer::mapping::kMinProbability)/100.f + cartographer::mapping::kMinProbability;
    }
    vGrid.push_back(probability);
  }

  pgm_file.close();


  LOG(INFO) << width << std::endl;
  LOG(INFO) << height << std::endl;
  LOG(INFO) << resolution << std::endl;
  LOG(INFO) << grid.origin.x << std::endl;
  LOG(INFO) << grid.origin.y << std::endl;
  LOG(INFO) << grid.origin.x+resolution*width << std::endl;
  LOG(INFO) << grid.origin.y+resolution*height << std::endl;
  cartographer::mapping_2d::MapLimits lim(resolution, Eigen::Vector2d(grid.origin.x+resolution*width, grid.origin.y+resolution*height), cartographer::mapping_2d::CellLimits(height, width));
  probability_grid = std::make_shared<cartographer::mapping_2d::ProbabilityGrid>(lim);
  LOG(INFO) << "probability_grid_.limits().resolution() " << probability_grid->limits().resolution() << std::endl;
  LOG(INFO) << "probability_grid_.limits().cell_limits().num_x_cells " << probability_grid->limits().cell_limits().num_x_cells << std::endl;
  LOG(INFO) << "probability_grid_.limits().cell_limits().num_y_cells " << probability_grid->limits().cell_limits().num_y_cells << std::endl;
  LOG(INFO) << "probability_grid_.limits().max().x() " << probability_grid->limits().max().x() << std::endl;
  LOG(INFO) << "probability_grid_.limits().max().y() " << probability_grid->limits().max().y() << std::endl;
  LOG(INFO) << "(int)vGrid.size() " << (int)vGrid.size() << std::endl;

  for (int i = 0; i < (int)vGrid.size(); ++i)
  {
    int x = width - (i % width + 1);
    //int y = height - ( 1 + i/width);
    //int x = i % width;
    int y = i/width;
    probability_grid->StartUpdate();
    probability_grid->SetProbability(Eigen::Array2i(y, x), vGrid[i]);   
  }
  probability_grid->StartUpdate();
}
}  // namespace cartographer_ros
