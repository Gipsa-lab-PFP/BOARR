/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
 * Copyright 2018 Thibaut Tezenas, GIPSA-LAB Grenoble, France
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "gazebo_wind_plugin.h"

#include <fstream>
#include <math.h>

namespace gazebo {

BOARRGazeboWindPlugin::~BOARRGazeboWindPlugin() {
  event::Events::DisconnectWorldUpdateBegin(update_connection_);
}

void BOARRGazeboWindPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  if (kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  // Store the pointer to the model.
  model_ = _model;
  world_ = model_->GetWorld();

  double wind_gust_cycle_duration = kDefaultWindGustCycleDuration;
  double wind_gust_duration = kDefaultWindGustDuration;

  //==============================================//
  //========== READ IN PARAMS FROM SDF ===========//
  //==============================================//

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_wind_plugin] Please specify a robotNamespace.\n";

  // Create Gazebo Node.
  node_handle_ = gazebo::transport::NodePtr(new transport::Node());

  // Initialise with default namespace (typically /gazebo/default/).
  node_handle_->Init();

  if (_sdf->HasElement("xyzOffset"))
    xyz_offset_ = _sdf->GetElement("xyzOffset")->Get<math::Vector3>();
  else
    gzerr << "[gazebo_wind_plugin] Please specify a xyzOffset.\n";

  getSdfParam<std::string>(_sdf, "linkName", link_name_, link_name_);
  
  // Get the wind params from SDF.
	getSdfParam<math::Vector3>(_sdf, "windDirection", wind_direction_,
                      wind_direction_);
    getSdfParam<double>(_sdf, "windForceMean", wind_force_mean_,
                        wind_force_mean_);
    getSdfParam<double>(_sdf, "windForceVariance", wind_force_variance_,
                        wind_force_variance_);
    // Get the wind gust params from SDF.
    getSdfParam<double>(_sdf, "windGustDurationCycle", wind_gust_cycle_duration, wind_gust_cycle_duration);
    getSdfParam<double>(_sdf, "windGustDuration", wind_gust_duration,
                        wind_gust_duration);
    getSdfParam<double>(_sdf, "windGustForceMean", wind_gust_force_mean_,
                        wind_gust_force_mean_);
    getSdfParam<double>(_sdf, "windGustForceVariance", wind_gust_force_variance_,
                        wind_gust_force_variance_);
    getSdfParam<math::Vector3>(_sdf, "windGustDirection", wind_gust_direction_,
                        wind_gust_direction_);

    wind_direction_.Normalize();
    wind_gust_direction_.Normalize();
    wind_gust_cycle_duration_ = wind_gust_cycle_duration;
    wind_gust_gust_duration_ = wind_gust_duration;
  
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_wind_plugin] Couldn't find specified link \"" << link_name_
                                                                   << "\".");

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&BOARRGazeboWindPlugin::OnUpdate, this, _1));
}

double BOARRGazeboWindPlugin::RandNormal(double mean, double stddev)//Box muller method
{
  static double n2 = 0.0;
  static int n2_cached = 0;
  if (!n2_cached)
  {
    double x, y, r;
    do
    {
      x = 2.0*rand()/RAND_MAX - 1;
      y = 2.0*rand()/RAND_MAX - 1;

      r = x*x + y*y;
    }
    while (r == 0.0 || r > 1.0);
    {
      double d = sqrt(-2.0*log(r)/r);
      double n1 = x*d;
      n2 = y*d;
      double result = n1*stddev + mean;
      n2_cached = 1;
      return result;
    }
  }
  else
  {
    n2_cached = 0;
    return n2*stddev + mean;
  }
}

// This gets called by the world update start event.
void BOARRGazeboWindPlugin::OnUpdate(const common::UpdateInfo& _info) {
  if (kPrintOnUpdates) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  // Get the current simulation time.
  double now = world_->GetSimTime().Double();
  
  // Calculate the wind force.
  double wind_strength = wind_force_mean_ + RandNormal(0,std::sqrt(wind_force_variance_));//XXX use std::normal_distribution instead, see http://www.cplusplus.com/reference/random/normal_distribution/
  math::Vector3 wind = wind_strength * wind_direction_;
  // Apply a force from the constant wind to the link.
  link_->AddForceAtRelativePosition(wind, xyz_offset_);

  math::Vector3 wind_gust(0.0, 0.0, 0.0);
  // Calculate the wind gust force.
  if ( std::fmod(now,wind_gust_cycle_duration_) >= wind_gust_cycle_duration_ - wind_gust_gust_duration_ ) {
    double wind_gust_strength = wind_gust_force_mean_ + RandNormal(0,std::sqrt(wind_gust_force_variance_)) ;//XXX use std::normal_distribution instead, see http://www.cplusplus.com/reference/random/normal_distribution/
    wind_gust = wind_gust_strength * wind_gust_direction_;
    // Apply a force from the wind gust to the link.
    link_->AddForceAtRelativePosition(wind_gust, xyz_offset_);
  }
}

// register the plugin
GZ_REGISTER_MODEL_PLUGIN(BOARRGazeboWindPlugin)
} // namespace gazebo
