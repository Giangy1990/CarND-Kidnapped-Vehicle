/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include <limits>       // std::numeric_limits
#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  //Set the number of particles
  num_particles = 200;

  // Set standard deviations for x, y, and theta
  double std_x = std[0];
  double std_y = std[1];
  double std_theta = std[2];
  
  // Create normal distributions for x, y and theta
  std::normal_distribution<double> dist_x(x, std_x);
  std::normal_distribution<double> dist_y(y, std_y);
  std::normal_distribution<double> dist_theta(theta, std_theta);
  
  // init all particles
  std::default_random_engine gen;
  Particle curr_particle;
  while (particles.size() < num_particles){
    curr_particle.x = dist_x(gen);
    curr_particle.y = dist_y(gen);
    curr_particle.theta = dist_theta(gen);
    curr_particle.weight = 1;
    particles.push_back(curr_particle);
    weights.push_back(1);
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  // normal distribution generator
  std::default_random_engine gen;
  
  // temporary variable for the motion model computation
  bool complex_model = false;
  double k;
  if (fabs(yaw_rate) > std::numeric_limits<double>::epsilon()){
    k = velocity/yaw_rate;
    complex_model = true;
  }
  else{
    k = velocity*delta_t;
  }
  
  // Set standard deviations for x, y, and theta
  double std_x = std_pos[0];
  double std_y = std_pos[1];
  double std_theta = std_pos[2];
  
  for (int particle_idx = 0; particle_idx < num_particles; ++particle_idx){
    Particle curr_particle = particles[particle_idx];

    // compute update using model
    if (complex_model){
      float phi = curr_particle.theta + yaw_rate*delta_t;
      curr_particle.x += k*(sin(phi) - sin(curr_particle.theta));
      curr_particle.y += k*(cos(curr_particle.theta) - cos(phi));
      curr_particle.theta = phi;
    }
    else{
      curr_particle.x += k*cos(curr_particle.theta);
      curr_particle.y += k*sin(curr_particle.theta);
    }
    
    // add noise
    std::normal_distribution<double> dist_x(0, std_x);
    std::normal_distribution<double> dist_y(0, std_y);
    std::normal_distribution<double> dist_theta(0, std_theta);
    curr_particle.x += dist_x(gen);
    curr_particle.y += dist_y(gen);
    curr_particle.theta += dist_theta(gen);
    
    // store results
    particles[particle_idx] = curr_particle;
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  // If no predicted landmark is present, no association is made
  if (predicted.size() == 0) {
    return;
  }
  
  // association
  LandmarkObs curr_onservation, curr_predicted;
  for (int observations_idx = 0; observations_idx < observations.size(); ++observations_idx){
    // init variables for nearest search
    double min_dist = std::numeric_limits<double>::max();
    int associated_id = -1;
    curr_onservation = observations[observations_idx];
    
    // nearest search
    for (int predicted_idx = 0; predicted_idx < predicted.size(); ++predicted_idx){
      curr_predicted = predicted[predicted_idx];
      double curr_dist = dist(curr_onservation.x, curr_onservation.y, curr_predicted.x, curr_predicted.y);
      if (curr_dist < min_dist){
        // update near predicted
        min_dist = curr_dist;
        associated_id = curr_predicted.id;
      }
    }
    // store nearest predicted id
    observations[observations_idx].id = associated_id;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  double std_x = std_landmark[0];
  double std_y = std_landmark[1];
  for (int particle_idx = 0; particle_idx < num_particles; ++particle_idx){
    Particle curr_particle = particles[particle_idx];
    
    // find the landmaks into the sensor range
    vector<LandmarkObs> visible_landmarks;
    LandmarkObs curr_landmark;
    for (int landmark_idx = 0; landmark_idx < map_landmarks.landmark_list.size(); ++landmark_idx){
      curr_landmark.x = map_landmarks.landmark_list[landmark_idx].x_f;
      curr_landmark.y = map_landmarks.landmark_list[landmark_idx].y_f;
      curr_landmark.id = map_landmarks.landmark_list[landmark_idx].id_i;
      if (dist(curr_landmark.x, curr_landmark.y, curr_particle.x, curr_particle.y) <= sensor_range){
        visible_landmarks.push_back(curr_landmark);
      }
    }
    // std::cout << "curr landmark count = " << visible_landmarks.size() << std::endl;
    
    // rototranslate observation in the current particle ref frame
    float cos_th0 = cos(curr_particle.theta);
    float sin_th0 = sin(curr_particle.theta);
    vector<LandmarkObs> rt_observations;
    for (int observations_idx = 0; observations_idx < observations.size(); ++observations_idx){
      LandmarkObs curr_observation = observations[observations_idx];
      LandmarkObs new_onservation;
      // coordinate transformation
      new_onservation.x = curr_particle.x + curr_observation.x*cos_th0 - curr_observation.y*sin_th0;
      new_onservation.y = curr_particle.y + curr_observation.x*sin_th0 + curr_observation.y*cos_th0;
      rt_observations.push_back(new_onservation);
    }
    
    // compute associations
    dataAssociation(visible_landmarks, rt_observations);
    
    // compute weight
    double w = 1;
    for (int landmark_idx = 0; landmark_idx < visible_landmarks.size(); ++landmark_idx){
      LandmarkObs curr_landmark = visible_landmarks[landmark_idx];
      for (int observations_idx = 0; observations_idx < rt_observations.size(); ++observations_idx){
        LandmarkObs curr_observation = rt_observations[observations_idx];
        if (curr_observation.id == curr_landmark.id){
          w *= multiv_prob(std_x, std_y, curr_landmark.x, curr_landmark.y, curr_observation.x, curr_observation.y);
        }
      }
    }
    
    // set particle weight
    particles[particle_idx].weight = w;
    weights[particle_idx] = w;
  }
  
  // normalize weights
  double sum_of_elems = std::accumulate(weights.begin(), weights.end(), decltype(weights)::value_type(0));
  for (int particle_idx = 0; particle_idx < num_particles; ++particle_idx){
    particles[particle_idx].weight /= sum_of_elems;
    weights[particle_idx] /= sum_of_elems;
  }
}

void ParticleFilter::resample() {
  // setup random generator for resampling
  std::default_random_engine gen;
  std::discrete_distribution<> d(weights.begin(), weights.end());
  
  // random resampling
  vector<Particle> new_particles;
  while (new_particles.size() < num_particles){
    int idx = d(gen);
    new_particles.push_back(particles[idx]);
  }
  
  // set particles
  particles = new_particles;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}