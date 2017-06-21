/*
 * particle_filter.cpp
 *
 *  Created on: June 21, 2017
 *  Author: Junsheng Fu
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

static int NUM_PARTICLES = 50;

/* Set the number of particles.
 * Initialize all particles to first position (based on estimates of
 * x, y, theta and their uncertainties from GPS) and all weights to 1.
 * Add random Gaussian noise to each particle.
 */
void ParticleFilter::init(double x, double y, double theta, double std[]) {

  // create normal distributions for x, y, and theta
  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);
  std::default_random_engine gen;

  // resize the vectors of particles and weights
  num_particles = NUM_PARTICLES;
  particles.resize(num_particles);
  //weights.resize(num_particles);

  // generate the particles
  for(auto& p: particles){
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1;
  }

  is_initialized = true;

}

/* Add measurements to each particle and add random Gaussian noise.
 */
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

  std::default_random_engine gen;

  // generate random Gaussian noise
  std::normal_distribution<double> N_x(0, std_pos[0]);
  std::normal_distribution<double> N_y(0, std_pos[1]);
  std::normal_distribution<double> N_theta(0, std_pos[2]);

  for(auto& p: particles){

    // add measurements to each particle
    if( fabs(yaw_rate) < 0.0001){  // constant velocity
      p.x += velocity * delta_t * cos(p.theta);
      p.y += velocity * delta_t * sin(p.theta);

    } else{
      p.x += velocity / yaw_rate * ( sin( p.theta + yaw_rate*delta_t ) - sin(p.theta) );
      p.y += velocity / yaw_rate * ( cos( p.theta ) - cos( p.theta + yaw_rate*delta_t ) );
      p.theta += yaw_rate * delta_t;
    }

    // predicted particles with added sensor noise
    p.x += N_x(gen);
    p.y += N_y(gen);
    p.theta += N_theta(gen);
  }

}

/* Find the predicted measurement that is closest to each observed measurement and assign the
 * observed measurement to this particular landmark, with exhausted search (may be replaced with KD-tree approaches)
 */
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {

  for(auto& obs: observations){
    double minD = std::numeric_limits<float>::max();

    for(const auto& pred: predicted){
      double distance = dist(obs.x, obs.y, pred.x, pred.y);
      if( minD > distance){
        minD = distance;
        obs.id = pred.id;
      }
    }
  }
}

/**
* updateWeights Updates the weights for each particle based on the likelihood of the
  observed measurements.

* Steps:
* 1: collect the landmarks within the sensor range for each particle, as predictions
* 2: convert the observations from vehicle coordinate to map coordinate,
*    by a rigid transform (rotation & translation), see equation 3.3 from http://planning.cs.uiuc.edu/node99.html
* 3: Use dataAssociation(predictions, observations) to find the landmark index for each observation
* 4: Update the weights of each particle using a multi-variate Gaussian distribution,
*    https://en.wikipedia.org/wiki/Multivariate_normal_distribution
*/
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		std::vector<LandmarkObs> observations, Map map_landmarks) {

  for(auto& p: particles){
    p.weight = 1.0;

    // step 1: collect valid landmarks
    vector<LandmarkObs> predictions;
    for(const auto& lm: map_landmarks.landmark_list){
      double distance = dist(p.x, p.y, lm.x_f, lm.y_f);
      if( distance < sensor_range){ // if the landmark is within the sensor range, save it to predictions
        predictions.push_back(LandmarkObs{lm.id_i, lm.x_f, lm.y_f});
      }
    }

    // step 2: convert observations coordinates from vehicle to map
    vector<LandmarkObs> observations_map;
    double cos_theta = cos(p.theta);
    double sin_theta = sin(p.theta);

    for(const auto& obs: observations){
      LandmarkObs tmp;
      tmp.x = obs.x * cos_theta - obs.y * sin_theta + p.x;
      tmp.y = obs.x * sin_theta + obs.y * cos_theta + p.y;
      //tmp.id = obs.id; // maybe an unnecessary step, since the each obersation will get the id from dataAssociation step.
      observations_map.push_back(tmp);
    }

    // step 3: find landmark index for each observation
    dataAssociation(predictions, observations_map);

    // step 4: compute the particle's weight:
    // see equation this link:
    for(const auto& obs_m: observations_map){

      Map::single_landmark_s landmark = map_landmarks.landmark_list.at(obs_m.id-1);
      double x_term = pow(obs_m.x - landmark.x_f, 2) / (2 * pow(std_landmark[0], 2));
      double y_term = pow(obs_m.y - landmark.y_f, 2) / (2 * pow(std_landmark[1], 2));
      double w = exp(-(x_term + y_term)) / (2 * M_PI * std_landmark[0] * std_landmark[1]);
      p.weight *=  w;
    }

    weights.push_back(p.weight);

  }

}

/* Resample particles with replacement with probability proportional to their weight.
 * Reference 1: std::discrete_distribution, see the link below with an example
 * http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
 */
void ParticleFilter::resample() {

  // generate distribution according to weights
  std::random_device rd;
  std::mt19937 gen(rd());
  std::discrete_distribution<> dist(weights.begin(), weights.end());

  // create resampled particles
  vector<Particle> resampled_particles;
  resampled_particles.resize(num_particles);

  // resample the particles according to weights
  for(int i=0; i<num_particles; i++){
    int idx = dist(gen);
    resampled_particles[i] = particles[idx];
  }

  // assign the resampled_particles to the previous particles
  particles = resampled_particles;

  // clear the weight vector for the next round
  weights.clear();
}

/* particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
 * associations: The landmark id that goes along with each listed association
 * sense_x: the associations x mapping already converted to world coordinates
 * sense_y: the associations y mapping already converted to world coordinates
 */
Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
  stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
