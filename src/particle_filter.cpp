/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
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

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
    //   x, y, theta and their uncertainties from GPS) and all weights to 1.
    // Add random Gaussian noise to each particle.
    // NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    numParticles = 50;
    double weight = 1. / numParticles;
    default_random_engine gen;

    normal_distribution<double> distX(x, std[0]);
    normal_distribution<double> distY(y, std[1]);
    normal_distribution<double> distTheta(theta, std[2]);

    double sampleX, sampleY, sampleTheta;
    for (int i = 0; i < numParticles; ++i) {
        Particle particle;
        sampleX = distX(gen);
        sampleY = distY(gen);
        sampleTheta = distTheta(gen);
        while (sampleTheta > M_PI) sampleTheta -= 2. * M_PI;
        while (sampleTheta < -M_PI) sampleTheta += 2. * M_PI;

        particle.id = i;
        particle.x = sampleX;
        particle.y = sampleY;
        particle.theta = sampleTheta;
        particle.weight = weight;
        particles.push_back(particle);
        weights.push_back(weight);
    }
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    // TODO: Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/
    //extract values for better readability

    double x, y, yaw;
    default_random_engine gen;
    //add noise
    normal_distribution<double> distX(0.0, std_pos[0]);
    normal_distribution<double> distY(0.0, std_pos[1]);
    normal_distribution<double> distTheta(0.0, std_pos[2]);
    for (int i = 0; i < numParticles; ++i) {
        x = particles[i].x;
        y = particles[i].y;
        yaw = particles[i].theta;
        //avoid division by zero
        if (fabs(yaw_rate) > 0.001) {
            x += velocity / yaw_rate * (sin(yaw + yaw_rate * delta_t) - sin(yaw)) + distX(gen);
            y += velocity / yaw_rate * (cos(yaw) - cos(yaw + yaw_rate * delta_t)) + distY(gen);
        } else {
            x += velocity * delta_t * cos(yaw) + distX(gen);
            y += velocity * delta_t * sin(yaw) + distY(gen);
        }

        yaw += yaw_rate * delta_t + distTheta(gen);

        //write predicted position to each particle
        particles[i].x = x;
        particles[i].y = y;
        particles[i].theta = yaw;
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs> &observations) {
    // TODO: Find the predicted measurement that is closest to each observed measurement and assign the
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
    //   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   std::vector<LandmarkObs> observations, Map map_landmarks) {
    // TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
    //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
    // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
    //   according to the MAP'S coordinate system. You will need to transform between the two systems.
    //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
    //   The following is a good resource for the theory:
    //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
    //   and the following is a good resource for the actual equation to implement (look at equation
    //   3.33
    //   http://planning.cs.uiuc.edu/node99.html


}

void ParticleFilter::resample() {
    // TODO: Resample particles with replacement with probability proportional to their weight.
    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x,
                                         std::vector<double> sense_y) {
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    //Clear the previous associations

}

string ParticleFilter::getAssociations(Particle best) {
    vector<int> v = best.associations;

    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1); //Remove trailing spaces
    return s;
}

string ParticleFilter::getSenseX(Particle best) {
    vector<double> v = best.sense_x;

    makeString(v);
}

string ParticleFilter::getSenseY(Particle best) {
    vector<double> v = best.sense_y;

    return makeString(v);
}

string ParticleFilter::makeString(vector<double, allocator<double>> &v) const {
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  //Remove trailing spaces
    return s;
}
