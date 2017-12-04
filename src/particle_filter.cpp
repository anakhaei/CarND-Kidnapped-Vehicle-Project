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
#include <vector>
#include <algorithm>
#include <float.h>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[])
{
    // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
    //   x, y, theta and their uncertainties from GPS) and all weights to 1.
    // Add random Gaussian noise to each particle.
    // NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    num_particles = 100;
    default_random_engine gen;

    // This line creates a normal (Gaussian) distribution.
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);

    for (unsigned int i = 0; i < num_particles; i++)
    {

        Particle p;
        p.id = i;
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
        p.weight = 1;
        particles.push_back(p);
        weights.push_back(1);
    }
    normalizeWeights();
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate)
{
    // TODO: Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/

    default_random_engine gen;

    for (unsigned int i = 0; i < num_particles; i++)
    {
        double x0 = particles[i].x;
        double y0 = particles[i].y;
        double theta0 = particles[i].theta;

        double xf = 0;
        double yf = 0;
        double thetaf = theta0 + yaw_rate * delta_t;

        if (yaw_rate != 0)
        {
            xf = x0 + velocity / yaw_rate * (sin(theta0 + yaw_rate * delta_t) - sin(theta0));
            yf = y0 + velocity / yaw_rate * (cos(theta0) - cos(theta0 + yaw_rate * delta_t));
        }
        else
        {
            xf = x0 + velocity * delta_t * sin(theta0);
            yf = y0 + velocity * delta_t * cos(theta0);
        }

        // This line creates a normal (Gaussian) distribution.
        normal_distribution<double> dist_x(xf, std_pos[0]);
        normal_distribution<double> dist_y(yf, std_pos[1]);
        normal_distribution<double> dist_theta(thetaf, std_pos[2]);

        particles[i].x = dist_x(gen);
        particles[i].y = dist_y(gen);
        particles[i].theta = dist_theta(gen);
    }
}

/**
	 * dataAssociation Finds which observations correspond to which landmarks (likely by using
	 *   a nearest-neighbors data association).
	 * @param predicted Vector of predicted landmark observations
	 * @param observations Vector of landmark observations
	 */
// void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs> &observations)
// {
//     // TODO: Find the predicted measurement that is closest to each observed measurement and assign the
//     //   observed measurement to this particular landmark.
//     // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
//     //   implement this method and use it as a helper during the updateWeights phase.
// }

int ParticleFilter::findNearestLandmark(double x, double y, const Map &map_landmarks)
{
    // TODO: Find the predicted measurement that is closest to each observed measurement and assign the
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
    //   implement this method and use it as a helper during the updateWeights phase.

    int nearest_it = -1;
    double nearest_dist = 1000;
    for (unsigned int k = 0; k < map_landmarks.landmark_list.size(); k++)
    {
        double mu_x = map_landmarks.landmark_list[k].x_f;
        double mu_y = map_landmarks.landmark_list[k].y_f;
        double distance = dist(x, y, mu_x, mu_y);
        if (distance < nearest_dist)
        {
            nearest_dist = distance;
            nearest_it = k;
        }
    }
    return nearest_it;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const std::vector<LandmarkObs> &observations, const Map &map_landmarks)
{
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
    //cout << "--------- updateWeights ------------" << endl;
    double sig_x = std_landmark[0];
    double sig_y = std_landmark[1];
    double gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

    for (unsigned int i = 0; i < num_particles; i++)
    {
        double w = 1;
        double px = particles[i].x;
        double py = particles[i].y;
        double c = cos(particles[i].theta);
        double s = sin(particles[i].theta);

        for (unsigned int j = 0; j < observations.size(); j++)
        {
            LandmarkObs obs = observations[j];
            double x_g = c * obs.x - s * obs.y + px;
            double y_g = s * obs.x + c * obs.y + py;
            //double dist = sensor_range;
            int k = findNearestLandmark(x_g, y_g, map_landmarks);

            double mu_x = map_landmarks.landmark_list[k].x_f;
            double mu_y = map_landmarks.landmark_list[k].y_f;
            double exponent = ((x_g - mu_x) * (x_g - mu_x)) / (2 * sig_x * sig_x) + ((y_g - mu_y) * (y_g - mu_y)) / (2 * sig_y * sig_y);
            double distance = dist(x_g, y_g, mu_x, mu_y);
            w = w * gauss_norm * exp(-exponent);
        }
        particles[i].weight = w;
    }
    normalizeWeights();
}

void ParticleFilter::normalizeWeights()
{
    double sum = 0;
    for (unsigned int i = 0; i < num_particles; i++)
    {
        sum += particles[i].weight;
    }
    for (unsigned int i = 0; i < num_particles; i++)
    {
        particles[i].weight /= sum;
        weights[i] = particles[i].weight;
    }
}

void ParticleFilter::resample()
{
    // TODO: Resample particles with replacement with probability proportional to their weight.
    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    //cout << "before resample" << endl;
    //printWeights();

    std::vector<Particle> new_particles;
    int index = rand() % num_particles;
    double beta = 0.0;
    auto it = std::max_element(std::begin(weights), std::end(weights));

    double max_w = *it;

    cout << "max_weights before = " << max_w << endl;

    for (unsigned int i = 0; i < num_particles; i++)
    {
        double f = (double)rand() / RAND_MAX * 2 * max_w;
        beta += f;
        while (beta > weights[index])
        {
            beta -= weights[index];
            index = (index + 1) % num_particles;
        }
        //cout << "selected index = " << index << endl;
        new_particles.push_back(particles[index]);
    }
    particles.clear();
    for (unsigned int i = 0; i < num_particles; i++)
    {
        particles.push_back(new_particles[i]);
    }

    normalizeWeights();
}

Particle ParticleFilter::SetAssociations(Particle &particle, const std::vector<int> &associations,
                                         const std::vector<double> &sense_x, const std::vector<double> &sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
    vector<int> v = best.associations;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1); // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
    vector<double> v = best.sense_x;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1); // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
    vector<double> v = best.sense_y;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1); // get rid of the trailing space
    return s;
}

void ParticleFilter::printStatus()
{

    for (unsigned int i = 0; i < num_particles; i++)
    {
        Particle p = particles[i];
        cout << "i:" << i << ", ID:" << p.id << ", x:" << p.x << ", y: " << p.y << ", theta: " << p.theta << ", w: " << p.weight << ") ," << endl;
    }
    cout << endl;
}

void ParticleFilter::printWeights()
{
    cout << "-------------------------------------printWeights " << endl;
    for (unsigned int i = 0; i < num_particles; i++)
    {
        Particle p = particles[i];
        cout << "i:" << i << ", ID:" << p.id << ", w: " << p.weight << endl;
    }
    cout << endl;
}
