# Overview
This repository contains all the code needed to complete the final project for the Localization course in Udacity's Self-Driving Car Nanodegree. I implemented a particle filter by following the instructions provided during the course. My implementation meets the Success Criteria since the output says "Success! Your particle filter passed!".

## Project Introduction
The robot in the project has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

## Success Criteria
My implementation passes the current grading code in the simulator. The things the grading code is looking for are:

1. **Accuracy**: my particle filter localizes vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`.

2. **Performance**: my particle filter completes execution within the time of 100 seconds.

## Running the Code
I used docker to setup the required environment. Once run_term2_docker.sh is executed, the main program can be built and ran by doing the following from the /src directory:

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

# Implementing the Particle Filter
I followed the instructions provided in the course to implement the filter.

## Inputs to the Particle Filter
Here are the inputs to the particle filter:
1. a map of the location,
2. a (noisy) GPS estimate of its initial location,
3. lots of (noisy) sensor and control data.

## Discussion

My implementation follows the general processing flow as taught in the lessons and I was able to meet the required performance. I encountered 2 issues during the implementation of the filter:
    * The first issue was regarding the the situations when yaw_rat was 0. I addressed the issue in the vehicle model that I use in prediction phase.
    * The second issue was in resampling function. A bug in my implementation prevented sampling from the best particles. After addressing the issue, the algorithm woks as expected.
Once the issues were addressed, my implementation met the Success Criteria!




