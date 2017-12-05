# Overview
This repository contains all the code needed to complete the final project for the Localization course in Udacity's Self-Driving Car Nanodegree. I implemented a particle filter by following the instruction provided during the course. My implementation meet the requirements since the output says "Success! Your particle filter passed!" then it means I’ve met the requirements.

## Project Introduction
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

## Success Criteria
My implementation passes the current grading code in the simulator. The things the grading code is looking for are:

1. **Accuracy**: my particle filter localizes vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`.

2. **Performance**: my particle filter completes execution within the time of 100 seconds.

## Running the Code
I used docker setup the environment. You can run ./run_term2_docker.sh to use the doker environment. Once run_term2_docker.sh is executed, the main program can be built and ran by doing the following from the /src directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

# Implementing the Particle Filter
I followed the instructions provided in the course to implement the filter.

## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory. 

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.

## Discussion

1. My algorithm follows the general processing flow as taught in the lessons and I was able to get the required performance. I encountered 2 issues during the implementation.
    * The first issue was regarding the the situations when yaw_rat was 0. I addressed the issue in the vehicle model that I use in prediction phase.
    * The second issue was in resampling function. A bug in my implementation prevented sampling from the best particles. After addressing the issue, the algorithm woks as expected.





