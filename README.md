# Overview
---
This repository contains all the code needed to complete the final project for the Localization course in Udacity's Self-Driving Car Nanodegree.

## Project Introduction
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project implements a 2 dimensional particle filter in C++.\
To the particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step the filter will also get observation and control data.

## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

The protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

    // sense noisy position data from the simulator
      ["sense_x"]
      ["sense_y"]
      ["sense_theta"]
    // get the previous velocity and yaw rate to predict the particle's transitioned state
      ["previous_velocity"]
      ["previous_yawrate"]
    // receive noisy observation data from the simulator, in a respective list of x/y values
      ["sense_observations_x"]
      ["sense_observations_y"]

OUTPUT: values provided by the c++ program to the simulator

    // best particle values used for calculating the error evaluation
      ["best_particle_x"]
      ["best_particle_y"]
      ["best_particle_theta"]

[//]: # (Image References)

[image1]: media/algo.png "algorithm"
[image2]: media/null_yaw_rate.png "null_yaw_rate"
[image3]: media/not_null_yaw_rate.png "not_null_yaw_rate"
[image4]: media/multivariate_gaussian.PNG "multivariate_gaussian"
[image5]: media/weight.png "weight"
[image6]: media/np_10.png "np_10"
---
# Particle Filter
The structure of the particle filter used in this project is showed below:
![alt text][image1]
*From Udacity Localization Lesson.*\
The particle filter definition is contained in the file [particle_filter.h](./src/particle_filter.h) while its implementation is contained in the file [particle_filter.cpp](./src/particle_filter.cpp).

As showed in the previous image, the algorithm starts with an `Initialization` step and then runs over a loop composed by three main steps, `Prediction`, `Update` and `Resample`.

## Initialization
In this step, the number of filter's particles and all the particles position and weight will be initialized(lines 26-49).\
The particle position initial guess is provided by a GPS measurement. In order to consider the noisy nature of the measurement, a Gaussian distribution generator has been created using the GPS measure and its standard deviation.
It follows that the all the particles have been initialized sampling from that generator.\
The weight of all particles has been initialized to 1.

## Prediction
The prediction step is used to update the particles' state according to three measurements:
1. time elapsed from the previous step;
2. vehicle velocity;
3. yaw rate;

The model used for the computation is the bicycle one since it represents a good approximation of the vehicle motion(lines 77-86).\
It is necessary to distinguish between two conditions:
1. ***null yaw rate***\
![alt text][image2]

2. ***not null yaw rate***\
![alt text][image3]

In order to take into account the measurement noise, a Gaussian distribution with 0 mean has been used to add the noise to the computation results(lines 89-94).

## Update
In this step, the weight of each particles is updated according to the Multivariate-Gaussian probability density.\
Its formulation can be seen below:\
![alt text][image4]

In order to apply this formula, some preprocessing are required.\
For each particle, the following steps are performed:
1. extract from the map all the landmarks within the sensor range with center in the particle reference frame (lines 140-149);
2. rototranslate all the observations in the particle reference frame (lines 153-163);
3. associate a landmark to each observation using the nearest neighbor algorithm (function call at line 166; function code at lines 104-128);
4. compute the weight of the particle (lines 169-177).

As introduced before, the particle weight is computed according the Multivariate-Gaussian probability. To improve the readability of the code, the function `multiv_prob(...)` has been defined and implemented in the file [helper_functions.h](./src/helper_functions.h) (lines 251-267).\
The final weight is given by:\
![alt text][image5]\
where `i` represents the pair (observation, landmark) obtained by the nearest neighbor algorithm.

Last operation performed in this function is the normalization of the weights vector (lines 186-190). This operation is a requirement for the **Resample** step.

## Resamlpe
In this step, a new particles set is computed performing a discrete distribution resampling according to the normalized particles weights (lines 195-206).

# Results
The success of the particle filter is evaluated by the grading code in the simulator.\
The things the grading code is looking for are:
1. **Accuracy**: the particle filter should localize vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`.
2. **Performance**: the particle filter should complete execution within the time of 100 seconds.

If the above requirement are satisfied, then the simulator window will show the message

    "Success! Your particle filter passed!"
Here an example of the simulation result:\
![alt text][image6]\

To evaluate the filter goodness, different test have been performed changing the number of filter's particles. The following table contains the test results:

|    Number of particles    |    x error [m]    |    y error [m]    |    yaw error [rad]   |    execution time [ms]   |
|:-------------------------:|:-----------------:|:-----------------:|:--------------------:|:------------------------:|
|    10                     |    0.175          |    0.151          |    0.006             |    20                    |
|    50                     |    0.117          |    0.112          |    0.004             |    20                    |
|    100                    |    0.114          |    0.106          |    0.004             |    20                    |
|    200                    |    0.111          |    0.107          |    0.004             |    20.3                  |
|    500                    |    0.104          |    0.099          |    0.003             |    21.9                  |
|    1000                   |    0.107          |    0.099          |    0.004             |    34.4                  |
|    2000                   |    0.109          |    0.098          |    0.003             |    47                    |

From the above results it is possible to affirm that:
* increase the particles number beyond hundreds, doesn't bring big improvements in accuracy and highly affects the computation time;
* with only 10 particles, the filter can localize the vehicle with a good accuracy.

Since the autonomous driving requires a localization with an accuracy of about 10cm and a real time computation, it is possible to affirm that a filter with 200 particle is good enough to this purpose.\
[Here](media/np_200.mp4) a video of the performances of a filter with 200 particles.
