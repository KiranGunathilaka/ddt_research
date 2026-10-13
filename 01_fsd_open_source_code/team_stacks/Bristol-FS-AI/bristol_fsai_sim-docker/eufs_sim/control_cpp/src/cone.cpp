#include "control_cpp/cone.hpp"
#include "control_cpp/cone_colors.hpp"

Cone::Cone (float xt, float yt, int colour) 
  : gen(std::random_device{}()), distribution(0.0, 0.05), broadDistribution(0.0, 1.0)
{
  blue_observations_ = 0;
  yellow_observations_ = 0;
  orange_observations_ = 0;
  if (colour == BLUE) {
    blue_observations_ = 1;
  } else if (colour == YELLOW) {
    yellow_observations_ = 1;
  } else if (colour == ORANGE) {
    orange_observations_ = 1;
  }
  this->x = xt; this->y = yt;
  this->prevTime = -1.0f;
  this->colour = colour;
  this->deadTimer = 0.0;
}

int Cone::getColour() const {
  int max_obs = blue_observations_;
  int color = BLUE;
  if (yellow_observations_ > max_obs) {
    max_obs = yellow_observations_;
    color = YELLOW;
  }
  if (orange_observations_ > max_obs) {
    color = ORANGE;
  }
  return color;
}

void Cone::updateColour(int colour) {
  if (colour == BLUE) {
    blue_observations_++;
  } else if (colour == YELLOW) {
    yellow_observations_++;
  } else if (colour == ORANGE) {
    orange_observations_++;
  }
}

void Cone::createParticleSet() {
  particleSet = std::vector<float>(2 * NUM_PARTICLES);

  for (int i = 0; i < NUM_PARTICLES; i++) {
    particleSet[2 * i] = this->x + broadDistribution(gen);
    particleSet[2 * i + 1] = this->y + broadDistribution(gen);
  }
}

void Cone::odometryUpdate(float velx, float angularz, int nanosec) {
  if (prevTime < 0.0) {
    prevTime = nanosec * 1E-9;
    return;
  }

  double deltaT_pf = nanosec * 1E-9 - prevTime;

  // discard update if overflow (fix this)
  if (deltaT_pf < 0.0) {
    prevTime = nanosec * 1E-9;
    return;
  }

  // update each particle according to the motion model
  // particles represent cone position in vehicle's local frame
  for (int i = 0; i < NUM_PARTICLES; i++) {
    double x = particleSet[2 * i];
    double y = particleSet[2 * i + 1];
    
    // Transform particles based on vehicle motion
    // Vehicle moves forward by velx * deltaT and rotates by angularz * deltaT
    double theta = angularz * deltaT_pf;
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    
    // Apply rotation and translation to transform cone position
    // Cone appears to move backward relative to the moving vehicle
    double new_x = x * cos_theta + y * sin_theta - velx * deltaT_pf;
    double new_y = -x * sin_theta + y * cos_theta;
    
    // Add process noise to maintain particle diversity
    particleSet[2 * i] = new_x + distribution(gen);
    particleSet[2 * i + 1] = new_y + distribution(gen);
  }

  float avgx = 0.0, avgy = 0.0;
  for (int i = 0; i < NUM_PARTICLES; i++) {
    avgx += particleSet[2 * i];
    avgy += particleSet[2 * i + 1];
  }
  avgx /= NUM_PARTICLES;
  avgy /= NUM_PARTICLES;
  this->x = avgx;
  this->y = avgy;

  this->prevTime = nanosec * 1E-9;
}

void Cone::perceptionUpdate(float xt, float yt) {
  float weights[NUM_PARTICLES];
  float totalWeight = 0.0;
  for (int i = 0; i < NUM_PARTICLES; i++) {
    float dist = sqrt(pow(particleSet[2 * i] - xt, 2) + 
      pow(particleSet[2 * i + 1] - yt, 2));
    weights[i] = exp(-pow(dist / SIGMA, 2));
    totalWeight += weights[i];
  }

  std::uniform_real_distribution<float> weightDistribution(0.0, totalWeight);
  // generate NUM_PARTICLES random numbers in the range [0, totalWeight]
  float randomNumbers[NUM_PARTICLES];
  for (int i = 0; i < NUM_PARTICLES; i++) {
    randomNumbers[i] = weightDistribution(gen);
  }
  std::sort(std::begin(randomNumbers), std::end(randomNumbers));

  // now resample from this distribution
  std::vector<float> newParticleSet = std::vector<float>(2 * NUM_PARTICLES);
  float sum = 0.0;                            // a sum of all weights so far
  int index = 0;                              // index of the new particle set
  for (int i = 0; i < NUM_PARTICLES; i++) {
    sum += weights[i];                        // increase the sum

    while (sum >= randomNumbers[index]) {     // add samples while the value of sum is less then the random number
      if (index >= NUM_PARTICLES) break;

      // adding the randomness maintains the same number of particles
      newParticleSet[index * 2] = particleSet[2 * i] + distribution(gen);
      newParticleSet[index * 2 + 1] = particleSet[2 * i + 1] + distribution(gen);
      index++;
    }
  }

  float avgx = 0.0, avgy = 0.0;
  for (int i = 0; i < NUM_PARTICLES; i++) {
    avgx += newParticleSet[2 * i];
    avgy += newParticleSet[2 * i + 1];
  }
  avgx /= NUM_PARTICLES;
  avgy /= NUM_PARTICLES;
  this->x = avgx;
  this->y = avgy;

  this->particleSet = newParticleSet;
}