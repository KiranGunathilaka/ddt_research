#include <vector>
#include <random>
#include <functional>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "control_cpp/cone_colors.hpp"

#define NUM_PARTICLES 100
#define SIGMA 0.20

class Cone {
  public:
  // function prototypes
  Cone (float xt, float yt, int colour);
  void createParticleSet();
  void perceptionUpdate(float xt, float yt);
  void odometryUpdate(float velx, float angularz, int nanosec);

  // how many updates has the cone been off screen for
  double deadTimer;

  // co-ordinates of cone relative to the car
  float x, y;

  // represents the colour of the cone - replace with enum?
  int colour;

  // New methods for probabilistic color tracking
  int getColour() const;
  void updateColour(int colour);

  private:
  // create distributions
  std::mt19937 gen;
  std::normal_distribution<float> distribution;
  std::normal_distribution<float> broadDistribution;

  // store time between position updates
  float prevTime;

  // the particle set used to update the cone position estimate
  std::vector<float> particleSet;

  // Color observation counters for probabilistic color tracking
  int blue_observations_ = 0;
  int yellow_observations_ = 0;
  int orange_observations_ = 0;
};