#include <math.h>

#define EIGEN_DONT_PARALLELIZE
#include <Eigen/Dense>

#include <vector>

#include "manipulator.h"
#include "manipulator\main_interface.h"

#define M_PI 3.1415926535897932384626433832795

using namespace std;
using Eigen::Vector3f;

Robot robot;

/*extern "C"*/ void manipulator_init()
{
  Vector3f baseSize(2, 1, 2);
	Vector3f zero(0, 0, 0);

	Part basePart(zero, Vector3f::UnitZ(), true);
	basePart.setRoughBounding(baseSize, zero, zero);
	basePart.addBox(baseSize, zero, zero);

	Vector3f joint1(0, 0.5f, 0);
	Part part1(joint1, Vector3f::UnitY(), true);
	part1.setParent(&basePart);
	part1.setRoughBounding(Vector3f(1, 2.6f, 1), Vector3f(0, 1.3f, 1), zero);
	part1.addCilinder(0.5f, 0.25f, Vector3f(0, 0.25f, 0), Vector3f(M_PI / 4, 0, 0));
	part1.addCilinder(0.3f, 1, Vector3f(0, 1.4f, 0), Vector3f(M_PI / 4, 0, 0));
	
	Vector3f joint2(0, 2.4f, 0);
	Part part2(joint2, Vector3f::UnitZ(), true);
	part2.setParent(&part1);
	part2.setRoughBounding(Vector3f(1, 3.5f, 1), Vector3f(0, 1.3f, 1), zero);
	part2.addCilinder(0.2f, 0.7f, Vector3f(0, 1.3f, 0), Vector3f(M_PI / 4, 0, 0));
	part2.addCilinder(0.25f, 0.3f, Vector3f(0, 0, 0), Vector3f(0, 0, 0));
	part2.addCilinder(0.25f, 0.3f, Vector3f(0, 2.6f, 0), Vector3f(0, 0, 0));
	part2.addBox(Vector3f(0.5f, 0.6f, 0.6f), Vector3f(0, 0.32f, 0), zero);
	part2.addBox(Vector3f(0.5f, 0.6f, 0.6f), Vector3f(0, 2.28f, 0), zero);

	Vector3f joint3(0, 2.6f, 0);
	Part part3(joint3, Vector3f::UnitZ(), true);
	part3.setParent(&part2);
	part3.setRoughBounding(Vector3f(1.5f, 3.2f, 1.3f), Vector3f(0, 1.15f, 1), zero);
	part3.addCilinder(0.15f, 0.1f, Vector3f(0, 2.05f, 0), Vector3f(M_PI / 4, 0, 0));
	part3.addSphere(0.26f, Vector3f(0, 2.343f, 0), Vector3f(0, 0, 0));
	part3.addBox(Vector3f(0.5f, 0.6f, 0.6f), Vector3f(0, 0.32f, 0), zero);
	part3.addBox(Vector3f(0.5f, 0.6f, 0.6f), Vector3f(0, 2.28f, 0), zero);
  
	robot.addPart(&basePart);
	robot.addPart(&part1);
	robot.addPart(&part2);
	robot.addPart(&part3);

	robot.addPairOfPartsForChecking(2, 0);
	robot.addPairOfPartsForChecking(3, 0);
	robot.addPairOfPartsForChecking(3, 1);
}

/*extern "C"*/ float get_slowdown_coefficient(const float* position, const float* speed, const int partNumber)
{
  std::vector<float> positionArray;
  for (int i = 0; i < partNumber; i++)
  {
    positionArray.push_back(position[i]);
  }
  std::vector<float> speedArray;
  for (int i = 0; i < partNumber; i++)
  {
    speedArray.push_back(speed[i]);
  }
  return robot.getSlowdownCoefficient(positionArray, speedArray);
}
