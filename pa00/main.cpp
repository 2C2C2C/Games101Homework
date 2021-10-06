#define _USE_MATH_DEFINES

#include <math.h>
#include <eigen3/Eigen/Dense>
#include <iostream>

void Example()
{
	// Basic Example of cpp
	std::cout << "Hiko homework 0 \n";

	std::cout << "Example of cpp \n";
	float a = 1.0, b = 2.0;
	std::cout << a << std::endl;
	std::cout << a / b << std::endl;
	std::cout << std::sqrt(b) << std::endl;
	std::cout << std::acos(-1) << std::endl;
	std::cout << std::sin(30.0 / 180.0 * acos(-1)) << std::endl;

	// Example of vector
	std::cout << "Example of vector \n";
	// vector definition
	Eigen::Vector3f v(1.0f, 2.0f, 3.0f);
	Eigen::Vector3f w(1.0f, 0.0f, 0.0f);
	// vector output
	std::cout << "Example of output \n";
	std::cout << v << std::endl;
	// vector add
	std::cout << "Example of add \n";
	std::cout << v + w << std::endl;
	// vector scalar multiply
	std::cout << "Example of scalar multiply \n";
	std::cout << v * 3.0f << std::endl;
	std::cout << 2.0f * v << std::endl;

	// Example of matrix
	std::cout << "Example of matrix \n";
	// matrix definition
	Eigen::Matrix3f i, j;
	i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
	j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
	// matrix output
	std::cout << "Example of output \n";
	std::cout << i << std::endl;
	// matrix add i + j
	// matrix scalar multiply i * 2.0
	// matrix multiply i * j
	// matrix multiply vector i * v
}

float AngelToRad(float angle)
{
	return angle * M_PI / 180.0;
}

// anti clockwise rotate
void Pa0Exec(float rotateRad, Eigen::Vector2f move)
{
	// Eigen::MatrixXf ogPointMatrix(3, 1);
	// ogPointMatrix << 2.0, 1.0, 0.0;
	Eigen::Vector3f ogPointMatrix;
	ogPointMatrix << 2.0, 1.0, 1.0;
	std::cout << ogPointMatrix << std::endl;
	std::cout << std::endl;

	// anticlock wise rotate
	// [ cosθ, -sinθ, 0
	// sinθ, cosθ, 0
	// 0, 0, 1 ]

	Eigen::Matrix3f rotMatrix;
	rotMatrix << cos(rotateRad), -sin(rotateRad), 0, sin(rotateRad), cos(rotateRad), 0, 0, 0, 1;

	Eigen::Matrix3f moveMatrix;
	moveMatrix << 1.0, 0.0, move(0), 0.0, 1.0, move(1), 0.0, 0.0, 1.0;

	Eigen::Matrix3Xf result;
	result = rotMatrix * ogPointMatrix;
	std::cout << "after rotate: " << std::endl;
	std::cout << result << std::endl;

	result = moveMatrix * result;
	std::cout << "after move: " << std::endl;

	std::cout << result << std::endl;
}

int main()
{
	// Example();
	float rotateAngle = 45.0;
	Eigen::Vector2f extraMove(1.0, 2.0);
	Pa0Exec(AngelToRad(rotateAngle), extraMove);
	return 0;
}