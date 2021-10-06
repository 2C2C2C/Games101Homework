#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

float angleToRad(float angle)
{
	return angle * (float)MY_PI / 180.0;
}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
	Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

	Eigen::Matrix4f translate;
	translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
		-eye_pos[2], 0, 0, 0, 1;

	view = translate * view;

	return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
	Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

	// TODO: Implement this function
	// Create the model matrix for rotating the triangle around the Z axis.
	// Then return it.

	/*
	// rotate around z axis
	Rz(θ)=
	[cosθ -sinθ 0 0
	 sinθ cosθ 0 0
	 0 0 1 0
	 0 0 0 1]
	*/

	float rad = angleToRad(rotation_angle);
	Eigen::Matrix4f translate;
	translate <<
		cos(rad), -sin(rad), 0, 0,
		sin(rad), cos(rad), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	model = translate * model;
	return model;
}

// is it the eye_fov is yFov? fov is in degrees
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
	float zNear, float zFar)
{
	// Students will implement this function

	Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

	// TODO: Implement this function
	// Create the projection matrix for the given parameters.
	// Then return it.

	float yTop = 0, yBottom = 0;
	float xLeft = 0, xRight = 0;

	float eye_fov_rad = angleToRad(eye_fov);

	yTop = tan(eye_fov_rad * 0.5f) * abs(zNear);
	xRight = aspect_ratio * yTop;
	xLeft = -xRight;
	yBottom = -yTop;

	Eigen::Matrix4f zTransform;
	zTransform <<
		zNear, 0, 0, 0,
		0, zNear, 0, 0,
		0, 0, zNear + zFar, 0,
		0, 0, 0, zNear *zFar;

	Eigen::Matrix4f orthoMoveTransform;
	orthoMoveTransform <<
		1, 0, 0, -(xRight + xLeft)*0.5f,
		0, 1, 0, -(yTop + yBottom)*0.5f,
		0, 0, 1, -(zNear + zFar)*0.5f,
		0, 0, 0, 1;

	Eigen::Matrix4f scaleTransform;
	scaleTransform <<
		2 / (xRight - xLeft), 0, 0, 0,
		0, 2 / (yTop - yBottom), 0, 0,
		0, 0, 2 / (zNear - zFar), 0,
		0, 0, 0, 1;

	// !!! move first, second scale, then squash it :)
	// first 2 are for ortho projection
	projection = orthoMoveTransform * scaleTransform * zTransform;
	return projection;
}

/*
(1) translate space so that the rotation axis passes through the origin
(2) rotate space about the x axis so that the rotation axis lies in the xz plane
(3) rotate space about the y axis so that the rotation axis lies along the z axis
(4) perform the desired rotation by theta about the z axis
(5) apply the inverse of step (3)
(6) apply the inverse of step (2)
(7) apply the inverse of step (1)
*/
Eigen::Matrix4f get_rotation(Eigen::Vector3f axis, float angle)
{
	// TODO: Implement this function
	Eigen::Vector4f axisTemp;
	axisTemp << axis(0), axis(1), axis(2), 0;
	Eigen::Matrix4f rotTransform = Eigen::Matrix4f::Identity();

	axis.normalize();
	Eigen::Matrix4f axisRx = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f axisRx_anti = Eigen::Matrix4f::Identity();
	if ((axis(1)*axis(1) + axis(2)*axis(2)) != 0)
	{
		float tempRadA = asin(axis.dot(Eigen::Vector3f::UnitY()));
		axisRx <<
			1, 0, 0, 0,
			0, std::cos(tempRadA), -std::sin(tempRadA), 0,
			0, std::sin(tempRadA), std::cos(tempRadA), 0,
			0, 0, 0, 1;

		axisRx_anti = axisRx.transpose();
	}

	Eigen::Matrix4f axisRz = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f axisRz_anti = Eigen::Matrix4f::Identity();
	Eigen::Vector4f tempAxisOnXOZ = axisTemp.transpose() * axisRx;
	if ((axis(0)*axis(0) + axis(1)*axis(1)) != 0)
	{
		float tempAngle = 0.0f;
		float tempRadA = acos(tempAxisOnXOZ.dot(Eigen::Vector4f::UnitZ()));
		axisRz <<
			cos(tempRadA), 0, sin(tempRadA), 0,
			0, 1, 0, 0,
			-sin(tempRadA), 0, cos(tempRadA), 0,
			0, 0, 0, 1;

		axisRz_anti = axisRz.transpose();
	}

	rotTransform = axisRx * axisRz * get_model_matrix(angle) * axisRz_anti * axisRx_anti;
	return rotTransform;
}

int main(int argc, const char** argv)
{
	float angle = 0;
	bool command_line = false;
	std::string filename = "output.png";

	if (argc >= 3) {
		command_line = true;
		angle = std::stof(argv[2]); // -r by default
		if (argc == 4) {
			filename = std::string(argv[3]);
		}
		else
			return 0;
	}

	rst::rasterizer r(700, 700);

	Eigen::Vector3f eye_pos = { 0, 0, 5 };

	std::vector<Eigen::Vector3f> pos{ {2, 0, -2}, {0, 2, -2}, {-2, 0, -2} };

	std::vector<Eigen::Vector3i> ind{ {0, 1, 2} };

	auto pos_id = r.load_positions(pos);
	auto ind_id = r.load_indices(ind);

	int key = 0;
	int frame_count = 0;

	if (command_line) {
		r.clear(rst::Buffers::Color | rst::Buffers::Depth);

		r.set_model(get_model_matrix(angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

		r.draw(pos_id, ind_id, rst::Primitive::Triangle);
		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
		image.convertTo(image, CV_8UC3, 1.0f);

		cv::imwrite(filename, image);

		return 0;
	}

	float tempXAngle = 0;
	float tempYAngle = 0;
	float tempZAngle = 0;
	while (key != 27) {
		r.clear(rst::Buffers::Color | rst::Buffers::Depth);

		Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
		rotation =
			get_rotation(Eigen::Vector3f::UnitX(), tempXAngle)*
			get_rotation(Eigen::Vector3f::UnitY(), tempYAngle)*
			get_rotation(Eigen::Vector3f::UnitZ(), tempZAngle);

		//r.set_model(get_model_matrix(angle));
		r.set_model(rotation);
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

		r.draw(pos_id, ind_id, rst::Primitive::Triangle);

		// TODO why the fk the drawing has a huge delay?
		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
		image.convertTo(image, CV_8UC3, 1.0f);
		cv::imshow("image", image);
		key = cv::waitKey(10);

		std::cout << "frame count: " << frame_count++ << '\n';

		//std::cout << "current z angle: " << angle << '\n';
		//if (key == 'a')
		//	angle += 10;
		//else if (key == 'd')
		//	angle -= 10;

		std::cout << "current x angle: " << tempXAngle << '\n';
		std::cout << "current y angle: " << tempYAngle << '\n';
		std::cout << "current z angle: " << tempZAngle << '\n';

		if (key == 'a')
			tempZAngle += 10;
		else if (key == 'd')
			tempZAngle -= 10;
		else if (key == 'w')
			tempXAngle += 10;
		else if (key == 's')
			tempXAngle -= 10;
		else if (key == 'q')
			tempYAngle += 10;
		else if (key == 'e')
			tempYAngle -= 10;
	}

	return 0;
}
