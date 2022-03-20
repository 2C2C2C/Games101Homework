#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata)
{
	if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4)
	{
		std::cout << "Left button of the mouse is clicked - position (" << x << ", "
			<< y << ")" << '\n';
		control_points.emplace_back(x, y);
	}
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window)
{
	auto &p_0 = points[0];
	auto &p_1 = points[1];
	auto &p_2 = points[2];
	auto &p_3 = points[3];

	for (double t = 0.0; t <= 1.0; t += 0.001)
	{
		auto point =
			std::pow(1 - t, 3) * p_0 +
			3 * t * std::pow(1 - t, 2) * p_1 +
			3 * std::pow(t, 2) * (1 - t) * p_2 +
			std::pow(t, 3) * p_3;

		window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
	}
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t)
{
	// TODO: Implement de Casteljau's algorithm
	if (control_points.size() == 2)
	{
		// b01(t) = (1-t)*b0 + t*b1
		float x = (1.0f - t)*control_points[0].x + t * control_points[1].x;
		float y = (1.0f - t)*control_points[0].y + t * control_points[1].y;
		return cv::Point2f(x, y);
	}

	int size = control_points.size();
	std::vector<cv::Point2f> tempPoints;
	for (int i = 0; i < size - 1; i++)
	{
		float tempX = (1.0f - t)*control_points[i].x + t * control_points[i + 1].x;
		float tempY = (1.0f - t)*control_points[i].y + t * control_points[i + 1].y;
		tempPoints.emplace_back(cv::Point2f(tempX, tempY));
	}
	return recursive_bezier(tempPoints, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window)
{
	// TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
	// recursive Bezier algorithm.
	bool doAA = true;
	cv::Point2f aaPointArr[4] = { cv::Point2f(),cv::Point2f() ,cv::Point2f() ,cv::Point2f() };
	for (double t = 0.0; t <= 1.0; t += 0.001)
	{
		auto point = recursive_bezier(control_points, (float)t);

		// draw point color
		// wtf index -> BGR as 012
		if (doAA)
		{
			aaPointArr[0] = cv::Point2f(std::floor(point.x) + 0.5f, std::ceil(point.y) + 0.5f);
			aaPointArr[1] = cv::Point2f(std::ceil(point.x) + 0.5f, std::ceil(point.y) + 0.5f);
			aaPointArr[2] = cv::Point2f(std::floor(point.x) + 0.5f, std::floor(point.y) + 0.5f);
			aaPointArr[3] = cv::Point2f(std::ceil(point.x) + 0.5f, std::floor(point.y) + 0.5f);

			// when it reach the max dist, it's like the topLeft is our point, and the bottom right is definitely too far
			float maxDistance = std::sqrt(2.0f);
			for (int i = 0; i < 4; i++)
			{
				cv::Point2f tempPoint = aaPointArr[i];
				float tempDist = std::sqrt(std::pow((point.x - tempPoint.x), 2) + std::pow((point.y - tempPoint.y), 2));
				float lerpValue = 1.0f - std::min(tempDist / maxDistance, 1.0f);
				window.at<cv::Vec3b>(tempPoint.y, tempPoint.x)[1] = 255.0f * lerpValue;
			}
		}
		else
			window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
	}
}

int main()
{
	cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
	cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
	cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

	cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

	int key = -1;
	while (key != 27)
	{
		for (auto &point : control_points)
		{
			cv::circle(window, point, 3, { 255, 255, 255 }, 3);
		}

		if (control_points.size() == 4)
		{
			//naive_bezier(control_points, window);
			// my bezier
			bezier(control_points, window);

			cv::imshow("Bezier Curve", window);
			cv::imwrite("my_bezier_curve.png", window);
			key = cv::waitKey(0);

			return 0;
		}

		cv::imshow("Bezier Curve", window);
		key = cv::waitKey(20);
	}

	return 0;
}
