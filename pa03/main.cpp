#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

float angleToRad(float angle)
{
	return angle * (float)MY_PI / 180.0;
}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
	Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

	Eigen::Matrix4f translate;
	translate << 1, 0, 0, -eye_pos[0],
		0, 1, 0, -eye_pos[1],
		0, 0, 1, -eye_pos[2],
		0, 0, 0, 1;

	view = translate * view;

	return view;
}

Eigen::Matrix4f get_model_matrix(float angle)
{
	Eigen::Matrix4f rotation;
	angle = angle * MY_PI / 180.f;
	rotation << cos(angle), 0, sin(angle), 0,
		0, 1, 0, 0,
		-sin(angle), 0, cos(angle), 0,
		0, 0, 0, 1;

	Eigen::Matrix4f scale;
	scale << 2.5, 0, 0, 0,
		0, 2.5, 0, 0,
		0, 0, 2.5, 0,
		0, 0, 0, 1;

	Eigen::Matrix4f translate;
	translate << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
	// --done TODO: Use the same projection matrix from the previous assignments
	Eigen::Matrix4f projection;

	float yTop = 0, yBottom = 0;
	float xLeft = 0, xRight = 0;

	float eye_fov_rad = angleToRad(eye_fov * 0.5f);
	// o.g yTop = tan(eye_fov_rad) * zNear;
	// HACK @Hiko to prevent y-reverse
	yTop = tan(eye_fov_rad) * -zNear;
	xRight = aspect_ratio * yTop;
	xLeft = -xRight;
	yBottom = -yTop;

	Eigen::Matrix4f zTransform;
	zTransform <<
		zNear, 0, 0, 0,
		0, zNear, 0, 0,
		0, 0, zNear + zFar, -zNear * zFar,
		0, 0, 1, 0;

	Eigen::Matrix4f scaleTransform;
	scaleTransform <<
		2 / (xRight - xLeft), 0, 0, 0,
		0, 2 / (yTop - yBottom), 0, 0,
		0, 0, 2 / (zNear - zFar), 0,
		0, 0, 0, 1;

	Eigen::Matrix4f orthoMoveTransform;
	orthoMoveTransform <<
		1, 0, 0, -(xRight + xLeft) * 0.5f,
		0, 1, 0, -(yTop + yBottom) * 0.5f,
		0, 0, 1, -(zNear + zFar) * 0.5f,
		0, 0, 0, 1;

	// !!! move first, second scale
	// first 2 are for ortho projection
	projection = scaleTransform * orthoMoveTransform * zTransform;
	return projection;
}

Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
	return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
	Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
	Eigen::Vector3f result;
	result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
	return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
	auto costheta = vec.dot(axis);
	return (2 * costheta * axis - vec).normalized();
}

struct light
{
	Eigen::Vector3f position;
	Eigen::Vector3f intensity;

	light(Eigen::Vector3f pos, Eigen::Vector3f i)
	{
		this->position = pos;
		this->intensity = i;
	}
};

struct LightParameter
{
	Eigen::Vector3f ka;
	Eigen::Vector3f kd;
	Eigen::Vector3f ks;
	Eigen::Vector3f amb_light;

	// parameters : ka, kd, ks, ambient
	LightParameter(Eigen::Vector3f inKa, Eigen::Vector3f inKd, Eigen::Vector3f inKs, Eigen::Vector3f inAmb)
	{
		this->ka = inKa;
		this->kd = inKd;
		this->ks = inKs;
		this->amb_light = inAmb;
	}
};

struct RefelectionData
{
	Eigen::Vector3f point;
	Eigen::Vector3f normal;
	Eigen::Vector3f eyePos;
	float p;

	// parameters : reflection point, normal, eye position, p value (100 ~ 200)
	RefelectionData(Eigen::Vector3f inPoint, Eigen::Vector3f inNormal, Eigen::Vector3f inEyePos, float inPValue)
	{
		this->point = inPoint;
		this->normal = inNormal;
		this->eyePos = inEyePos;
		this->p = inPValue;
	}
};

Eigen::Vector3f calculate_result_color(light lightData, LightParameter lightParameter, RefelectionData refelectionData)
{
	// --done  TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
	// components are. Then, accumulate that result on the *result_color* object.
	Eigen::Vector3f result_color = { 0, 0, 0 };
	Eigen::Vector3f ambient(0.0f, 0.0f, 0.0f);
	Eigen::Vector3f diffuse(0.0f, 0.0f, 0.0f);
	Eigen::Vector3f specular(0.0f, 0.0f, 0.0f);

	Eigen::Vector3f lightDir = (lightData.position - refelectionData.point);
	float rSqr = lightDir.squaredNorm();
	lightDir.normalize();

	// ambient La = Ka * Ia
	ambient[0] = lightParameter.ka[0] * lightParameter.amb_light[0];
	ambient[1] = lightParameter.ka[1] * lightParameter.amb_light[1];
	ambient[2] = lightParameter.ka[2] * lightParameter.amb_light[2];

	float lightIntensityX = lightData.intensity[0] / rSqr;
	float lightIntensityY = lightData.intensity[1] / rSqr;
	float lightIntensityZ = lightData.intensity[2] / rSqr;

	// diffuse Ld = kd(I / r^2) max(0, n * l)
	diffuse[0] = lightParameter.kd[0] * lightIntensityX * MAX(0.0f, refelectionData.normal.dot(lightDir));
	diffuse[1] = lightParameter.kd[1] * lightIntensityY * MAX(0.0f, refelectionData.normal.dot(lightDir));
	diffuse[2] = lightParameter.kd[2] * lightIntensityZ * MAX(0.0f, refelectionData.normal.dot(lightDir));

	Eigen::Vector3f viewDir = (refelectionData.eyePos - refelectionData.point).normalized();
	Eigen::Vector3f specularDirH = (viewDir + lightDir).normalized();

	// specular Ls = Ks*(I/r^2) max(0,cos a)^p = Ks * (I / r ^ 2) max(0, n * h) ^ p ; [p : 100 ~ 200]
	specular[0] = lightParameter.ks[0] * lightIntensityX * std::pow(MAX(0.0f, refelectionData.normal.dot(specularDirH)), refelectionData.p);
	specular[1] = lightParameter.ks[1] * lightIntensityY *	std::pow(MAX(0.0f, refelectionData.normal.dot(specularDirH)), refelectionData.p);
	specular[2] = lightParameter.ks[2] * lightIntensityZ * std::pow(MAX(0.0f, refelectionData.normal.dot(specularDirH)), refelectionData.p);

	// L = La + Ld + Ls
	result_color += ambient + diffuse + specular;

	return result_color;
}

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
	Eigen::Vector3f return_color = { 0, 0, 0 };
	if (payload.texture)
	{
		// --done TODO: Get the texture value at the texture coordinates of the current fragment
		return_color = payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y());
	}
	Eigen::Vector3f texture_color;
	texture_color << return_color.x(), return_color.y(), return_color.z();

	Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
	Eigen::Vector3f kd = texture_color / 255.f;
	Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

	auto l1 = light{ {20, 20, 20}, {500, 500, 500} };
	auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };

	std::vector<light> lights = { l1, l2 };
	Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
	Eigen::Vector3f eye_pos{ 0, 0, 10 };

	float p = 150;

	Eigen::Vector3f color = texture_color;
	Eigen::Vector3f point = payload.view_pos;
	Eigen::Vector3f normal = payload.normal;

	Eigen::Vector3f result_color = { 0, 0, 0 };

	LightParameter lightParameter(ka, kd, ks, amb_light_intensity);
	RefelectionData refelectionData(point, normal, eye_pos, p);

	// --done TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
	// components are. Then, accumulate that result on the *result_color* object.
	for (auto& light : lights)
		result_color += calculate_result_color(light, lightParameter, refelectionData);

	return result_color * 255.f;
}

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
	Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
	Eigen::Vector3f kd = payload.color;
	Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

	auto l1 = light{ {20, 20, 20}, {500, 500, 500} };
	auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };

	std::vector<light> lights = { l1, l2 };
	Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
	Eigen::Vector3f eye_pos{ 0, 0, 10 };

	float p = 150;

	Eigen::Vector3f color = payload.color;
	Eigen::Vector3f point = payload.view_pos;
	Eigen::Vector3f normal = payload.normal;

	Eigen::Vector3f result_color = { 0, 0, 0 };
	LightParameter lightParameter(ka, kd, ks, amb_light_intensity);
	RefelectionData refelectionData(point, normal, eye_pos, p);

	// --done TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
	// components are. Then, accumulate that result on the *result_color* object.
	for (auto& light : lights)
		result_color += calculate_result_color(light, lightParameter, refelectionData);

	return result_color * 255.f;
}

Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
	Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
	Eigen::Vector3f kd = payload.color;
	Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

	auto l1 = light{ {20, 20, 20}, {500, 500, 500} };
	auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };

	std::vector<light> lights = { l1, l2 };
	Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
	Eigen::Vector3f eye_pos{ 0, 0, 10 };

	float p = 150;

	Eigen::Vector3f color = payload.color;
	Eigen::Vector3f point = payload.view_pos;
	Eigen::Vector3f normal = payload.normal;

	float kh = 0.2, kn = 0.1;

	// --done TODO: Implement displacement mapping here

	//Let n = normal = (x, y, z)
	//Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
	//Vector b = n cross product t
	//Matrix TBN = [t b n]
	//dU = kh * kn * (h(u+1/w,v)-h(u,v))
	//dV = kh * kn * (h(u,v+1/h)-h(u,v))
	//Vector ln = (-dU, -dV, 1)
	//Normal n = normalize(TBN * ln)
	//Position p = p + kn * n * h(u,v)

	Eigen::Vector3f t(normal.x() * normal.y() / std::sqrt(std::pow(normal.x(), 2) + std::pow(normal.z(), 2)),
		std::sqrt(std::pow(normal.x(), 2) + std::pow(normal.z(), 2)),
		normal.z() * normal.y() / sqrt(std::pow(normal.x(), 2) + std::pow(normal.z(), 2)));

	Eigen::Vector3f b = normal.cross(t);
	Eigen::Matrix3f TBN;
	TBN.col(0) = t.normalized();
	TBN.col(0) = b.normalized();
	TBN.col(0) = normal;

	int w = payload.texture->width;
	int h = payload.texture->height;
	float u = payload.tex_coords.x();
	float v = payload.tex_coords.y();

	float huv = payload.texture->getColor(u, v).norm();

	float dU = kh * kn * (payload.texture->getColor(u + 1.0f / w, v).norm() - huv);
	float dV = kh * kn * (payload.texture->getColor(u, v + 1.0f / h).norm() - huv);

	Eigen::Vector3f ln(-dU, -dV, 1);
	Eigen::Vector3f normalTemp = (TBN * ln).normalized();
	Eigen::Vector3f posTemp = point + kn * normalTemp * huv;

	// displacement mapping here

	Eigen::Vector3f result_color = { 0, 0, 0 };
	LightParameter lightParameter(ka, kd, ks, amb_light_intensity);
	RefelectionData refelectionData(posTemp, normal, eye_pos, p);

	// --done TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
	// components are. Then, accumulate that result on the *result_color* object.
	for (auto& light : lights)
		result_color += calculate_result_color(light, lightParameter, refelectionData);

	return result_color * 255.f;
}

Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
	Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
	Eigen::Vector3f kd = payload.color;
	Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

	auto l1 = light{ {20, 20, 20}, {500, 500, 500} };
	auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };

	std::vector<light> lights = { l1, l2 };
	Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
	Eigen::Vector3f eye_pos{ 0, 0, 10 };

	float p = 150;

	Eigen::Vector3f color = payload.color;
	Eigen::Vector3f point = payload.view_pos;
	Eigen::Vector3f normal = payload.normal;


	float kh = 0.2, kn = 0.1;

	// --done TODO: Implement displacement mapping here

	//Let n = normal = (x, y, z)
	//Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
	//Vector b = n cross product t
	//Matrix TBN = [t b n]
	//dU = kh * kn * (h(u+1/w,v)-h(u,v))
	//dV = kh * kn * (h(u,v+1/h)-h(u,v))
	//Vector ln = (-dU, -dV, 1)
	//Normal n = normalize(TBN * ln)
	//Position p = p + kn * n * h(u,v)

	Eigen::Vector3f t(normal.x() * normal.y() / std::sqrt(std::pow(normal.x(), 2) + std::pow(normal.z(), 2)),
		std::sqrt(std::pow(normal.x(), 2) + std::pow(normal.z(), 2)),
		normal.z() * normal.y() / sqrt(std::pow(normal.x(), 2) + std::pow(normal.z(), 2)));

	Eigen::Vector3f b = normal.cross(t);
	Eigen::Matrix3f TBN;
	TBN.col(0) = t.normalized();
	TBN.col(0) = b.normalized();
	TBN.col(0) = normal;

	int w = payload.texture->width;
	int h = payload.texture->height;
	float u = payload.tex_coords.x();
	float v = payload.tex_coords.y();

	float huv = payload.texture->getColor(u, v).norm();

	float dU = kh * kn * (payload.texture->getColor(u + 1.0f / w, v).norm() - huv);
	float dV = kh * kn * (payload.texture->getColor(u, v + 1.0f / h).norm() - huv);

	Eigen::Vector3f ln(-dU, -dV, 1);
	Eigen::Vector3f normalTemp = (TBN * ln).normalized();

	// --done TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
	// components are. Then, accumulate that result on the *result_color* object.

	Eigen::Vector3f result_color = normalTemp;
	return result_color * 255.f;
}

int main(int argc, const char** argv)
{
	std::vector<Triangle*> TriangleList;

	int angle = 140;
	bool command_line = false;

	std::string filename = "output.png";
	objl::Loader Loader;
	std::string obj_path = "../models/spot/";

	// Load .obj File
	bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
	for (auto mesh : Loader.LoadedMeshes)
	{
		for (int i = 0; i < mesh.Vertices.size(); i += 3)
		{
			Triangle* t = new Triangle();
			for (int j = 0; j < 3; j++)
			{
				t->setVertex(j, Vector4f(mesh.Vertices[i + j].Position.X, mesh.Vertices[i + j].Position.Y, mesh.Vertices[i + j].Position.Z, 1.0));
				t->setNormal(j, Vector3f(mesh.Vertices[i + j].Normal.X, mesh.Vertices[i + j].Normal.Y, mesh.Vertices[i + j].Normal.Z));
				t->setTexCoord(j, Vector2f(mesh.Vertices[i + j].TextureCoordinate.X, mesh.Vertices[i + j].TextureCoordinate.Y));
			}
			TriangleList.push_back(t);
		}
	}

	rst::rasterizer r(700, 700);

	auto texture_path = "hmap.jpg";
	r.set_texture(Texture(obj_path + texture_path));

	std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;

	if (argc >= 2)
	{
		command_line = true;
		filename = std::string(argv[1]);

		if (argc == 3 && std::string(argv[2]) == "texture")
		{
			std::cout << "Rasterizing using the texture shader\n";
			active_shader = texture_fragment_shader;
			texture_path = "spot_texture.png";
			r.set_texture(Texture(obj_path + texture_path));
		}
		else if (argc == 3 && std::string(argv[2]) == "normal")
		{
			std::cout << "Rasterizing using the normal shader\n";
			active_shader = normal_fragment_shader;
		}
		else if (argc == 3 && std::string(argv[2]) == "phong")
		{
			std::cout << "Rasterizing using the phong shader\n";
			active_shader = phong_fragment_shader;
		}
		else if (argc == 3 && std::string(argv[2]) == "bump")
		{
			std::cout << "Rasterizing using the bump shader\n";
			active_shader = bump_fragment_shader;
		}
		else if (argc == 3 && std::string(argv[2]) == "displacement")
		{
			std::cout << "Rasterizing using the bump shader\n";
			active_shader = displacement_fragment_shader;
		}
	}

	Eigen::Vector3f eye_pos = { 0,0,10 };

	r.set_vertex_shader(vertex_shader);
	r.set_fragment_shader(active_shader);

	int key = 0;
	int frame_count = 0;

	if (command_line)
	{
		r.clear(rst::Buffers::Color | rst::Buffers::Depth);
		r.set_model(get_model_matrix(angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

		r.draw(TriangleList);
		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
		image.convertTo(image, CV_8UC3, 1.0f);
		cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

		cv::imwrite(filename, image);

		return 0;
	}

	while (key != 27)
	{
		if (key == 'a')
		{
			angle -= 5;
		}
		else if (key == 'd')
		{
			angle += 5;
		}
		angle = angle >= 0 ? angle % 360 : 360 + angle;
		std::cout << "check angle: " << angle << std::endl;

		if (key < '6' && key > '0')
		{
			switch (key)
			{
			case '1':
				std::cout << "Rasterizing using the texture shader\n";
				active_shader = texture_fragment_shader;
				texture_path = "spot_texture.png";
				r.set_texture(Texture(obj_path + texture_path));
				break;
			case '2':
				std::cout << "Rasterizing using the normal shader\n";
				active_shader = normal_fragment_shader;
				break;
			case '3':
				std::cout << "Rasterizing using the phong shader\n";
				active_shader = phong_fragment_shader;
				break;
			case '4':
				std::cout << "Rasterizing using the bump shader\n";
				active_shader = bump_fragment_shader;
				break;
			case '5':
				std::cout << "Rasterizing using the bump shader\n";
				active_shader = displacement_fragment_shader;
				break;
			default:
				break;
			}
			r.set_fragment_shader(active_shader);

			r.clear(rst::Buffers::Color | rst::Buffers::Depth);
			r.set_model(get_model_matrix(angle));
			r.set_view(get_view_matrix(eye_pos));
			r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

			r.draw(TriangleList);
			cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
			image.convertTo(image, CV_8UC3, 1.0f);
			cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

			cv::imwrite(filename, image);
		}
	}
	return 0;
}
