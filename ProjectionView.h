#pragma once
#include "Camera.h"
#include <opencv2/opencv.hpp>
using namespace std;
class ProjectionView
{
public:
	ProjectionView(int id,string cameraPath,string imagePath);
	~ProjectionView(void);

public:
	void LoadRelatedViews(const vector<ProjectionView*>& allViews);
	void InitialMatch();
	Vec3f GetPixel(int x, int y);
	Vec3f GetPixel(float x, float y);

public:
	Image::Camera* m_Camera;
	cv::Mat m_Image;   //reference image
	std::vector<ProjectionView*> m_AllPossibleRelatedViews;//all the images to consider
	std::string m_ImagePath;
	int imageWidth, imageHeight;
	int m_ID;
};

