#include "stdafx.h"
#include "ProjectionView.h"
#include <fstream>
#include <vector>

using namespace Image;

ProjectionView::ProjectionView(int id, std::string cameraPath, std::string imagePath)
{
	m_ID = id; 									//image index
	m_ImagePath = imagePath;		//path
	m_Camera = new Image::Camera();//creating a camera class obj
	m_Camera->LoadFromFile(cameraPath);//loading camera vals and update the center and axis of camera in world ref frame
	m_Image = cv::imread(imagePath);//reading img
	cv::Size imgSize = m_Image.size();//size is struct having heigth and width as members
	imageHeight = imgSize.height;
	imageWidth = imgSize.width;
	//cout<<imageHeight;
}

ProjectionView::~ProjectionView(void)
{
}

void ProjectionView::LoadRelatedViews(const std::vector<ProjectionView*>& allViews)
{
	m_AllPossibleRelatedViews.clear();
	for(auto vIterator = allViews.begin(); vIterator != allViews.end(); vIterator++)
	{
		if((*vIterator)->m_ID == m_ID)
			continue;

		ProjectionView* candidateView = *vIterator;
		if(this->m_Camera->IsInSimilarDirectionWithAnotherCamera(candidateView->m_Camera))	//the other camerea lies in 60 deg angle
		{
			m_AllPossibleRelatedViews.push_back(candidateView);
		}
	}
}

Vec3f ProjectionView::GetPixel(int x, int y)
{
	if(x > imageWidth - 1 || y > imageHeight - 1 || x < 0 || y < 0)
		return Vec3f(-100, -100, -100);

	cv::Vec3b pixelValue = m_Image.at<cv::Vec3b>(y, x);
	return Vec3f( 
		pixelValue.val[0] / 255.0f,
		pixelValue.val[1] / 255.0f,
		pixelValue.val[2] / 255.0f );
}

Vec3f ProjectionView::GetPixel(float x, float y)
{
	if(x > imageWidth - 1 || y > imageHeight - 1 || x < 0 || y < 0)
		return -100;

	int pt00x = x, pt00y = y;
	int pt01x = pt00x + 1, pt01y = y;
	int pt10x = x, pt10y = pt00y+1;
	int pt11x = pt00x + 1, pt11y = pt00y + 1;

	if(pt11x >= imageWidth| pt11y >= imageHeight)
	{
		return GetPixel(pt00y, pt00x);
	}

	return ( 
			GetPixel(pt00x, pt00y) * (pt11x - x) * (pt11y - y)
		+	GetPixel(pt01x, pt01y) * ( x - pt00x) * (pt11y - y)
		+	GetPixel(pt10x, pt10y) * (pt11x - x) * (y - pt00y)
		+	GetPixel(pt11x, pt11y) * (x - pt00x) * (y - pt00y) );
}

