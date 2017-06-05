#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <fstream>
#include "stdafx.h"
#include "ProjectionView.h"
//#include "Vec4.h"
#define MAX_IMG 1
using namespace std;
int main(){
	vector<ProjectionView*> views;
	for(int i=0;i<MAX_IMG;i++)
	{
			string str=to_string(i);
			if(str.length()==1)
				str="0"+str;
			//string s1(),s2(),s3(),s4(".txt");
			string image_path="visualize/000000"+str+".jpg",txt_path="txt/000000"+str+".txt";
			views.push_back(new ProjectionView(i,txt_path,image_path));
	}
	cout<<views[0]->imageHeight<<' '<<views[0]->imageWidth<<endl;
	//cout<<views[0].m_Image.cols<<' '<<views[0].m_Image.rows<<endl;
	//return 0;
}
