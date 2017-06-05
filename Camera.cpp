#include "stdafx.h"
// Part of the code are copied from PMVS
#include <fstream>
#include <cstdlib>
#include "Camera.h"

using namespace std;
using namespace Image;


Camera::Camera(void) {
	m_axesScale = 1.0f;
}

Camera::~Camera() {
}

void Camera::LoadFromFile(const std::string cameraFilePath) {
	m_IntrinsicParameters.resize(6);
	m_ExtrinsicParameters.resize(6);

	ifstream fin(cameraFilePath);

	string header;
	fin >> header;

	for (int i = 0; i < 6; ++i)
		fin >> m_IntrinsicParameters[i];
	for (int i = 0; i < 6; ++i)
		fin >> m_ExtrinsicParameters[i];

	fin.close();

	UpdateCamera();//updating the camera center,axis in world ref frame
}

void Camera::UpdateCamera(void) {
	setProjection(m_IntrinsicParameters, m_ExtrinsicParameters, m_projectionMatrix);

	//----------------------------------------------------------------------
	m_OpticalAxis = m_projectionMatrix[2];
	m_OpticalAxis[3] = 0.0;
	const float ftmp = norm(m_OpticalAxis);
	m_OpticalAxis[3] = m_projectionMatrix[2][3];
	m_OpticalAxis /= ftmp;

	m_OpticalCenter = GetOpticalCenter();

	m_zAxis = Vec3f(m_OpticalAxis[0], m_OpticalAxis[1], m_OpticalAxis[2]);
	m_xAxis = Vec3f(m_projectionMatrix[0][0],
		m_projectionMatrix[0][1],
		m_projectionMatrix[0][2]);
	m_yAxis = cross(m_zAxis, m_xAxis);
	unitize(m_yAxis);
	m_xAxis = cross(m_yAxis, m_zAxis);
}

Vec4f Camera::GetOpticalCenter(void) const {				// [iR*-t 1]
	// orthographic case
	Vec4f ans;
	if (m_projectionMatrix[2][0] == 0.0 && m_projectionMatrix[2][1] == 0.0 &&
		m_projectionMatrix[2][2] == 0.0) {
			Vec3f vtmp[2];
			for (int i = 0; i < 2; ++i)
				for (int y = 0; y < 3; ++y)
					vtmp[i][y] = m_projectionMatrix[i][y];

			Vec3f vtmp2 = cross(vtmp[0], vtmp[1]);
			unitize(vtmp2);
			for (int y = 0; y < 3; ++y)
				ans[y] = vtmp2[y];
			ans[3] = 0.0;
	}
	else {
		Mat3 A;
		Vec3 b;
		for (int y = 0; y < 3; ++y) {
			for (int x = 0; x < 3; ++x)
				A[y][x] = m_projectionMatrix[y][x];
			b[y] = - m_projectionMatrix[y][3];
		}
		Mat3 iA;
		invert(iA, A);
		b = iA * b;

		for (int y = 0; y < 3; ++y)
			ans[y] = b[y];
		ans[3] = 1.0;
	}
	return ans;
}

void Camera::setProjection(const std::vector<float>& intrinsics,
	const std::vector<float>& extrinsics,
	std::vector<Vec4f>& projection) 
{
	projection.resize(3);
	double params[12];
	for (int i = 0; i < 6; ++i) {
		params[i] = intrinsics[i];
		params[6 + i] = extrinsics[i];
	}

	for (int y = 0; y < 3; ++y) 
	{
		for (int x = 0; x < 4; ++x ) 
		{
			projection[y][x] = params[4 * y + x];
		}
	}


}

void Camera::setProjectionSub(double params[], std::vector<Vec4f>& projection, const int level) {
	const double rx = params[6] * M_PI / 180.0;
	const double ry = params[7] * M_PI / 180.0;
	const double rz = params[8] * M_PI / 180.0;

	const double fovx = params[0] * M_PI / 180.0;

	const double f = params[1] / 2.0 / tan(fovx / 2.0);
	Mat3 K;
	K[0] = Vec3(f, 0.0, 0.0);
	K[1] = Vec3(0.0, f, 0.0);
	K[2] = Vec3(0.0, 0.0, -1.0);

	Mat3 trans;
	trans[0] = Vec3(1.0, 0.0, params[1] / 2.0);
	trans[1] = Vec3(0.0, -1.0, params[2] / 2.0);
	trans[2] = Vec3(0.0, 0.0, 1.0);

	K = trans * K;

	Mat3 Rx;
	Rx[0] = Vec3(1.0, 0.0, 0.0);
	Rx[1] = Vec3(0.0f, cos(rx), -sin(rx));
	Rx[2] = Vec3(0.0, sin(rx), cos(rx));

	Mat3 Ry;
	Ry[0] = Vec3(cos(ry), 0, sin(ry));
	Ry[1] = Vec3(0.0, 1.0, 0.0);
	Ry[2] = Vec3(-sin(ry), 0, cos(ry));

	Mat3 Rz;
	Rz[0] = Vec3(cos(rz), -sin(rz), 0.0);
	Rz[1] = Vec3(sin(rz), cos(rz), 0.0);
	Rz[2] = Vec3(0.0, 0.0, 1.0);

	//????????
	//Mat3 R = transpose(Rz) * transpose(Rx) * transpose(Ry);
	Mat3 R = transpose(Rx) * transpose(Ry) * transpose(Rz);

	Vec3 t(params[3], params[4], params[5]);

	Mat3 left = K * R;
	Vec3 right = - K * (R * t);

	for (int y = 0; y < 3; ++y) {
		for (int x = 0; x < 3; ++x)
			projection[y][x] = left[y][x];
		projection[y][3] = right[y];
	}

	const int scale = 0x0001 << level;
	projection[0] /= scale;
	projection[1] /= scale;
}

void Camera::proj2q(Mat4& mat, double q[6]) {
	double s;
	int i;

	q[3] = mat[0][3];
	q[4] = mat[1][3];
	q[5] = mat[2][3];
	q[0] = 0;
	q[1] = 0;
	q[2] = 0;
	if (mat[2][0] == 1.0) {
		q[1] = (double) -M_PI/2.0;
		q[2] = 0;
		q[0]=atan2(-mat[0][1],mat[1][1]);
	}
	else {
		if (mat[2][0] == -1.0) { 
			q[1] = M_PI/2.0;
			q[2] = 0;
			q[0]=atan2(mat[0][1],mat[1][1]);    
		}
		else {
			q[1] = (double)  asin(-mat[2][0]);
			if (cos(q[1]) > 0.0) { s = 1.0;} else { s =-1.0;};
			q[0] =atan2(mat[2][1]*s, mat[2][2]*s); 
			q[2] =atan2(mat[1][0]*s, mat[0][0]*s); 
		}
	}
	q[0]=q[0]*180/M_PI;//RadInDeg;
	q[1]=q[1]*180/M_PI;//RadInDeg;
	q[2]=q[2]*180/M_PI;//RadInDeg;
	for(i=0;i<3;i++){
		if (fabs(q[i])>180.0){
			q[i]= (q[i]>0) ? q[i]-360.0 : q[i]+360.0;
		}
	}
}

void Camera::q2proj(const double q[6], Mat4& mat) {
	const double a = q[0] * M_PI / 180.0;
	const double b = q[1] * M_PI / 180.0;
	const double g = q[2] * M_PI / 180.0;

	const double s1=sin(a);  const double s2=sin(b);  const double s3=sin(g);
	const double c1=cos(a);  const double c2=cos(b);  const double c3=cos(g);

	/*   Premiere colonne*/	/*   Seconde colonne	*/
	mat[0][0]=c2*c3; 		mat[0][1]=c3*s2*s1-s3*c1;  
	mat[1][0]=s3*c2; 		mat[1][1]=s3*s2*s1+c3*c1; 
	mat[2][0]=-s2;   		mat[2][1]=c2*s1;

	/*   Troisieme colonne*/	/*  Quatrieme colonne	*/
	mat[0][2]=c3*s2*c1+s3*s1; 	mat[0][3]=q[3]; 
	mat[1][2]=s3*s2*c1-c3*s1; 	mat[1][3]=q[4]; 
	mat[2][2]=c2*c1; 		mat[2][3]=q[5];

	mat[3][0] = mat[3][1] = mat[3][2] = 0.0;
	mat[3][3] = 1.0;
}


bool Camera::IsInSimilarDirectionWithAnotherCamera(Camera* other)
{
	const float threshold = 60.0f * M_PI / 180.0f;

	Vec4f rayOAxis = m_OpticalAxis;
	rayOAxis[3] = 0.0f;

	Vec4f rayOAxisOfOther = other->m_OpticalAxis;
	rayOAxisOfOther[3] = 0.0f;

	//std::cout << "cos(theta) = " << rayOAxis * rayOAxisOfOther << std::endl;

	return rayOAxis * rayOAxisOfOther > cos(threshold);
}

Vec4f Camera::UnprojectTo3D(const Vec3f& icoord) const 
{
	Mat3 A;
	Vec3 b(icoord[0], icoord[1], icoord[2]);
	for (int y = 0; y < 3; ++y) 
	{
		for (int x = 0; x < 3; ++x)
			A[y][x] = m_projectionMatrix[y][x];
		b[y] -= m_projectionMatrix[y][3];    
	}
	Mat3 IA;
	invert(IA, A);
	Vec3 x = IA * b;
	return Vec4f(x, 1.0f);
}
