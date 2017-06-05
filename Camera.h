#pragma once
#include <vector>
#include <string>
#include <climits>
#include "Vec4.h"
#include "Mat4.h"
#include "Mat3.h"

namespace Image {

	class Camera {
	public:
		Camera(void);
		virtual ~Camera();

		// Update projection matrices from intrinsics and extrinsics
		void UpdateProjection(void);
		// Update all the camera related parameters
		void UpdateCamera(void);

		bool IsInSimilarDirectionWithAnotherCamera(Camera* other);

		virtual void LoadFromFile(const std::string cname);

		inline Vec3f Project(const Vec4f& coord) const;
		inline Vec3f mult(const Vec4f& coord, const int level) const;

		static void setProjection(const std::vector<float>& intrinsics,
			const std::vector<float>& extrinsics,
			std::vector<Vec4f>& projection);

		float getScale(const Vec4f& coord, const int level) const;
		void getPAxes(const Vec4f& coord, const Vec4f& normal,
			Vec4f& pxaxis, Vec4f& pyaxis, const int level = 0) const;

		void setAxesScale(const float axesScale);

		static void proj2q(Mat4& mat, double q[6]);
		static void q2proj(const double q[6], Mat4& mat);  
		static void setProjectionSub(double params[], std::vector<Vec4f>& projection,
			const int level);
		Vec4f GetOpticalCenter(void) const;
		Vec4f UnprojectTo3D(const Vec3f& icoord) const;

		Vec4f m_OpticalCenter;
		Vec4f m_OpticalAxis;

		float m_ipscale;
		std::vector<Vec4f> m_projectionMatrix;
		Vec3f m_xAxis;
		Vec3f m_yAxis;
		Vec3f m_zAxis;

		std::vector<float> m_IntrinsicParameters;
		std::vector<float> m_ExtrinsicParameters;
	protected:
		float m_axesScale;

	};

	inline Vec3f Camera::Project(const Vec4f& coord) const {
			Vec3f vtmp;    
			for (int i = 0; i < 3; ++i)
				vtmp[i] = m_projectionMatrix[i] * coord;

			if (vtmp[2] <= 0.0) {
				vtmp[0] = -0xffff;
				vtmp[1] = -0xffff;
				vtmp[2] = -1.0f;
				return vtmp;
			}
			else
				vtmp /= vtmp[2];

			vtmp[0] = std::max((float)(INT_MIN + 3.0f),
				std::min((float)(INT_MAX - 3.0f),
				vtmp[0]));
			vtmp[1] = std::max((float)(INT_MIN + 3.0f),
				std::min((float)(INT_MAX - 3.0f),
				vtmp[1]));

			return vtmp;
	};

	inline Vec3f Camera::mult(const Vec4f& coord,
		const int level) const {
			Vec3f vtmp;    
			for (int i = 0; i < 3; ++i)
				vtmp[i] = m_projectionMatrix[i] * coord;

			return vtmp;
	};

	template<class T>
	void CalculateFundamentalMatrix(const Image::Camera& view0, const Image::Camera& view1,
		TMat3<T>& F) 
	{
			const TVec4<T>& p00 = view0.m_projectionMatrix[0];
			const TVec4<T>& p01 = view0.m_projectionMatrix[1];
			const TVec4<T>& p02 = view0.m_projectionMatrix[2];

			const TVec4<T>& p10 = view1.m_projectionMatrix[0];
			const TVec4<T>& p11 = view1.m_projectionMatrix[1];
			const TVec4<T>& p12 = view1.m_projectionMatrix[2];

			F[0][0] = det(TMat4<T>(p01, p02, p11, p12));
			F[0][1] = det(TMat4<T>(p01, p02, p12, p10));
			F[0][2] = det(TMat4<T>(p01, p02, p10, p11));

			F[1][0] = det(TMat4<T>(p02, p00, p11, p12));
			F[1][1] = det(TMat4<T>(p02, p00, p12, p10));
			F[1][2] = det(TMat4<T>(p02, p00, p10, p11));

			F[2][0] = det(TMat4<T>(p00, p01, p11, p12));
			F[2][1] = det(TMat4<T>(p00, p01, p12, p10));
			F[2][2] = det(TMat4<T>(p00, p01, p10, p11));
	};

}; // namespace image

