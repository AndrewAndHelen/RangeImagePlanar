#ifndef RANGEIMAGEPLANAR_H
#define RANGEIMAGEPLANAR_H

#include <pcl/point_cloud.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/common/vector_average.h>
#include <memory>

#include "range_image_variable.h"
#include "range_image_util.h"
class RangeImagePlanar:public pcl::PointCloud<pcl::PointWithRange>
{
public:
	typedef pcl::PointCloud<pcl::PointWithRange> BaseClass;
	typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > VectorOfEigenVector3f;
	typedef std::shared_ptr<RangeImagePlanar> Ptr;
	typedef std::shared_ptr<const RangeImagePlanar> ConstPtr;

	RangeImagePlanar()
	{
		reset();
		unobserved_point_.x = unobserved_point_.y = unobserved_point_.z = std::numeric_limits<float>::quiet_NaN();
		unobserved_point_.range = -std::numeric_limits<float>::infinity();
	};

	virtual ~RangeImagePlanar() {};

	Ptr makeShared() { return Ptr(new RangeImagePlanar(*this)); }

	void reset()
	{
		is_dense = true;
		width = height = 0;
		points.clear();

		location_.setZero();
		to_range_system_.setIdentity();

		format_x_ = format_y_ = 0;
		pixel_size_ = 0;
		focal_length_ = 0;
		x0_ = y0_ = 0;
	}

	void createFromPointCloudWithFixedSize(const pcl::PointCloud<pcl::PointXYZ>& point_cloud,
		BACameraIntrinsics intrinsic, BACamera extrinsic,
		float noise_level = 0.0f, float min_range = 0.0f);

	void doZBuffer(const pcl::PointCloud<pcl::PointXYZ>& point_cloud, float noise_level, float min_range, int& top, int& right, int& bottom, int& left);

	void calculate3DPoint(float image_x, float image_y, float range, pcl::PointWithRange& point) const;

	void recalculate3DPointPositions();

	void real2DToInt2D(float x, float y, int& xInt, int& yInt) const;

	bool isInImage(int x, int y) const;

	void calculate3DPoint(float image_x, float image_y, float range, Eigen::Vector3f& point) const;

	void getImagePoint(const Eigen::Vector3f& point, float& image_x, float& image_y, float& range) const;

	// =====MEMBER VARIABLES=====
	// BaseClass:
	//   roslib::Header header;
	//   std::vector<PointT> points;
	//   uint32_t width;
	//   uint32_t height;
	//   bool is_dense;

public:
	 Eigen::MatrixXf difference_x_;
	 Eigen::MatrixXf difference_y_;

protected:
	Eigen::Vector3d location_;
	Eigen::Matrix3d to_range_system_; //phi, omg, kap

	float format_x_, format_y_;
	float pixel_size_;//< Actual length of each pixel(millimetre)	
	float focal_length_;//< The focal length of the image(millimetre)
	float x0_, y0_;

	pcl::PointWithRange unobserved_point_;
};

#endif // RANGEIMAGEPLANAR_H
