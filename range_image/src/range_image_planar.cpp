#include "range_image_planar.h"


/////////////////////////////////////////////////////////////////////////
void RangeImagePlanar::createFromPointCloudWithFixedSize(const pcl::PointCloud<pcl::PointXYZ>& point_cloud,
	BACameraIntrinsics intrinsic, BACamera extrinsic,
	float noise_level, float min_range)
{
	location_ = extrinsic.Location;
	to_range_system_ = caculateRotation(extrinsic.Posture(EOS_PHI), extrinsic.Posture(EOS_OMG), extrinsic.Posture(EOS_KAP));

	format_x_ = intrinsic.format_x;
	format_y_ = intrinsic.format_y;
	pixel_size_ = intrinsic.pixelsize;
	focal_length_ = intrinsic.intrins[OFFSET_F0];
	x0_ = intrinsic.intrins[OFFSET_X0];
	y0_ = intrinsic.intrins[OFFSET_Y0];

	width = format_x_ / pixel_size_;
	height = format_y_ / pixel_size_;

	is_dense = false;

	difference_x_ = Eigen::MatrixXf::Zero(height, width);
	difference_y_ = Eigen::MatrixXf::Zero(height, width);

	unsigned int size = width * height;
	points.clear();
	points.resize(size, unobserved_point_);

	int top = height, right = -1, bottom = -1, left = width;
	doZBuffer(point_cloud, noise_level, min_range, top, right, bottom, left);

	recalculate3DPointPositions();
}

void RangeImagePlanar::doZBuffer(const pcl::PointCloud<pcl::PointXYZ>& point_cloud, float noise_level, float min_range, int& top, int& right, int& bottom, int& left)
{
	const  std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> &points_set = point_cloud.points;

	unsigned int size = width * height;
	std::vector<int> counters(size, 0);

	top = height; right = -1; bottom = -1; left = width;

	float x_real, y_real, range_of_current_point;
	int x, y;
	for (auto it = points_set.begin(); it != points_set.end(); ++it)
	{
		if (!isFinite(*it))  // Check for NAN etc
			continue;
		pcl::Vector3fMapConst current_point = it->getVector3fMap();

		getImagePoint(current_point, x_real, y_real, range_of_current_point);
		real2DToInt2D(x_real, y_real, x, y);

		if (range_of_current_point < min_range || !isInImage(x, y))
			continue;

		// Do some minor interpolation by checking the three closest neighbors to the point, that are not filled yet.
		int floor_x = pcl_lrint(floor(x_real)), floor_y = pcl_lrint(floor(y_real)),
			ceil_x = pcl_lrint(ceil(x_real)), ceil_y = pcl_lrint(ceil(y_real));

		int neighbor_x[4], neighbor_y[4];
		neighbor_x[0] = floor_x; neighbor_y[0] = floor_y;
		neighbor_x[1] = floor_x; neighbor_y[1] = ceil_y;
		neighbor_x[2] = ceil_x;  neighbor_y[2] = floor_y;
		neighbor_x[3] = ceil_x;  neighbor_y[3] = ceil_y;

		for (int i = 0; i < 4; ++i)
		{
			int n_x = neighbor_x[i], n_y = neighbor_y[i];

			if (n_x == x && n_y == y)
				continue;
			if (isInImage(n_x, n_y))
			{
				int neighbor_array_pos = n_y * width + n_x;
				if (counters[neighbor_array_pos] == 0)
				{
					float& neighbor_range = points[neighbor_array_pos].range;
					neighbor_range = (pcl_isinf(neighbor_range) ? range_of_current_point : (std::min) (neighbor_range, range_of_current_point));
					top = (std::min) (top, n_y); right = (std::max) (right, n_x); bottom = (std::max) (bottom, n_y); left = (std::min) (left, n_x);
				}
			}
		}

		// The point itself
		int arrayPos = y * width + x;
		float& range_at_image_point = points[arrayPos].range;
		int& counter = counters[arrayPos];
		bool addCurrentPoint = false, replace_with_current_point = false;

		if (counter == 0)
		{
			replace_with_current_point = true;
		}
		else
		{
			if (range_of_current_point > range_at_image_point - noise_level)//选高程最高的
			{
				replace_with_current_point = true;
			}
			else if (fabs(range_of_current_point - range_at_image_point) <= noise_level)
			{
				addCurrentPoint = true;
			}
		}

		if (replace_with_current_point)
		{
			counter = 1;
			range_at_image_point = range_of_current_point;
			top = (std::min) (top, y); right = (std::max) (right, x); bottom = (std::max) (bottom, y); left = (std::min) (left, x);

			difference_x_(y, x) = x_real - x;
			difference_y_(y, x) = y_real - y;
		}
		else if (addCurrentPoint)
		{
			++counter;
			range_at_image_point += (range_of_current_point - range_at_image_point) / counter;

			difference_x_(y, x) = x_real - x;
			difference_y_(y, x) = y_real - y;
		}
	}
}

bool RangeImagePlanar::isInImage(int x, int y) const
{
	return (x >= 0 && x < static_cast<int> (width) && y >= 0 && y < static_cast<int> (height));
}

void RangeImagePlanar::real2DToInt2D(float x, float y, int& xInt, int& yInt) const
{
	xInt = static_cast<int> (pcl_lrintf(x));
	yInt = static_cast<int> (pcl_lrintf(y));
}

void RangeImagePlanar::calculate3DPoint(float image_x, float image_y, float range, pcl::PointWithRange& point) const {
	point.range = range;
	Eigen::Vector3f tmp_point;
	calculate3DPoint(image_x, image_y, range, tmp_point);
	point.x = tmp_point[0];  point.y = tmp_point[1];  point.z = tmp_point[2];
}

void RangeImagePlanar::recalculate3DPointPositions()
{
	for (int y = 0; y < static_cast<int> (height); ++y)
	{
		for (int x = 0; x < static_cast<int> (width); ++x)
		{
			pcl::PointWithRange& point = points[y*width + x];
			if (!std::isinf(point.range))
				calculate3DPoint(static_cast<float> (x), static_cast<float> (y), point.range, point);
		}
	}
}

/////////////////////////////////////////////////////////////////////////
void RangeImagePlanar::calculate3DPoint(float image_x, float image_y, float range, Eigen::Vector3f& point) const
{
	float x = image_x*pixel_size_ - 0.5*format_x_ - x0_ ;
	float y = -image_y*pixel_size_ + 0.5*format_y_ - y0_ ;

	point(0) = location_(EOS_XS) + (range - location_(EOS_ZS))*(to_range_system_(0, 0)*x + to_range_system_(0, 1)*y - to_range_system_(0, 2)*focal_length_) /
		(to_range_system_(2, 0)*x + to_range_system_(2, 1)*y - to_range_system_(2, 2)*focal_length_);

	point(1) = location_(EOS_YS) + (range - location_(EOS_ZS))*(to_range_system_(1, 0)*x + to_range_system_(1, 1)*y - to_range_system_(1, 2)*focal_length_) /
		(to_range_system_(2, 0)*x + to_range_system_(2, 1)*y - to_range_system_(2, 2)*focal_length_);

	point(2) = range;
}

/////////////////////////////////////////////////////////////////////////
inline void RangeImagePlanar::getImagePoint(const Eigen::Vector3f& point, float& image_x, float& image_y, float& range) const
{
	float x = -focal_length_ * (to_range_system_(0, 0)*(point(0) - location_(EOS_XS)) + to_range_system_(1, 0)*(point(1) - location_(EOS_YS)) + to_range_system_(2, 0)*(point(2) - location_(EOS_ZS))) /
		(to_range_system_(0, 2)*(point(0) - location_(EOS_XS)) + to_range_system_(1, 2)*(point(1) - location_(EOS_YS)) + to_range_system_(2, 2)*(point(2) - location_(EOS_ZS)));
	float y = -focal_length_ * (to_range_system_(0, 1)*(point(0) - location_(EOS_XS)) + to_range_system_(1, 1)*(point(1) - location_(EOS_YS)) + to_range_system_(2, 1)*(point(2) - location_(EOS_ZS))) /
		(to_range_system_(0, 2)*(point(0) - location_(EOS_XS)) + to_range_system_(1, 2)*(point(1) - location_(EOS_YS)) + to_range_system_(2, 2)*(point(2) - location_(EOS_ZS)));

	x = x + 0.5*format_x_ + x0_;
	y = -y + 0.5*format_y_ - y0_;

	image_x = x / pixel_size_;
	image_y = y / pixel_size_;
	range = point(2);
}