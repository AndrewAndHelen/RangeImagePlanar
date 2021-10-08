#ifndef RANGEIMAGETOOL_H
#define RANGEIMAGETOOL_H

#include <cmath>
#include <cfloat>
#include <chrono>

#include "Eigen/Dense"
#include "range_image_util.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/eigen.hpp>

using PointRangeType = std::vector<pcl::PointWithRange, Eigen::aligned_allocator<pcl::PointWithRange> >;

bool loadPointCloudFromList(const BACameraIntrinsics& intrinsic,
	const BACamera& extrinsic,
	const std::vector < std::string>& las_list,
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);

void buildRegionMapping(const BACameraIntrinsics& intrinsic,
	const BACamera& extrinsic,
	const BoundingBox3D& box,
	const double grid_length,
	std::vector<int>& point_map);

bool outputSingleRangeImage(const PointRangeType& points_set,
	const uint32_t width, const uint32_t height,
	std::string range_image_path,
	std::string range_coord_path);

class RangeImageTool
{
public:
	using Ptr = std::shared_ptr<RangeImageTool>;
	using ConstPtr = std::shared_ptr<const RangeImageTool> ;

	RangeImageTool() = default;

	~RangeImageTool() = default;

	inline Ptr makeShared() { return Ptr(new RangeImageTool(*this)); }

	bool generateRangeImage(std::string las_path, 
		std::string image_path,
		int thread_num);

	static void generateSingleRangeImage(std::string range_image_path, 
		std::string range_coord_path, 
		BACameraIntrinsics intrinsic, 
		BACamera extrinsic, 
		std::vector < std::string> las_pathes);
protected:
	friend bool loadPointCloudFromList(const BACameraIntrinsics& intrinsic,
		const BACamera& extrinsic,
		const std::vector < std::string>& las_list,
		pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);

	bool isHomologous(const BACameraIntrinsics& intrinsic, 
		const BACamera& extrinsic, 
		const BoundingBox3D& box);

	void caculateImageLasMapping(std::string las_path);

	friend void buildRegionMapping(const BACameraIntrinsics& intrinsic,
		const BACamera& extrinsic,
		const BoundingBox3D& box,
		const double grid_length,
		std::vector<int>& point_map);

	friend bool outputSingleRangeImage(const PointRangeType& points_set,
		const uint32_t width, const uint32_t height,
		std::string range_image_path, 
		std::string range_coord_path);

public:
	std::map<int, std::vector<std::string>> m_mapping;//The index between imageid and lasfile associated 
	std::vector<BACameraIntrinsics> m_intrinsics;
	std::vector<BACamera> m_extrinsics;
};


#endif //RANGEIMAGETOOL_H
