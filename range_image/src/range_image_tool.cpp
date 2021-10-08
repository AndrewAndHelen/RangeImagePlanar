#include "range_image_tool.h"
#include "range_image_interpolation.hpp"
#include "range_image_planar.h"
#include "threadpool.h"
#include <functional>
bool writePointCloudToLas(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string las_path)
{
	LASwriteOpener laswriteopener;
	LASheader            las_header_write;
	LASwriter*           laswriter;
	laswriteopener.set_file_name(las_path.c_str());

	las_header_write.x_scale_factor = 0.0001;
	las_header_write.y_scale_factor = 0.0001;
	las_header_write.z_scale_factor = 0.0001;
	las_header_write.point_data_format = 3;
	las_header_write.point_data_record_length = 34;

	if (!laswriteopener.active()) {
		std::cerr << "Error: could not write las file: " << las_path << std::endl;
		return false;
	}

	laswriteopener.set_format(LAS_TOOLS_FORMAT_LAS);

	laswriter = laswriteopener.open(&las_header_write);
	if (!laswriter) {
		return false;
	}

	LASpoint laspoint_w;
	if (!laspoint_w.init(&las_header_write, las_header_write.point_data_format, las_header_write.point_data_record_length, 0)) {
		return false;
	}

	// write points
	double minX = DBL_MAX, minY = DBL_MAX, minZ = DBL_MAX;
	double maxX = -DBL_MAX, maxY = -DBL_MAX, maxZ = -DBL_MAX;

	for (auto point : cloud->points)
	{
		laspoint_w.set_x(point.x);
		laspoint_w.set_y(point.y);
		laspoint_w.set_z(point.z);

		laswriter->write_point(&laspoint_w);
		laswriter->update_inventory(&laspoint_w);

		
		if (point.x < minX) minX = point.x;
		if (point.x > maxX) maxX = point.x;
		if (point.y < minY) minY = point.y;
		if (point.y > maxY) maxY = point.y;
		if (point.z < minZ) minZ = point.z;
		if (point.z > maxZ) maxZ = point.z;

	}

	las_header_write.set_bounding_box(minX, minY, minZ, maxX, maxY, maxZ);

	laswriter->update_header(&las_header_write, true);

	laswriter->close();
	delete laswriter;
	laswriter = nullptr;

	return true;
}

// =====PUBLIC METHODS=====
/////////////////////////////////////////////////////////////////////////
bool RangeImageTool::generateRangeImage(std::string las_path, std::string image_path, int thread_num)
{
	if (m_intrinsics.empty() || m_extrinsics.empty())
		return false;

	if (!isExist(image_path))
		createFolder(image_path);

	caculateImageLasMapping(las_path);

	threadpool pool(thread_num);
	std::vector<std::future<void>> results;

	for (int i = 0; i < m_extrinsics.size(); i++)
	{
		BACamera& extrinsic = m_extrinsics[i];
		int imageid = extrinsic.imageid;
		int intrinsics_group = extrinsic.intrinsics_group;
		BACameraIntrinsics& intrinsic = m_intrinsics[intrinsics_group];

		std::vector < std::string>& las_pathes = m_mapping[imageid];

		if (!las_pathes.empty())
		{
			std::string range_image_path = image_path + "/" + std::to_string(imageid) + "_dep.jpg";
			std::string range_coord_path = image_path + "/" + std::to_string(imageid) + "_cor.tif";
			results.emplace_back(pool.commit(std::bind(RangeImageTool::generateSingleRangeImage,
				range_image_path, range_coord_path, intrinsic, extrinsic, las_pathes)));
			//RangeImageTool::generateSingleRangeImage(range_image_path, range_coord_path, intrinsic, extrinsic, las_pathes);
		}
	}
	for (auto&& result : results)
		result.get();

	return true;
}

/////////////////////////////////////////////////////////////////////////
void RangeImageTool::generateSingleRangeImage(std::string range_image_path, std::string range_coord_path, BACameraIntrinsics intrinsic, BACamera extrinsic, std::vector < std::string> las_pathes)
{
	auto start_time = std::chrono::high_resolution_clock::now();
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	loadPointCloudFromList(intrinsic, extrinsic, las_pathes, point_cloud);
	//writePointCloudToLas(point_cloud, "0.las");
	float pixel_size = intrinsic.pixelsize;
	int format_x = static_cast<int>(intrinsic.format_x / pixel_size);
	int format_y = static_cast<int>(intrinsic.format_y / pixel_size);

	float x0 = static_cast<float>(intrinsic.intrins[OFFSET_X0] / pixel_size);
	float y0 = static_cast<float>(intrinsic.intrins[OFFSET_Y0] / pixel_size);
	float f0 = static_cast<float>(intrinsic.intrins[OFFSET_F0]);

	//Photogrammetry defination
	float cx = static_cast<float>(format_x *0.5) + x0;
	float cy = static_cast<float>(format_y *0.5) + y0;

	RangeImagePlanar range_image_planar;
	range_image_planar.createFromPointCloudWithFixedSize(*point_cloud, intrinsic, extrinsic);

	PointRangeType& points_set = range_image_planar.points;
	uint32_t width = range_image_planar.width;
	uint32_t height = range_image_planar.height;
	outputSingleRangeImage(points_set, width, height, range_image_path, range_coord_path);

	auto end_time = std::chrono::high_resolution_clock::now();
	auto cost_time = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
}

// =====PROTECT METHODS=====

/////////////////////////////////////////////////////////////////////////
void RangeImageTool::caculateImageLasMapping(std::string las_path)
{
	std::vector<std::string> las_list;
	std::map<std::string, BoundingBox3D> las_box_map;

	GetFiles(las_path, "las", las_list);

	BoundingBox3D tmp;
	for (auto lasfile_path : las_list)
	{
		GetBoundingBox(lasfile_path, tmp);
		las_box_map.insert(std::pair<std::string, BoundingBox3D>(lasfile_path, tmp));
	}
	
	for (int i = 0; i < m_extrinsics.size(); i++)
	{
		std::vector<std::string> contain_las;
		int imageid = m_extrinsics[i].imageid;
		int intrinsics_group = m_extrinsics[i].intrinsics_group;

		for (auto lasfile_path : las_list)
		{
			if (isHomologous(m_intrinsics[intrinsics_group], m_extrinsics[i], las_box_map[lasfile_path]))
				contain_las.push_back(lasfile_path);
		}
		m_mapping.insert(std::pair<int, std::vector<std::string>>(imageid, contain_las));

	}
}

/////////////////////////////////////////////////////////////////////////
bool RangeImageTool::isHomologous(const BACameraIntrinsics& intrinsic, const BACamera& extrinsic, const BoundingBox3D& box)
{
	const int sampleSize = 3;

	double format_x = intrinsic.format_x / intrinsic.pixelsize;
	double format_y = intrinsic.format_y / intrinsic.pixelsize;

	double pcLength = box.max_x - box.min_x;
	double pcWidth = box.max_y - box.min_y;

	double pcLengthInternal = pcLength / sampleSize;
	double pcWidthInternal = pcWidth / sampleSize;

	Eigen::Vector2d point2d;
	double tmp_x, tmp_y, tmp_z;
	tmp_z = box.min_z;

	for (int i = 0; i <= sampleSize; ++i)
	{
		for (int j = 0; j <= sampleSize; ++j)
		{
			tmp_x = box.min_x + i * pcLengthInternal;
			tmp_y = box.min_y + j * pcWidthInternal;
			Eigen::Vector3d point3d(tmp_x, tmp_y, tmp_z);

			caculateImagePoint(intrinsic, extrinsic, point3d, point2d);
			if ((point2d[0] >= 0 && point2d[0] < format_x && point2d[1] >= 0 && point2d[1] < format_y))
				return true;
		}
	}
	return false;
}

/////////////////////////////////////////////////////////////////////////
bool loadPointCloudFromList(const BACameraIntrinsics& intrinsic,
	const BACamera& extrinsic,
	const std::vector < std::string>& las_list,
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
	const double grid_length = 10;//默认格网长度=10m

	pcl::PointXYZ point_tmp;
	BoundingBox3D box;
	std::vector<int> point_map;
	for (auto las_path : las_list)
	{
		GetBoundingBox(las_path, box);
		double box_width = box.max_x - box.min_x;
		double box_height = box.max_y - box.min_y;

		buildRegionMapping(intrinsic, extrinsic, box, grid_length, point_map);

		int grid_nx = static_cast<int>(std::floor(box_width / grid_length)) + 1;
		int grid_ny = static_cast<int>(std::floor(box_height / grid_length)) + 1;

		LASreadOpener lasreadopener;
		LASheader las_header_read;
		LASreader *lasreader;

		lasreadopener.set_file_name(las_path.c_str());

		if (!lasreadopener.active()) {
			std::cout << "ERROR: no input specified" << std::endl;
			continue;
		}

		lasreader = lasreadopener.open();
		if (!lasreader) {
			std::cout << "ERROR: could not open LAS file: " << las_path << std::endl;
			continue;
		}

		las_header_read = lasreader->header;
		las_header_read.user_data_after_header = nullptr;
		U8 point_type = las_header_read.point_data_format;
		U16 point_size = las_header_read.point_data_record_length;

		LASpoint point_r;
		point_r.init(&las_header_read, point_type, point_size, &las_header_read);

		while (lasreader->read_point())
		{
			point_r = lasreader->point;

			point_tmp.x = static_cast<float>(point_r.get_x());
			point_tmp.y = static_cast<float>(point_r.get_y());
			point_tmp.z = static_cast<float>(point_r.get_z());

			int temp_y = static_cast<int>(std::floor((point_tmp.y - box.min_y) / grid_length));
			int temp_x = static_cast<int>(std::floor((point_tmp.x - box.min_x) / grid_length));

			if (temp_y < 0 || temp_x < 0)
				continue;

			if (point_map[temp_y*grid_nx + temp_x] > 0)
				point_cloud->points.push_back(point_tmp);
		}
		lasreader->close();
	}

	point_cloud->width = point_cloud->points.size();
	point_cloud->height = 1;

	return true;
}

/////////////////////////////////////////////////////////////////////////
void buildRegionMapping(const BACameraIntrinsics& intrinsic,
	const BACamera& extrinsic,
	const BoundingBox3D& box,
	const double grid_length,
	std::vector<int>& point_map)
{
	const int sample_size = 5;//范围1-10
	const float internal = 1.0 / sample_size;

	double format_x = intrinsic.format_x / intrinsic.pixelsize;
	double format_y = intrinsic.format_y / intrinsic.pixelsize;

	double box_width = box.max_x - box.min_x;
	double box_height = box.max_y - box.min_y;

	int grid_nx = static_cast<int>(std::floor(box_width / grid_length)) + 1;
	int grid_ny = static_cast<int>(std::floor(box_height / grid_length)) + 1;

	point_map.resize(grid_ny * grid_nx, 0);

	Eigen::Vector3d point3d;
	Eigen::Vector2d point2d;

	for (int i = 0; i < grid_ny; ++i)
	{
		for (int j = 0; j < grid_nx; ++j)
		{
			for (int k = 1; k <= sample_size; ++k)
			{
				point3d(0) = box.min_x + (j + internal *k) * grid_length > box.max_x ? box.max_x : box.min_x + (j + internal *k) * grid_length;
				point3d(1) = box.min_y + (i + internal *k) * grid_length > box.max_y ? box.max_y : box.min_y + (i + internal *k) * grid_length;
				point3d(2) = box.min_z;

				caculateImagePoint(intrinsic, extrinsic, point3d, point2d);

				if ((point2d[0] >= 0 && point2d[0] < format_x && point2d[1] >= 0 && point2d[1] < format_y))
				{
					point_map[i*grid_nx + j] = 1;
					continue;
				}
					
			}

		}
	}
}

/////////////////////////////////////////////////////////////////////////
bool outputSingleRangeImage(const PointRangeType& points_set,
	const uint32_t width, const uint32_t height,
	std::string range_image_path,
	std::string range_coord_path)
{
	const int multi_factor = 2;

	cv::Mat range_coord = cv::Mat::zeros(height, width, CV_32FC3);
	for (int y = 0; y < height; ++y)
	{
		cv::Vec3f* ptr_range_coord = range_coord.ptr<cv::Vec3f>(y);
		for (int x = 0; x < width; ++x)
		{
			const pcl::PointWithRange& point = points_set[y*width + x];
			if (!std::isinf(point.range))
				ptr_range_coord[x] = cv::Vec3f(point.x, point.y, point.z);
		}
	}

	std::vector<cv::Mat> channels;
	cv::split(range_coord, channels);

	cv::Mat strech_range_img;
	PercentLinearTension(channels[2], strech_range_img, 0, 0);
	cv::imwrite(range_image_path, strech_range_img*multi_factor);

	Mat2GDAL(range_coord_path, range_coord);

	return true;
}