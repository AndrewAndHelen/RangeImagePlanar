#ifndef RANGE_IMAGE_UTILS_H
#define RANGE_IMAGE_UTILS_H

#include "lasreader.hpp"
#include "laswriter.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "range_image_variable.h"
#include "gdal.h"
#include "gdal_priv.h"
#ifdef  _WIN32
#include<io.h>
#include<direct.h>
#else defined linux
#include <sys/io.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#endif

/***********************************************************/
// GCDataType:GDAL和OpenCV数据类型转换的中间格式
// GC_Byte   =======  GDT_Byte   =======  CV_8U  =======  unsigned char
// GC_UInt16 =======  GDT_UInt16 =======  CV_16U =======  unsigned short
// GC_Int16  =======  GDT_Int16  =======  CV_16S =======  short int
// GC_UInt32 =======  GDT_UInt32 =======  缺失   =======  unsigned int
// GC_Int32  =======  GDT_Int32  =======  CV_32S =======  int
// GC_Float32=======  GDT_Float32=======  CV_32F =======  float
// GC_Float64=======  GDT_Float64=======  CV_64F =======  double
/***********************************************************/
enum GCDataType {
	GC_Byte = 0,
	GC_UInt16 = 1,
	GC_Int16 = 2,
	GC_UInt32 = 3,
	GC_Int32 = 4,
	GC_Float32 = 5,
	GC_Float64 = 6,
	GC_ERRType = 7
};

inline bool GDAL2Mat(const std::string inFileName, cv::Mat& img);//读取多通道tiff转为mat

inline bool Mat2GDAL(const std::string outFileName, cv::Mat &img, const int flag = 1);  // Mat文件输出为影像 flag = 默认为1  输出TIFF   另外还支持ENVI 和 ARDAS数据格式

inline GCDataType GDALType2GCType(const GDALDataType ty); // GDAL Type ==========> GDALOpenCV Type

inline GDALDataType GCType2GDALType(const GCDataType ty); //  GDALOpenCV Type ==========> GDAL Type

inline GCDataType OPenCVType2GCType(const int ty); // OPenCV Type ==========> GDALOpenCV Type

inline int GCType2OPenCVType(const GCDataType ty); // GDALOpenCV Type ==========> OPenCV Type

inline void* AllocateMemory(const GCDataType lDataType, const long long lSize);   // 智能分配内存

inline void* SetMemCopy(void *dst, const void *src, const GCDataType lDataType, const long long lSize);

template <typename T>
inline void Distortion(const T& x0, const T& y0,
	const T& k1, const T& k2, const T& k3,
	const T& p1, const T& p2,
	const T& b1, const T& b2,
	const T& obs_x, const T& obs_y,
	T& dltx, T& dlty);

inline Eigen::Matrix3d caculateRotation(double phi, double omega, double kappa);

inline void  caculateImagePoint(const BACameraIntrinsics& intrinsic, const BACamera& extrinsic, const Eigen::Vector3d& point3d, Eigen::Vector2d& point2d);

inline bool isExist(std::string path);

inline bool createFolder(std::string folder_path);

inline bool GetBoundingBox(std::string lasFilePath, BoundingBox3D& box);

inline void GetFiles(std::string path, std::string ext, std::vector<std::string>& files);

inline void PercentLinearTension(cv::Mat& src_image, cv::Mat& dst_image, float low_percent, float high_percent);

/*--------------------------IMPLEMENT----------------------------*/
inline bool GDAL2Mat(const std::string inFileName, cv::Mat& img)
{
	GDALAllRegister();
	GDALDataset *m_poDataSet = (GDALDataset*)GDALOpen(inFileName.c_str(), GA_ReadOnly);
	if (!m_poDataSet)
		return false;

	int m_imgHeigth = m_poDataSet->GetRasterYSize();  // 影像行
	int m_imgWidth = m_poDataSet->GetRasterXSize();  // 影像列
	int m_bandNum = m_poDataSet->GetRasterCount(); // 影像波段数

	GDALRasterBand *pBand = m_poDataSet->GetRasterBand(1);
	GDALDataType gdalTy = pBand->GetRasterDataType();
	GCDataType m_dataType = GDALType2GCType(gdalTy);

	void *pafBuffer = AllocateMemory(m_dataType, m_imgHeigth*m_imgWidth);  // 开辟内存
	std::vector<cv::Mat> channels(m_bandNum);
	cv::Mat *tmpMat = nullptr;

	int iBand = 0; // 波段标记
	while (iBand < m_bandNum)
	{
		pBand = m_poDataSet->GetRasterBand(++iBand);
		pBand->RasterIO(GF_Read, 0, 0, m_imgWidth, m_imgHeigth, pafBuffer, m_imgWidth,
			m_imgHeigth, GCType2GDALType(m_dataType), 0, 0);
		tmpMat = new cv::Mat(m_imgHeigth, m_imgWidth, GCType2OPenCVType(m_dataType), pafBuffer);
		channels.at(iBand - 1) = (*tmpMat).clone();
		delete tmpMat;
		tmpMat = nullptr;
	}
	cv::merge(channels, img);

	delete pafBuffer;   pafBuffer = nullptr;
	GDALClose((GDALDatasetH)m_poDataSet);
}

inline bool Mat2GDAL(const std::string outFileName, cv::Mat &img, const int flag)
{
	if (img.empty())
		return false;

	const int nBandCount = img.channels();
	const int nImgSizeX = img.cols;
	const int nImgSizeY = img.rows;

	std::vector<cv::Mat> channels;
	cv::split(img, channels);

	GDALAllRegister();
	GDALDataset *m_outPoDataSet = nullptr;
	GDALDriver *poDriver;     //驱动，用于创建新的文件
	///flag：1 ====》TIFF
	///      2 ====》HFA
	///      3 ====》ENVI
	std::string pszFormat; //存储数据类型
	switch (flag) {
	case 1:
		pszFormat = "GTiff";
		break;
	case 2:
		pszFormat = "HFA";
		break;
	case 3:
		pszFormat = "ENVI";
		break;
	default:
		return false;
	}

	int OpencvType = channels[0].type();
	GCDataType GCty = OPenCVType2GCType(OpencvType);

	poDriver = GetGDALDriverManager()->GetDriverByName(pszFormat.c_str());
	if (poDriver == nullptr)
		return false;

	m_outPoDataSet = poDriver->Create(outFileName.c_str(), nImgSizeX, nImgSizeY, nBandCount,
		GCType2GDALType(GCty), nullptr);

	/*m_outPoDataSet->SetProjection(m_poDataSet->GetProjectionRef()); //如果有m_poDataSet的tiff可以设置投影
	double dGeotransform[6];
	m_poDataSet->GetGeoTransform(dGeotransform);
	m_outPoDataSet->SetGeoTransform(dGeotransform);*/

	GDALRasterBand *pBand = nullptr;
	void *ppafScan = AllocateMemory(GCty, nImgSizeX*nImgSizeY);
	int n1 = nImgSizeY;
	int nc = nImgSizeX;
	cv::Mat tmpMat;
	for (int i = 1; i <= nBandCount; i++)
	{
		pBand = m_outPoDataSet->GetRasterBand(i);
		tmpMat = channels.at(i - 1);
		if (tmpMat.isContinuous())
			SetMemCopy(ppafScan, (void*)tmpMat.data, GCty, nImgSizeX*nImgSizeY);
		else
			return false;
		CPLErr err = pBand->RasterIO(GF_Write, 0, 0, nImgSizeX, nImgSizeY, ppafScan,
			nImgSizeX, nImgSizeY, GCType2GDALType(GCty), 0, 0);
	}

	delete ppafScan;    ppafScan = nullptr;
	GDALClose((GDALDatasetH)m_outPoDataSet);
	return true;
}

inline GCDataType GDALType2GCType(const GDALDataType ty)
{
	switch (ty)
	{
	case GDT_Byte:
		return GC_Byte;
	case GDT_UInt16:
		return GC_UInt16;
	case GDT_Int16:
		return GC_Int16;
	case GDT_UInt32:
		return GC_UInt32;
	case GDT_Int32:
		return GC_Int32;
	case GDT_Float32:
		return GC_Float32;
	case GDT_Float64:
		return GC_Float64;
	default:
		assert(false);
		return GC_ERRType;
	}
}

inline GDALDataType GCType2GDALType(const GCDataType ty)
{
	switch (ty)
	{
	case GC_Byte:
		return GDT_Byte;
	case GC_UInt16:
		return GDT_UInt16;
	case GC_Int16:
		return GDT_Int16;
	case GC_UInt32:
		return GDT_UInt32;
	case GC_Int32:
		return GDT_Int32;
	case GC_Float32:
		return GDT_Float32;
	case GC_Float64:
		return GDT_Float64;
	default:
		assert(false);
		return GDT_TypeCount;
	}
}

inline GCDataType OPenCVType2GCType(const int ty)
{
	switch (ty)
	{
	case 0:
		return GC_Byte;
	case 2:
		return GC_UInt16;
	case 3:
		return GC_Int16;
	case 4:
		return GC_Int32;
	case 5:
		return GC_Float32;
	case 6:
		return GC_Float64;
	default:
		assert(false);
		return GC_ERRType;
	}
}

inline int GCType2OPenCVType(const GCDataType ty)
{
	switch (ty)
	{
	case GC_Byte:
		return 0;
	case GC_UInt16:
		return 2;
	case GC_Int16:
		return 3;
	case GC_Int32:
		return 4;
	case GC_Float32:
		return 5;
	case GC_Float64:
		return 6;
	default:
		assert(false);
		return -1;
	}
}

inline void* AllocateMemory(const GCDataType lDataType, const long long lSize)
{
	assert(0 != lSize);
	void* pvData = NULL;
	switch (lDataType)
	{
	case GC_Byte:
		pvData = new(std::nothrow) unsigned char[lSize];
		break;
	case GC_UInt16:
		pvData = new(std::nothrow) unsigned short int[lSize];
		break;
	case GC_Int16:
		pvData = new(std::nothrow) short int[lSize];
		break;
	case GC_UInt32:
		pvData = new(std::nothrow) unsigned int[lSize];
		break;
	case GC_Int32:
		pvData = new(std::nothrow) int[lSize];
		break;
	case GC_Float32:
		pvData = new(std::nothrow) float[lSize];
		break;
	case GC_Float64:
		pvData = new(std::nothrow) double[lSize];
		break;
	default:
		assert(false);
		break;
	}
	return pvData;
}

inline void* SetMemCopy(void *dst, const void *src, const GCDataType lDataType, const long long lSize)
{
	assert(0 != lSize);
	switch (lDataType)
	{
	case GC_Byte:
		return memmove(dst, src, sizeof(unsigned char)*lSize);
	case GC_UInt16:
		return memmove(dst, src, sizeof(unsigned short)*lSize);
	case GC_Int16:
		return memmove(dst, src, sizeof(short int)*lSize);
	case GC_UInt32:
		return memmove(dst, src, sizeof(unsigned int)*lSize);
	case GC_Int32:
		return memmove(dst, src, sizeof(int)*lSize);
	case GC_Float32:
		return memmove(dst, src, sizeof(float)*lSize);
	case GC_Float64:
		return memmove(dst, src, sizeof(double)*lSize);
	default:
		return NULL;
	}
}

template <typename T>
inline void Distortion(const T& x0, const T& y0,
	const T& k1, const T& k2, const T& k3,
	const T& p1, const T& p2,
	const T& b1, const T& b2,
	const T& obs_x, const T& obs_y,
	T& dltx, T& dlty)
{
	T r2 = (obs_x - x0) * (obs_x - x0) + (obs_y - y0) * (obs_y - y0);
	T r4 = r2 * r2;
	T r6 = r4 * r2;
	T radial = k1 * r2 + k2 * r4 + k3 * r6;
	T ux = p1 * (r2 + T(2.0) * (obs_x - x0) * (obs_x - x0)) + T(2.0)*p2*(obs_x - x0)*(obs_y - y0);
	T uy = p2 * (r2 + T(2.0) * (obs_y - y0) * (obs_y - y0)) + T(2.0)*p1*(obs_x - x0)*(obs_y - y0);

	dltx = (obs_x - x0)*radial + ux + b1 * (obs_x - x0) + b2 * (obs_y - y0);
	dlty = (obs_y - y0)*radial + uy;
}

inline Eigen::Matrix3d caculateRotation(double phi,double omega,double kappa)
{
	Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();

	rotation(0, 0) = std::cos(phi)*std::cos(kappa) - std::sin(phi)*std::sin(omega)*std::sin(kappa);//a1
	rotation(0, 1) = -std::cos(phi)* std::sin(kappa) - std::sin(phi)* std::sin(omega)* std::cos(kappa);//a2
	rotation(0, 2) = -std::sin(phi)* std::cos(omega);//a3

	rotation(1, 0) = std::cos(omega)* std::sin(kappa);//b1
	rotation(1, 1) = std::cos(omega)* std::cos(kappa);//b2
	rotation(1, 2) = -std::sin(omega);//b3

	rotation(2, 0) = std::sin(phi)* std::cos(kappa) + std::cos(phi)* std::sin(omega)* std::sin(kappa);//c1
	rotation(2, 1) = -std::sin(phi)* std::sin(kappa) + std::cos(phi)* std::sin(omega)* std::cos(kappa);//c2
	rotation(2, 2) = std::cos(phi)* std::cos(omega);//c3

	return rotation;
}

inline void caculateImagePoint(const BACameraIntrinsics& intrinsic, const BACamera& extrinsic, const Eigen::Vector3d& point3d, Eigen::Vector2d& point2d)
{
	Eigen::Matrix3d rotation = caculateRotation(extrinsic.Posture(EOS_PHI), extrinsic.Posture(EOS_OMG), extrinsic.Posture(EOS_KAP));
	Eigen::Vector3d location = extrinsic.Location;

	double format_x = intrinsic.format_x;
	double format_y = intrinsic.format_y;
	double x0 = intrinsic.intrins(OFFSET_X0);
	double y0 = intrinsic.intrins(OFFSET_Y0);
	double f0 = intrinsic.intrins(OFFSET_F0);

	//转成y向上的右手系
	double x = -f0 * (rotation(0, 0)*(point3d(0) - location(EOS_XS)) + rotation(1, 0)*(point3d(1) - location(EOS_YS)) + rotation(2, 0)*(point3d(2) - location(EOS_ZS))) /
		(rotation(0, 2)*(point3d(0) - location(EOS_XS)) + rotation(1, 2)*(point3d(1) - location(EOS_YS)) + rotation(2, 2)*(point3d(2) - location(EOS_ZS)));
	double y = -f0 * (rotation(0, 1)*(point3d(0) - location(EOS_XS)) + rotation(1, 1)*(point3d(1) - location(EOS_YS)) + rotation(2, 1)*(point3d(2) - location(EOS_ZS))) /
		(rotation(0, 2)*(point3d(0) - location(EOS_XS)) + rotation(1, 2)*(point3d(1) - location(EOS_YS)) + rotation(2, 2)*(point3d(2) - location(EOS_ZS)));

	//y为下，左上角的右手系
	x = x + 0.5*format_x + intrinsic.intrins(OFFSET_X0);
	y = -y + 0.5*format_y - intrinsic.intrins(OFFSET_Y0);

	point2d(0) = x / intrinsic.pixelsize;
	point2d(1) = y / intrinsic.pixelsize;
}

inline bool isExist(std::string path)
{
#ifdef _WIN32
	if (_access(path.c_str(), 0) == 0)
		return true;
	else
		return false;
#else defined linux
	if (access(path.c_str(), 0) == 0)
		return true;
	else
		return false;
#endif
}

inline bool createFolder(std::string folder_path)
{
#ifdef _WIN32
	_mkdir(folder_path.c_str());
#else defined linux
	mkdir(folder_path.c_str(), S_IRWXU | S_IRWXG);
#endif
	return true;
}

inline bool GetBoundingBox(std::string lasFilePath, BoundingBox3D& box)
{
	LASreadOpener lasreadopener;
	LASheader las_header_read;
	LASreader *lasreader;

	// Open the LAS file
	lasreadopener.set_file_name(lasFilePath.c_str());

	if (!lasreadopener.active()) {
		std::cout << "ERROR: no input specified" << std::endl;
		return false;
	}

	lasreader = lasreadopener.open();
	if (!lasreader) {
		std::cerr << "ERROR: could not open LAS file: " << lasFilePath << std::endl;
		return false;
	}

	las_header_read = lasreader->header;
	las_header_read.user_data_after_header = nullptr;

	box.min_x = las_header_read.min_x;
	box.min_y = las_header_read.min_y;
	box.min_z = las_header_read.min_z;
	box.max_x = las_header_read.max_x;
	box.max_y = las_header_read.max_y;
	box.max_z = las_header_read.max_z;

	// Close the LAS file
	lasreader->close();

	return true;
}

inline void GetFiles(std::string path, std::string ext, std::vector<std::string>& files)
{
	long long hFile = 0;

	struct _finddata_t fileinfo;
	std::string tmp_path;
	if ((hFile = _findfirst(tmp_path.assign(path).append("/*.").append(ext).c_str(), &fileinfo)) != -1)
	{
		do
		{
			if (fileinfo.attrib & _A_SUBDIR)
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					GetFiles(tmp_path.assign(path).append("/").append(fileinfo.name), ext, files);
			}
			else
			{
				files.push_back(tmp_path.assign(path).append("/").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);

		_findclose(hFile);
	}
}

inline void PercentLinearTension(cv::Mat& src_image, cv::Mat& dst_image, float low_percent, float high_percent)
{
	double src_min, src_max;
	float dst_min = 0, dst_max = 255;

	if (src_image.type() != 5)
		src_image.convertTo(dst_image, CV_32FC1);
	else
		src_image.copyTo(dst_image);

	cv::Mat no_zero_mask = src_image != 0;
	cv::minMaxLoc(src_image, &src_min, &src_max, 0, 0, no_zero_mask);

	float src_pixel_width = src_max - src_min;
	float src_low_value = src_min + src_pixel_width * low_percent;
	float src_high_value = src_max - src_pixel_width * low_percent;

	cv::Mat low_region = dst_image <= src_low_value;
	cv::Mat high_region = dst_image >= src_high_value;
	cv::Mat zero_region = src_image == 0;

	float k = (dst_max - dst_min) / (src_max - src_min);
	float b = (src_max*dst_min - dst_max * src_min) / (src_max - src_min);

	dst_image = k * dst_image + b;
	dst_image.setTo(1, low_region);
	dst_image.setTo(dst_max, high_region);
	dst_image.setTo(0, zero_region);

	dst_image.convertTo(dst_image, CV_8UC1);
	//cv::equalizeHist(dst_image, dst_image);
}

#endif 
