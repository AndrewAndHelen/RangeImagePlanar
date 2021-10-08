#ifndef RANGE_IMAGE_VARIABLE_H
#define RANGE_IMAGE_VARIABLE_H

#include <vector>
#include <string>
#include <string>
#include <fstream>
#include <iostream>
#include <map>
#include "Eigen/Core"

#define PER_IOS_BLOCK_NUM 10

enum OFFSET{
	OFFSET_F0,
	OFFSET_X0,
	OFFSET_Y0,
	OFFSET_K1,
	OFFSET_K2,
	OFFSET_K3,
	OFFSET_P1,
	OFFSET_P2,
	OFFSET_B1,
	OFFSET_B2
};

enum EOS_LOCATION
{
	EOS_XS = 0,
	EOS_YS = 1,
	EOS_ZS = 2
};

enum EOS_POSTURE
{
	EOS_PHI = 0,
	EOS_OMG = 1,
	EOS_KAP = 2
};

struct BACameraIntrinsics
{
	BACameraIntrinsics() {}
	BACameraIntrinsics(const BACameraIntrinsics& intrinsics) :
		intrins(intrinsics.intrins) {
		format_x = intrinsics.format_x;
		format_y = intrinsics.format_y;
		pixelsize = intrinsics.pixelsize;
	}
	double format_x, format_y, pixelsize; //ibundle内部接口,单位为mm
	Eigen::Matrix<double, PER_IOS_BLOCK_NUM, 1> intrins;
};

struct BACamera
{
	BACamera() : imageid(-1), intrinsics_group(0) { }
	BACamera(const BACamera& c) :
		imageid(c.imageid), intrinsics_group(c.intrinsics_group),
		Location(c.Location), Posture(c.Posture),
		nStripID(c.nStripID), Attrib(c.Attrib), bFlag(c.bFlag),
		BlockID(c.BlockID), nCamera(c.nCamera) { }

	int imageid;
	int intrinsics_group; //内参数组别，适用于多相机,默认一个
	Eigen::Vector3d Location;
	Eigen::Vector3d Posture; //phi, omg, kap
	int nStripID, Attrib, bFlag, BlockID, nCamera; //iBundle接口
};


struct BoundingBox3D
{
	BoundingBox3D() { }
	BoundingBox3D(const BoundingBox3D& other):
		min_x(other.min_x), min_y(other.min_y), min_z(other.min_z),
		max_x(other.max_x), max_y(other.max_y), max_z(other.max_z){ }

	double min_x, min_y, min_z;
	double max_x, max_y, max_z;
};

class BACameraFile
{
public:
	// =====TYPEDEFS=====
	typedef std::shared_ptr<BACameraFile> Ptr;
	typedef std::shared_ptr<const BACameraFile> ConstPtr;

	// =====CONSTRUCTOR & DESTRUCTOR=====
	 /** Constructor */
	BACameraFile()
	{

	};
	/** Destructor */
	~BACameraFile() { ReleaseData(); }

	// =====PUBLIC METHODS=====
	/** \brief Get a boost shared pointer of a copy of this */
	bool LoadCameraData()
	{
		if (LoadCmr(m_intrinsics_path, m_intrinsics) && LoadPht(m_extrinsics_path, m_extrinsics))
			return true;
		else
			return false;
	}

	void ReleaseData()
	{
		if (m_intrinsics.size() != 0)        m_intrinsics.~vector();
		if (m_extrinsics.size() != 0)           m_extrinsics.~vector();
	}

	bool LoadCmr(const std::string m_intrinsics_path, std::vector<BACameraIntrinsics> &m_intrinsics)
	{
		std::ifstream fp(m_intrinsics_path);
		if (!fp) {
			std::cout << "Couldn't find camera intrinsic file:" << m_intrinsics_path << std::endl;
			return false;
		}
		std::string buf;
		std::getline(fp, buf);
		std::getline(fp, buf);
		std::getline(fp, buf);
		std::getline(fp, buf);

		int ncmr;
		sscanf(buf.c_str(), "%d", &ncmr);
		m_intrinsics.resize(ncmr);

		double x0, y0, f, formatx, formaty, pixelsize, k0, k1, k2, k3, p1, p2, b1, b2;
		int cmridx, attrib;

		int tmp;
		for (int i = 0; i < ncmr; i++)
		{
			std::getline(fp, buf);
			tmp = sscanf(buf.c_str(), "%d%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%d", &cmridx,
				&x0, &y0, &f, &formatx, &formaty, &pixelsize,
				&k0, &k1, &k2, &k3, &p1, &p2, &b1, &b2, &attrib);

			if (tmp == 16) {
				m_intrinsics[i].intrins(OFFSET_F0) = f;
				m_intrinsics[i].intrins(OFFSET_X0) = x0;
				m_intrinsics[i].intrins(OFFSET_Y0) = y0;
				m_intrinsics[i].intrins(OFFSET_K1) = k1;
				m_intrinsics[i].intrins(OFFSET_K2) = k2;
				m_intrinsics[i].intrins(OFFSET_K3) = k3;
				m_intrinsics[i].intrins(OFFSET_P1) = p1;
				m_intrinsics[i].intrins(OFFSET_P2) = p2;
				m_intrinsics[i].intrins(OFFSET_B1) = b1;
				m_intrinsics[i].intrins(OFFSET_B2) = b2;
			}
			m_intrinsics[i].format_x = formatx;
			m_intrinsics[i].format_y = formaty;
			m_intrinsics[i].pixelsize = pixelsize;

		}

		return true;
	}

	bool LoadPht(const std::string m_extrinsic_path, std::vector<BACamera> &m_extrinsics)
	{
		std::ifstream fp(m_extrinsic_path);
		if (!fp) {
			std::cout << "Couldn't find camera extrinsic file:" << m_extrinsic_path << std::endl;
			return false;
		}

		std::string buf;
		std::getline(fp, buf);
		std::getline(fp, buf);
		std::getline(fp, buf);
		std::getline(fp, buf);

		int nimg;
		sscanf(buf.c_str(), "%d", &nimg);

		m_extrinsics.resize(nimg);

		int ImageID, StripID, Attrib, CameraID, bFlag, BlockID;
		double Xs, Ys, Zs, phi, omega, kappa;

		for (int i = 0; i < nimg; i++)
		{
			std::getline(fp, buf);
			sscanf(buf.c_str(), "%d%lf%lf%lf%lf%lf%lf%d%d%d%d%d", &ImageID,
				&Xs, &Ys, &Zs, &phi, &omega, &kappa,
				&StripID, &Attrib, &CameraID, &bFlag, &BlockID);

			m_extrinsics[i].Location(EOS_XS) = Xs;
			m_extrinsics[i].Location(EOS_YS) = Ys;
			m_extrinsics[i].Location(EOS_ZS) = Zs;
			m_extrinsics[i].Posture(EOS_PHI) = phi;
			m_extrinsics[i].Posture(EOS_OMG) = omega;
			m_extrinsics[i].Posture(EOS_KAP) = kappa;
			m_extrinsics[i].imageid = i;
			m_extrinsics[i].intrinsics_group = CameraID;

			m_imageidx_map.insert(std::pair<int, int>(ImageID, i));  //便于pts快速索引

			m_extrinsics[i].nStripID = StripID;
			m_extrinsics[i].nCamera = CameraID;
			m_extrinsics[i].Attrib = Attrib;
			m_extrinsics[i].bFlag = bFlag;
			m_extrinsics[i].BlockID = BlockID;
		}

		return true;
	}

	std::string m_intrinsics_path;
	std::string m_extrinsics_path;

	std::vector<BACameraIntrinsics> m_intrinsics;
	std::vector<BACamera> m_extrinsics;

	std::map<int, int> m_imageidx_map;  //IBundle中imagID->imgid(从0开始)
};
#endif
