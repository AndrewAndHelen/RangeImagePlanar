#ifdef  _WIN32
#include<io.h>
#include<direct.h>
#else defined linux
#include <sys/io.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#endif

#include "range_image_tool.h"

static void Usage(const char* pszErrorMsg = NULL)
{
	fprintf(stderr, "Usage:\n");
	fprintf(stderr, "RangeImageTool [-cmrPath] *.txt [-phtPath] *,txt [-lasPath] folderPath\n");
	fprintf(stderr, "options:\n");
	fprintf(stderr, "[-help,-h]										[produce help message]\n");
	fprintf(stderr, "[-CmrPath]									[input the absolute path of camera intrinsics file]\n");
	fprintf(stderr, "[-PhtPath]									[input the absolute path of camera extrinsic file]\n");
	fprintf(stderr, "[-GridNum]									[Dense image interpolate windows radius(sugguest 1-4) too big cause memory overflow]\n");
	fprintf(stderr, "[-LasPath]									[input the absolute path of las files corresponding to the photos]\n");
	fprintf(stderr, "[-SavePath]									[output the absolute path of range images]\n");
	fprintf(stderr, "[-ThreadNum]								[the thread nums]\n");
	if (pszErrorMsg != NULL)
		fprintf(stderr, "\nFAILURE: %s\n", pszErrorMsg);

	exit(1);
}

int main(int argc, char** argv)
{
	std::string cmr_path, pht_path, las_path , save_path ;
	int thread_num = 10;
	int ngrid = 16;

	for (int i = 1; i < argc; i++)
	{
		if (strcmp(argv[i], "-help") == 0 || strcmp(argv[i], "-h") == 0)
		{
			Usage();
		}
		else if (strcmp(argv[i], "-CmrPath") == 0)
		{
			i++; if (i >= argc) continue;
			cmr_path = argv[i];
		}
		else if (strcmp(argv[i], "-PhtPath") == 0) {
			i++; if (i >= argc) continue;
			pht_path = argv[i];
		}
		else if (strcmp(argv[i], "-GridNum") == 0) {
			i++; if (i >= argc) continue;
			ngrid = atoi(argv[i]);
		}
		else if (strcmp(argv[i], "-LasPath") == 0) {
			i++; if (i >= argc) continue;
			las_path = argv[i];
		}
		else if (strcmp(argv[i], "-SavePath") == 0) {
			i++; if (i >= argc) continue;
			save_path = argv[i];
		}
		else if (strcmp(argv[i], "-ThreadNum") == 0) {
			i++; if (i >= argc) continue;
			thread_num = atoi(argv[i]);
		}
		else
		{
			Usage("Too many command options.");
		}
	}



	if (!isExist(las_path))
		return 0;

	if (!isExist(save_path))
		createFolder(save_path);

	BACameraFile cameras;
	cameras.m_intrinsics_path = cmr_path;
	cameras.m_extrinsics_path = pht_path;
	//Load camera intrinsics file and extrinsic file 
	if (!cameras.LoadCameraData())
	{
		std::cout << "load camera data is false\n";
		return 0;
	}

	RangeImageTool tool;
	tool.m_intrinsics = cameras.m_intrinsics;
	tool.m_extrinsics = cameras.m_extrinsics;
	tool.generateRangeImage(las_path, save_path,thread_num);


}
