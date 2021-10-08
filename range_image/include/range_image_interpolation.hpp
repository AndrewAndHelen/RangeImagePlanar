#ifndef RANGE_IMAGE_INTERPOLATION_HPP
#define RANGE_IMAGE_INTERPOLATION_HPP

#include "Eigen/Dense"
#include <vector>

static void matrixInverse(Eigen::MatrixXf& matrix);

/** \brief Calculate the dense range image from sparse range image using window weighted interpolation
		*\param difference_x a matrix that stores the distance between each pixel and the real point in the x direction
		*\param difference_y a matrix that stores the distance between each pixel and the real point in the y direction
		*\param sparse_range_image origin range image
		*\param dense_range_image ouput interpolation range image
		*/
inline void getDenseRangeImage(Eigen::MatrixXf& difference_x, Eigen::MatrixXf& difference_y, int grid, Eigen::MatrixXf& sparse_range_image, Eigen::MatrixXf& dense_range_image);

/** \brief Calculate the dense range image from sparse range image using window weighted interpolation(Non-global matrix operations)
		*\param difference_x a matrix that stores the distance between each pixel and the real point in the x direction
		*\param difference_y a matrix that stores the distance between each pixel and the real point in the y direction
		*\param sparse_range_image origin range image
		*\param dense_range_image ouput interpolation range image
		*/
inline void getDenseRangeImage2(Eigen::MatrixXf& difference_x, Eigen::MatrixXf& difference_y, int grid, Eigen::MatrixXf& sparse_range_image, Eigen::MatrixXf& dense_range_image);

inline void getDenseRangeImage(Eigen::MatrixXf& difference_x, Eigen::MatrixXf& difference_y, int grid, Eigen::MatrixXf& sparse_range_image, Eigen::MatrixXf& dense_range_image)
{
	int windows_size = 2 * grid + 1;

	int rows = difference_x.rows();
	int cols = difference_x.cols();

	int dense_rows = rows - windows_size + 1;
	int dense_cols = cols - windows_size + 1;

	std::vector<std::vector<Eigen::MatrixXf>> KmX(windows_size);
	std::vector<std::vector<Eigen::MatrixXf>> KmY(windows_size);
	std::vector<std::vector<Eigen::MatrixXf>> KmD(windows_size);

	Eigen::MatrixXf tmp = Eigen::MatrixXf::Zero(dense_rows, dense_cols);
	for (int i = 0; i < windows_size; ++i)
		for (int j = 0; j < windows_size; ++j)
		{
			tmp.fill(i - grid);
			Eigen::MatrixXf KmX_ij = difference_x.block(i, j, dense_rows, dense_cols) - tmp;
			KmX[i].emplace_back(KmX_ij);

			tmp.fill(j - grid);
			Eigen::MatrixXf KmY_ij = difference_y.block(i, j, dense_rows, dense_cols) - tmp;
			KmY[i].emplace_back(KmY_ij);

			Eigen::MatrixXf KmD_ij = sparse_range_image.block(i, j, dense_rows, dense_cols);
			KmD[i].emplace_back(KmD_ij);
		}

	Eigen::MatrixXf sum_range = Eigen::MatrixXf::Zero(dense_rows, dense_cols);
	Eigen::MatrixXf sum_distance = Eigen::MatrixXf::Zero(dense_rows, dense_cols);

	Eigen::MatrixXf  tmp_distance;

	for (int i = 0; i < windows_size; ++i)
		for (int j = 0; j < windows_size; ++j)
		{
			tmp_distance = KmX[i][j].cwiseProduct(KmX[i][j]) + KmY[i][j].cwiseProduct(KmY[i][j]);
			tmp_distance = tmp_distance.cwiseSqrt();//sqrt()

			matrixInverse(tmp_distance);				//1./

			sum_range = sum_range + tmp_distance.cwiseProduct(KmD[i][j]);
			sum_distance = sum_distance + tmp_distance;
		}

	KmX.erase(KmX.begin(), KmX.end());
	KmY.erase(KmY.begin(), KmY.end());
	KmD.erase(KmD.begin(), KmD.end());

	auto mask = sum_distance.array() == 0;
	sum_distance = sum_distance + mask.matrix().cast<float>();

	dense_range_image = Eigen::MatrixXf::Zero(rows, cols);
	dense_range_image.block(grid, grid, dense_rows, dense_cols) = sum_range.cwiseQuotient(sum_distance);
}

inline void getDenseRangeImage2(Eigen::MatrixXf& difference_x, Eigen::MatrixXf& difference_y, int grid, Eigen::MatrixXf& sparse_range_image, Eigen::MatrixXf& dense_range_image)
{
	int windows_size = 2 * grid + 1;

	int rows = difference_x.rows();
	int cols = difference_x.cols();

	int dense_rows = rows - windows_size + 1;
	int dense_cols = cols - windows_size + 1;

	Eigen::MatrixXf template_x_direction = Eigen::MatrixXf::Zero(windows_size, windows_size);
	Eigen::MatrixXf template_y_direction = Eigen::MatrixXf::Zero(windows_size, windows_size);

	//build template

	for (int i = 0; i < windows_size; ++i)
		for (int j = 0; j < windows_size; ++j)
		{
			template_x_direction(i, j) = i - grid;
			template_y_direction(i, j) = j - grid;
		}

	dense_range_image = Eigen::MatrixXf::Zero(rows, cols);

	Eigen::MatrixXf region_range, region_x, region_y, region_distance, sum_range;
	for (int i = grid; i < grid + dense_rows - 1; ++i)
		for (int j = grid; j < grid + dense_cols - 1; ++j)
		{
			region_range = sparse_range_image.block(i - grid, j - grid, windows_size, windows_size);
			region_x = difference_x.block(i - grid, j - grid, windows_size, windows_size) + template_x_direction;
			region_y = difference_y.block(i - grid, j - grid, windows_size, windows_size) + template_y_direction;

			region_distance = (region_x.cwiseProduct(region_x) + region_y.cwiseProduct(region_y)).cwiseSqrt();
			//region_distance = region_distance.cwiseSqrt();
			matrixInverse(region_distance);

			sum_range = region_range.cwiseProduct(region_distance);

			auto mask = region_distance.array() == 0;
			region_distance = region_distance + mask.matrix().cast<float>();

			dense_range_image(i, j) = sum_range.sum() / region_distance.sum();
		}
}

static void matrixInverse(Eigen::MatrixXf& matrix)
{
	Eigen::Index rows = matrix.rows();
	Eigen::Index cols = matrix.cols();

	for (int i = 0; i < rows; ++i)
	{
		for (int j = 0; j < cols; ++j)
		{
			if (matrix(i, j) != 0)
				matrix(i, j) = 1 / matrix(i, j);
		}
	}
}
#endif
