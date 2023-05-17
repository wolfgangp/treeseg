#include <iostream>
#include <vector>
#include <sstream>
#include <fstream>
#include <algorithm>

// #include <dirent.h>
#include "treeseg.h"

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <boost/algorithm/string.hpp>

/**
int main(int argc, char **argv)
{
	std::vector<std::string> args(argv+1,argv+argc);
	std::string matrix_dir = args[0];
	if(matrix_dir[matrix_dir.length()-1] != '/') matrix_dir = matrix_dir + '/';
	std::vector<std::string> fnames;
	char buf[PATH_MAX + 1];
	DIR *dir = NULL;
	struct dirent *drnt = NULL;
	dir = opendir(matrix_dir.c_str());
	while(drnt = readdir(dir)) fnames.push_back(drnt->d_name);
	closedir(dir);
	std::vector<float> x,y;
	for(int i=0;i<fnames.size();i++)
	{
		std::stringstream ss;
		ss << matrix_dir << fnames[i];
		std::string fname = ss.str();
		if(fname[fname.length()-4] == '.' && fname[fname.length()-3] == 'd')
		{
			float matrix[16];
			std::fstream mfile;
			mfile.open(fname);
			int no_count = 0;
			if(mfile.is_open())
			{
				while(!mfile.eof())
				{
					mfile >> matrix[no_count];
					no_count++;
				}
			}
			x.push_back(matrix[3]);
			y.push_back(matrix[7]);
		}
	}
	float x_sum = 0;
	float y_sum = 0;
	for(int j=0;j<x.size();j++)
	{
		x_sum += x[j];
		y_sum += y[j];
	}
//	float x_mean = x_sum / x.size();
//	float y_mean = y_sum / y.size();
	auto xmm = std::minmax_element(x.begin(),x.end());
	auto ymm = std::minmax_element(y.begin(),y.end());
	float x_min = x[xmm.first-x.begin()];
	float x_max = x[xmm.second-x.begin()];
	float y_min = y[ymm.first-y.begin()];
	float y_max = y[ymm.second-y.begin()];
	std::cout << x_min << " " << x_max << " " << y_min << " " << y_max << std::endl;
//	std::cout << x_mean << " " << y_mean << std::endl; 
	return 0;
}
**/

int main(int argc, char** argv)
{
	std::vector<int> filenames;
	filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");
	if (filenames.size() != 1) {
		std::cout << "Please specify a .ply point cloud with XYZRGB points!" << std::endl;
		return -1;
	}
	pcl::PointCloud<PointTreeseg>::Ptr source_cloud(new pcl::PointCloud<PointTreeseg> ());
	if (pcl::io::loadPLYFile(argv[filenames[0]], *source_cloud) < 0) {
		std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
		return -1;
	}

 	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	if (strcmp(argv[1], "-luma") == 0) {
		float theta = M_PI/2;
		// transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));
		// transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
		transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY())).rotate(
			Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));
	} else {
		float theta = -M_PI/2; // The angle of rotation in radians
		transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));
	}
	pcl::PointCloud<PointTreeseg>::Ptr rotated_cloud(new pcl::PointCloud<PointTreeseg> ());
	pcl::transformPointCloud(*source_cloud, *rotated_cloud, transform);
	Eigen::Vector4f rotated_min, rotated_max;
	pcl::getMinMax3D(*rotated_cloud, rotated_min, rotated_max);
	std::cout << "Rotated cloud axes minmax" << std::endl;
	std::cout << rotated_min[0] << " < X < " << rotated_max[0] << std::endl;
	std::cout << rotated_min[1] << " < Y < " << rotated_max[1] << std::endl;
	std::cout << rotated_min[2] << " < Z < " << rotated_max[2] << std::endl;

	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
	transform_2.translation() << 0.0, 0.0, -rotated_min[2];  // will make the lowest Z 0.
	pcl::PointCloud<PointTreeseg>::Ptr zpositive_cloud(new pcl::PointCloud<PointTreeseg> ());
	pcl::transformPointCloud(*rotated_cloud, *zpositive_cloud, transform_2);
	Eigen::Vector4f zpos_min, zpos_max;
	pcl::getMinMax3D(*zpositive_cloud, zpos_min, zpos_max);
	std::cout << "Final z-positive cloud axes minmax" << std::endl;
	std::cout << zpos_min[0] << " < X < " << zpos_max[0] << std::endl;
	std::cout << zpos_min[1] << " < Y < " << zpos_max[1] << std::endl;
	std::cout << zpos_min[2] << " < Z < " << zpos_max[2] << std::endl;

	// Store zpos in <name>.tile.0.pcd
	std::vector<std::string> tmp1,tmp2;
	boost::split(tmp1, argv[filenames[0]], boost::is_any_of("/"));
	boost::split(tmp2, tmp1[tmp1.size() - 1], boost::is_any_of("."));
	std::stringstream ss;
	ss.str("");
	ss << tmp2[0] << ".tile.0.pcd";
	pcl::io::savePCDFileBinary(ss.str(), *zpositive_cloud);
	// Store rotated_min[2] in original_zmin.dat
	ss.str("");
	ss << tmp2[0] << ".zmin.txt";
	std::ofstream out(ss.str());
	out << rotated_min[2] << std::endl;
	out.close();
	// Store new xmin,xmax,ymin,ymax
	ss.str("");
	ss << tmp2[0] << ".coords.dat";
	std::ofstream out2(ss.str());
	out2 << zpos_min[0] << " " << zpos_max[0] << " " << zpos_min[1] << " " << zpos_max[1] << std::endl;
	out2.close();
	return 0;
}
