#include "treeseg.h"

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include "json.hpp"
using json = nlohmann::json;

int main(int argc, char **argv)
{
	std::vector<std::string> args(argv+1,argv+argc);
	//
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	std::stringstream ss;
	//
	std::cout << "Reading slice: " << std::flush;
	std::vector<std::string> id = getFileID(args[4]);
	pcl::PointCloud<PointTreeseg>::Ptr slice(new pcl::PointCloud<PointTreeseg>);
	reader.read(args[4],*slice);
	std::cout << "complete" << std::endl;
	//
	std::cout << "Cluster extraction: " << std::flush;
	std::vector<pcl::PointCloud<PointTreeseg>::Ptr> clusters;
	int nnearest = 18;
	int nmin = 100;
	std::vector<float> nndata = dNN(slice,nnearest);
	euclideanClustering(slice,nndata[0],nmin,clusters);
	ss.str("");
	ss << id[0] << ".intermediate.slice.clusters.pcd";
	writeClouds(clusters,ss.str(),false);
	std::cout << ss.str() << " | " << clusters.size() << std::endl;
	//
	std::cout << "Region-based segmentation: " << std::flush;
	std::vector<pcl::PointCloud<PointTreeseg>::Ptr> regions;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	float smoothness = std::stof(args[0]);
	for(int i=0;i<clusters.size();i++)
	{	// this is where we lose candidates in Glendoveer--Portland--h2. Patches get broken up too small. Minor inhomogeneities seem to cause this (bark!)
		std::vector<pcl::PointCloud<PointTreeseg>::Ptr> tmpregions;
		estimateNormals(clusters[i],50,normals);
		// regionSegmentation(clusters[i],normals,250,100,std::numeric_limits<int>::max(),smoothness,2,tmpregions);
		// we have a lot fewer omissions with this change.
		regionSegmentation(clusters[i],normals,250,50,std::numeric_limits<int>::max(),smoothness,2,tmpregions);
		for(int j=0;j<tmpregions.size();j++) regions.push_back(tmpregions[j]);
		normals->clear();
	}
	ss.str("");
	ss << id[0] << ".intermediate.slice.clusters.regions.pcd";
	writeClouds(regions,ss.str(),false);
	std::cout << ss.str() << " | " << regions.size() << std::endl;
	//
	std::cout << "RANSAC cylinder fits: " << std::endl << std::flush;
	// std::vector<std::pair<float,pcl::PointCloud<PointTreeseg>::Ptr>> cylinders;
	std::vector<cylinder> cylinders;
	nnearest = 60;
	float dmin = std::stof(args[1]);
	float dmax = std::stof(args[2]);
	std::ifstream coordfile;
	coordfile.open(args[3]);
	float coords[4];
	int n = 0;
	if(coordfile.is_open())
	{
		while(!coordfile.eof())
		{
			coordfile >> coords[n];
			n++;
		}
	}
	coordfile.close();
	float xmin = coords[0];
	float xmax = coords[1];
	float ymin = coords[2];
	float ymax = coords[3];
	float lmin = 1.0; // was 2.5m, assumed 3m slice
	float stepcovmax = 0.1;
	float radratiomin = 0.9;
	// problems arising here with our not-scaled-up pcd's: far too little length. no radratio.
	for(int i=0;i<regions.size();i++)
	{
		cylinder cyl;
		fitCylinder(regions[i],nnearest,true,true,cyl);
		// std::cout << "cyl: " << cyl.ismodel << " " << cyl.rad << " " << cyl.len << " " << cyl.stepcov << " " << cyl.radratio << " " << cyl.x << " " << cyl.y << std::endl;
		if(cyl.ismodel == true)
		{		
			if(cyl.rad*2 >= dmin && cyl.rad*2 <= dmax && cyl.len >= lmin)
			{
				if(cyl.stepcov <= stepcovmax && cyl.radratio > radratiomin)
				{
					if(cyl.x >= xmin && cyl.x <= xmax)
					{
						if(cyl.y >= ymin && cyl.y <= ymax)
						{
							// cylinders.push_back(std::make_pair(cyl.rad,cyl.inliers));
							cylinders.push_back(cyl);
						}
					}
				}
			}
		}
	}

	// std::sort(cylinders.rbegin(),cylinders.rend());
	std::sort(cylinders.begin(), cylinders.end(), [](cylinder a, cylinder b){return a.rad > b.rad;});
	std::vector<pcl::PointCloud<PointTreeseg>::Ptr> cyls;
	// we are interested in the cylinder objects. these have pointcloud members (=inliers) which will be output separately.
	// cylinders is a list of pairs (radius, inliers).
	// cyls is a list of pointcloud objects which are the cylinders' inliers.
	// for(int i=0;i<cylinders.size();i++) cyls.push_back(cylinders[i].second);

	// solely to be able to save the inlier clouds using writeClouds below
	for(int i=0; i<cylinders.size(); i++) {
		cyls.push_back(cylinders[i].inliers);
	}
	ss.str("");
	ss << id[0] << ".intermediate.slice.clusters.regions.cylinders.pcd";
	writeClouds(cyls,ss.str(),false);
	std::cout << ss.str() << " | " << cyls.size() << std::endl;
	//
	std::cout << "Principal component trimming: " << std::flush;
	float anglemax = 35;
	std::vector<int> idx;
	// for(int j=0;j<cyls.size();j++)
	for (int j=0; j<cylinders.size(); j++)
	{
		Eigen::Vector4f centroid;
		Eigen::Matrix3f covariancematrix;
		Eigen::Matrix3f eigenvectors;
		Eigen::Vector3f eigenvalues;
		// computePCA(cyls[j],centroid,covariancematrix,eigenvectors,eigenvalues);
		computePCA(cylinders[j].inliers,centroid,covariancematrix,eigenvectors,eigenvalues);
		Eigen::Vector4f gvector(eigenvectors(0,2),eigenvectors(1,2),0,0);
		Eigen::Vector4f cvector(eigenvectors(0,2),eigenvectors(1,2),eigenvectors(2,2),0);
		float angle = pcl::getAngle3D(gvector,cvector) * (180/M_PI);
		if (angle >= (90 - anglemax) && angle <= (90 + anglemax)) {
			cylinders[j].angle = angle;
			cylinders[j].cvector = cvector;
			idx.push_back(j);
		}
	}

	std::vector<cylinder> cylinders_angle_filtered(idx.size());
	std::transform(idx.begin(), idx.end(), cylinders_angle_filtered.begin(), 
		[cylinders](size_t pos) {return cylinders[pos];});

	// solely to be able to save the inlier clouds using writeClouds below
	std::vector<pcl::PointCloud<PointTreeseg>::Ptr> pca;
	for (size_t k=0; k<cylinders_angle_filtered.size(); k++) {
		pca.push_back(cylinders_angle_filtered[k].inliers);
	}
    // for (int k=0; k<idx.size(); k++) {
	// 	// pca.push_back(cyls[idx[k]]);
	// 	pca.push_back(cylinders[idx[k]].inliers);
	// }
	ss.str("");
	ss << id[0] << ".intermediate.slice.clusters.regions.cylinders.principal.pcd";
	writeClouds(pca,ss.str(),false);
	std::cout << ss.str() << " | " << pca.size() << std::endl;
	//
	std::cout << "Concatenating stems: " << std::flush;
	float expansionfactor = 0;
	// std::vector<pcl::PointCloud<PointTreeseg>::Ptr> stems;
	// stems = pca;
	// catIntersectingClouds(stems);
	catIntersectingClouds2(cylinders_angle_filtered);
	// solely to be able to save the inlier clouds using writeClouds below
	std::vector<pcl::PointCloud<PointTreeseg>::Ptr> stems;
	for (size_t k=0; k<cylinders_angle_filtered.size(); k++) {
		stems.push_back(cylinders_angle_filtered[k].inliers);
	}
	ss.str("");
	ss << id[0] << ".intermediate.slice.clusters.regions.cylinders.principal.cat.pcd";
	writeClouds(stems,ss.str(),false);
	for(int m=0;m<stems.size();m++)
	{
		ss.str("");
		ss << id[0] << ".cluster." << m << ".pcd";
		writer.write(ss.str(),*stems[m],true);
	}
	std::cout << stems.size() << std::endl;

	// write json
	json trees_array = json::array();
	for (auto cyl: cylinders_angle_filtered) {
		json tree = {
			{"x", cyl.x},
			{"y", cyl.y},
			{"z", cyl.z},
			{"dx", cyl.dx},
			{"dy", cyl.dy},
			{"dz", cyl.dz},
			{"radius", cyl.rad},
			{"angle", cyl.angle},
			{"cvector", cyl.cvector}
		};
		trees_array.push_back(tree);
	}
	std::ofstream trees_jsonfile("trees.json");
	trees_jsonfile << trees_array << std::endl;
	trees_jsonfile.close();
	return 0;
}
