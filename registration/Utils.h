#ifndef UTILS_H_
#define UTILS_H_

// System headers
#include <iostream>
#include <string>
// PCL input/output headers
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
// PCL cloud headers
#include <pcl/point_cloud.h>

#define DEBUG 1

class Utils
{
	public:

		static void get_pcd_filenames
			(
				const std::string & pDirectoryPath,
				std::vector<std::string> & pFilenames
			);
};

#endif