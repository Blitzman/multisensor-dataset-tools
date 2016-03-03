// Standard headers
#include <iostream>
#include <string>
// Boost headers
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
// PCL input/output headers
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
// PCL cloud headers
#include <pcl/point_cloud.h>
// PCL visualization headers
#include <pcl/visualization/pcl_visualizer.h>
// Custom includes
#include "PointCloudOperations.h"
#include "Utils.h"

#define DEBUG 1

#define PARAM_HELP						"help"
#define PARAM_NAME						"name"
#define PARAM_VIEWER					"v"
#define PARAM_ROTATION					"r"
#define PARAM_CLOUDSDIR					"dir"
#define PARAM_NUMCLOUDS					"n"
#define PARAM_STEP_ANGLE				"s"
#define PARAM_GROUND_CENTER_X			"c_x"
#define PARAM_GROUND_CENTER_Y			"c_y"
#define PARAM_GROUND_CENTER_Z			"c_z"
#define PARAM_GROUND_NORMAL_X			"n_x"
#define PARAM_GROUND_NORMAL_Y			"n_y"
#define PARAM_GROUND_NORMAL_Z			"n_z"
#define PARAM_BILATERAL_FILTER			"bf"
#define PARAM_BILATERAL_FILTER_SIGMA_R	"bf_sr"
#define PARAM_BILATERAL_FILTER_SIGMA_S	"bf_ss"
#define PARAM_NORMAL_ESTIMATION_K		"ne_k"
#define PARAM_NORMAL_ESTIMATION_DOWN_K	"ne_d_k"
#define PARAM_ROR						"ror"
#define PARAM_ROR_NEIGHBORS				"ror_n"
#define PARAM_ROR_RADIUS				"ror_r"
#define PARAM_SOR						"sor"
#define PARAM_SOR_MEANK					"sor_mk"
#define PARAM_SOR_STDDEV				"sor_sd"
#define PARAM_ECE						"ece"
#define PARAM_ECE_TOLERANCE				"ece_t"
#define PARAM_ECE_MINSIZE				"ece_mns"
#define PARAM_ECE_MAXSIZE				"ece_mxs"
#define PARAM_ECE_CLUSTERS				"ece_c"
#define PARAM_VG						"vg"
#define PARAM_VG_LEAFSIZE				"vg_ls"
#define PARAM_PMR						"pmr"
#define PARAM_PMR_MLS_SEARCHRADIUS		"pmr_mls_sr"
#define PARAM_PMR_MLS_POLYNOMIALFIT		"pmr_mls_pf"
#define PARAM_PMR_MLS_POLYNOMIALORDER	"pmr_mls_po"
#define PARAM_PMR_MLS_UPSAMPLINGRADIUS	"pmr_mls_ur"
#define PARAM_PMR_MLS_UPSAMPLINGSTEP	"pmr_mls_us"
#define PARAM_PMR_NE_RADIUSSEARCH		"pmr_ne_rs"
#define PARAM_PMR_P_DEPTH				"pmr_p_d"
#define PARAM_ICP						"icp"
#define PARAM_ICP_MAXITERATIONS			"icp_i"
#define PARAM_ICP_EPSILON				"icp_e"
#define PARAM_CUT						"cut"
#define PARAM_CUT_AMOUNT				"cut_a"
#define PARAM_SAVE_BINARY				"bin"


// Center is point #134946 in empty cloud
// Center is at: 0.001557, -0.136046, 0.80500
// Normal is: 0.021264, -0.928830, 0.369895
// Eigen::Vector3f originTranslation(0.001557, 0.136046, -0.80500);
// Eigen::Vector3f groundAxis(0.021264, -0.928830, 0.369895);

// ------------------------------------------------------------------------------------------------
bool parse_command_line_options(boost::program_options::variables_map & pVariablesMap, const int & pArgc, char ** pArgv)
{
	try
	{
		boost::program_options::options_description desc("Allowed options");
		desc.add_options()
			(PARAM_HELP, "produce help message")
			(PARAM_NAME, boost::program_options::value<std::string>()->default_value("noname"), "Name for the session")
			(PARAM_VIEWER, boost::program_options::value<bool>()->default_value(true), "Activate viewer")
			(PARAM_ROTATION, boost::program_options::value<int>()->default_value(-1), "Rotation modifier")
			(PARAM_CLOUDSDIR, boost::program_options::value<std::string>()->required(), "PCD files directory")
			(PARAM_NUMCLOUDS, boost::program_options::value<int>()->default_value(64), "Number of clouds")
			(PARAM_STEP_ANGLE, boost::program_options::value<double>()->default_value(5.625), "Step rotation angle")
			(PARAM_GROUND_CENTER_X, boost::program_options::value<double>()->default_value(0.001557), "Ground truth table center X")
			(PARAM_GROUND_CENTER_Y, boost::program_options::value<double>()->default_value(0.136046), "Ground truth table center Y")
			(PARAM_GROUND_CENTER_Z, boost::program_options::value<double>()->default_value(-0.80500), "Ground truth table center Z")
			(PARAM_GROUND_NORMAL_X, boost::program_options::value<double>()->default_value(0.021264), "Ground truth table normal X")
			(PARAM_GROUND_NORMAL_Y, boost::program_options::value<double>()->default_value(-0.92883), "Ground truth table normal Y")
			(PARAM_GROUND_NORMAL_Z, boost::program_options::value<double>()->default_value(0.369895), "Ground truth table normal Z")
			(PARAM_BILATERAL_FILTER, boost::program_options::value<bool>()->default_value(false), "Apply bilateral filter to scene")
			(PARAM_BILATERAL_FILTER_SIGMA_R, boost::program_options::value<float>()->default_value(0.05f), "Bilateral filter sigma R")
			(PARAM_BILATERAL_FILTER_SIGMA_S, boost::program_options::value<float>()->default_value(15.0f), "Bilateral filter sigma S")
			(PARAM_NORMAL_ESTIMATION_K, boost::program_options::value<int>()->default_value(25), "Normal estimation neighbors")
			(PARAM_NORMAL_ESTIMATION_DOWN_K, boost::program_options::value<int>()->default_value(25), "Normal estimation neighbors (downsampled cloud)")
			(PARAM_SOR, boost::program_options::value<bool>()->default_value(true), "Activate Statistical Outlier Removal")
			(PARAM_SOR_MEANK, boost::program_options::value<int>()->default_value(50), "SOR mean K")
			(PARAM_SOR_STDDEV, boost::program_options::value<float>()->default_value(0.90f), "SOR standard deviation")
			(PARAM_ROR, boost::program_options::value<bool>()->default_value(false), "Activate Radial Outlier Removal")
			(PARAM_ROR_NEIGHBORS, boost::program_options::value<int>()->default_value(10), "ROR number of neighbors")
			(PARAM_ROR_RADIUS, boost::program_options::value<float>()->default_value(0.008f), "ROR search radius")
			(PARAM_ECE, boost::program_options::value<bool>()->default_value(true), "Activate Euclidean Cluster Extraction")
			(PARAM_ECE_TOLERANCE, boost::program_options::value<float>()->default_value(0.02f), "ECE cluster tolerance")
			(PARAM_ECE_MINSIZE, boost::program_options::value<int>()->default_value(100), "ECE minimum cluster size")
			(PARAM_ECE_MAXSIZE, boost::program_options::value<int>()->default_value(25000), "ECE maximum cluster size")
			(PARAM_ECE_CLUSTERS, boost::program_options::value<int>()->default_value(1), "ECE clusters selected")
			(PARAM_VG, boost::program_options::value<bool>()->default_value(false), "Activate Voxel Grid downsampling")
			(PARAM_VG_LEAFSIZE, boost::program_options::value<float>()->default_value(0.05), "VG leaf size")
			(PARAM_PMR, boost::program_options::value<bool>()->default_value(false), "Activate Poisson Mesh Reconstruction")
			(PARAM_PMR_MLS_SEARCHRADIUS, boost::program_options::value<float>()->default_value(0.01), "PMR MLS search radius")
			(PARAM_PMR_MLS_POLYNOMIALFIT, boost::program_options::value<bool>()->default_value(true), "PMR MLS polynomial fit")
			(PARAM_PMR_MLS_POLYNOMIALORDER, boost::program_options::value<int>()->default_value(2), "PMR MLS polynomial order")
			(PARAM_PMR_MLS_UPSAMPLINGRADIUS, boost::program_options::value<float>()->default_value(0.005), "PMR MLS upsampling radius")
			(PARAM_PMR_MLS_UPSAMPLINGSTEP, boost::program_options::value<float>()->default_value(0.003), "PMR MLS step size")
			(PARAM_PMR_NE_RADIUSSEARCH, boost::program_options::value<float>()->default_value(0.01), "PMR NE search radius")
			(PARAM_PMR_P_DEPTH, boost::program_options::value<int>()->default_value(9), "PMR Poisson depth")
			(PARAM_ICP, boost::program_options::value<bool>()->default_value(false), "Activate Iterative Closest Point")
			(PARAM_ICP_MAXITERATIONS, boost::program_options::value<int>()->default_value(1000), "ICP maximum iterations")
			(PARAM_ICP_EPSILON, boost::program_options::value<double>()->default_value(0.000001), "ICP epsilon")
			(PARAM_CUT, boost::program_options::value<bool>()->default_value(false), "Activate Cloud Cut")
			(PARAM_CUT_AMOUNT, boost::program_options::value<int>()->default_value(50), "CUT amount")
			(PARAM_SAVE_BINARY, boost::program_options::value<bool>()->default_value(false), "Save clouds in binary format");

		boost::program_options::store(boost::program_options::parse_command_line(pArgc, pArgv, desc), pVariablesMap);

		if (pVariablesMap.count(PARAM_HELP))
		{
			std::cout << desc << "\n";
			return true;
		}

		boost::program_options::notify(pVariablesMap);
	}
	catch (std::exception & e)
	{
		std::cerr << "Error: " << e.what() << "\n";
		return true;
	}

	return false;
}

int main (int argc, char** argv)
{
	boost::program_options::variables_map variablesMap;

	// Parse parameters, exit if help is requested
	if (parse_command_line_options(variablesMap, argc, argv))
		return 1;

	// General parameters
	std::string sessionName			= variablesMap[PARAM_NAME].as<std::string>();
	bool enabledViewer				= variablesMap[PARAM_VIEWER].as<bool>();
	int rotationModifier			= variablesMap[PARAM_ROTATION].as<int>();
	std::string cloudsDirectory		= variablesMap[PARAM_CLOUDSDIR].as<std::string>();
	int numClouds					= variablesMap[PARAM_NUMCLOUDS].as<int>();
	double stepRotationAngle		= variablesMap[PARAM_STEP_ANGLE].as<double>();
	bool saveBinaryFormat			= variablesMap[PARAM_SAVE_BINARY].as<bool>();

	// Ground truth parameters
	double groundCenterX = variablesMap[PARAM_GROUND_CENTER_X].as<double>();
	double groundCenterY = variablesMap[PARAM_GROUND_CENTER_Y].as<double>();
	double groundCenterZ = variablesMap[PARAM_GROUND_CENTER_Z].as<double>();
	double groundNormalX = variablesMap[PARAM_GROUND_NORMAL_X].as<double>();
	double groundNormalY = variablesMap[PARAM_GROUND_NORMAL_Y].as<double>();
	double groundNormalZ = variablesMap[PARAM_GROUND_NORMAL_Z].as<double>();

	// Bilateral Filtering parameters
	bool applyBilateralFilter = variablesMap[PARAM_BILATERAL_FILTER].as<bool>();
	float bilateralFilterSigmaR = variablesMap[PARAM_BILATERAL_FILTER_SIGMA_R].as<float>();
	float bilateralFilterSigmaS = variablesMap[PARAM_BILATERAL_FILTER_SIGMA_S].as<float>();

	// Normal estimation parameters
	int normalEstimationK				= variablesMap[PARAM_NORMAL_ESTIMATION_K].as<int>();
	int downsampledNormalEstimationK	= variablesMap[PARAM_NORMAL_ESTIMATION_DOWN_K].as<int>();

	// SOR parameters
	bool sorEnabled		= variablesMap[PARAM_SOR].as<bool>();
	int sorMeanK		= variablesMap[PARAM_SOR_MEANK].as<int>();
	float sorStdDev		= variablesMap[PARAM_SOR_STDDEV].as<float>();

	// ROR parameters
	bool rorEnabled		= variablesMap[PARAM_ROR].as<bool>();
	int rorNeighbors	= variablesMap[PARAM_ROR_NEIGHBORS].as<int>();
	float rorRadius		= variablesMap[PARAM_ROR_RADIUS].as<float>();

	// Cut parameters
	bool cutEnabled		= variablesMap[PARAM_CUT].as<bool>();
	int cutAmount		= variablesMap[PARAM_CUT_AMOUNT].as<int>();

	// ECE parameters
	bool eceEnabled		= variablesMap[PARAM_ECE].as<bool>();
	int eceClusters		= variablesMap[PARAM_ECE_CLUSTERS].as<int>();
	int eceMaxSize		= variablesMap[PARAM_ECE_MAXSIZE].as<int>();
	int eceMinSize		= variablesMap[PARAM_ECE_MINSIZE].as<int>();
	float eceTolerance	= variablesMap[PARAM_ECE_TOLERANCE].as<float>();

	// ICP parameters
	bool icpEnabled		= variablesMap[PARAM_ICP].as<bool>();
	int icpIterations	= variablesMap[PARAM_ICP_MAXITERATIONS].as<int>();
	double icpEpsilon	= variablesMap[PARAM_ICP_EPSILON].as<double>();

	// VG parameters
	bool vgEnabled		= variablesMap[PARAM_VG].as<bool>();
	float vgLeafSize	= variablesMap[PARAM_VG_LEAFSIZE].as<float>();

	// PMR parameters
	bool pmrEnabled		= variablesMap[PARAM_PMR].as<bool>();
	float pmrSrchRadM	= variablesMap[PARAM_PMR_MLS_SEARCHRADIUS].as<float>();
	bool pmrPolyFit		= variablesMap[PARAM_PMR_MLS_POLYNOMIALFIT].as<bool>();
	int pmrPolyOrder	= variablesMap[PARAM_PMR_MLS_POLYNOMIALORDER].as<int>();
	float pmrUpSmpRad	= variablesMap[PARAM_PMR_MLS_UPSAMPLINGRADIUS].as<float>();
	float pmrUpSmpStp	= variablesMap[PARAM_PMR_MLS_UPSAMPLINGSTEP].as<float>();
	float pmrSrchRadN	= variablesMap[PARAM_PMR_NE_RADIUSSEARCH].as<float>();
	int pmrDepth		= variablesMap[PARAM_PMR_P_DEPTH].as<int>();


	// Get PCD filenames from the specified directory
	std::vector<std::string> filenames;
	Utils::get_pcd_filenames(cloudsDirectory, filenames);

	// Ground truth center and normal
	Eigen::Vector3f groundCenter(groundCenterX, groundCenterY, groundCenterZ);
	Eigen::Vector3f groundNormal(groundNormalX, groundNormalY, groundNormalZ);

	pcl::visualization::PCLVisualizer viewer(sessionName.c_str());
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
	Eigen::Matrix4f icpAccumulatedTransformation = Eigen::Matrix4f::Identity();

	int rotationMultiplier = 0;
	for (auto it = filenames.begin(); it != filenames.end() && rotationMultiplier < numClouds; ++it)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>());

		// Load PCD file
		if (pcl::io::loadPCDFile (*it, *cloud) < 0)  
		{
			std::cerr << "Error loading point cloud " << *it << "\n";
			return -1;
		}

		std::cout << "Loaded point cloud: " << *it << "\n";

		// Apply bilateral filter to cloud
		if (applyBilateralFilter)
			PointCloudOperations::filter_bilateral(
			cloud,
			bilateralFilterSigmaR,
			bilateralFilterSigmaS,
			cloud);

		// Remove NaNs from the point cloud
		std::vector<int> mapping;
		pcl::removeNaNFromPointCloud(*cloud, *cloud, mapping);
		std::cout << "NaNs removed..." << "\n";

		// Statistical Outlier Removal filtering
		if (sorEnabled)
			PointCloudOperations::sor_cloud	
				(
					cloud, 
					cloud, 
					sorMeanK, 
					sorStdDev
				);

		// Radial Outlier Removal filtering
		if (rorEnabled)
			PointCloudOperations::ror_cloud
				(
					cloud,
					cloud,
					rorNeighbors,
					rorRadius
				);

		// Cloud cutting threshold
		if (cutEnabled)
			PointCloudOperations::cut_cloud
				(
					cloud,
					cloud,
					cutAmount
				);

		// Euclidean Cluster Extraction filter
		if (eceEnabled)
			PointCloudOperations::ece_cloud
				(
					cloud, 
					cloud, 
					eceTolerance,
					eceMinSize,
					eceMaxSize,
					eceClusters
				);

		// Translation to origin
		PointCloudOperations::translate_cloud
			(
				cloud, 
				cloud, 
				groundCenter
			);

		// Step rotation
		PointCloudOperations::rotate_cloud
			(
				cloud, 
				cloud, 
				(rotationModifier * stepRotationAngle * rotationMultiplier), 
				groundNormal
			);

		// Cloud ICP alignment
		if (icpEnabled && rotationMultiplier > 0)
			PointCloudOperations::icp_align_cloud
				(
					cloud,
					clouds.at(rotationMultiplier - 1),
					cloud,
					icpIterations,
					icpEpsilon
				);

		clouds.push_back(cloud);
		++rotationMultiplier;
	}

	// Concatenate all the processed clouds
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr concatenatedClouds(new pcl::PointCloud<pcl::PointXYZRGB>());
	PointCloudOperations::concatenate_clouds(clouds, concatenatedClouds);

	//PointCloudOperations::translate_cloud(concatenatedClouds, concatenatedClouds, Eigen::Vector3f(-groundCenter.x(), -groundCenter.y(), -groundCenter.z()));

	// Save 360-registered clouds
	pcl::io::savePCDFile(sessionName + "_cloud.pcd", *concatenatedClouds, saveBinaryFormat);
	pcl::io::savePLYFile(sessionName + "_cloud.ply", *concatenatedClouds, saveBinaryFormat);

	// Save cloud normals
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
	PointCloudOperations::compute_normals(concatenatedClouds, normalEstimationK, normals);
	for (size_t i = 0; i < normals->size(); ++i)
	{
		normals->points[i].normal_x *= -1.0;
		normals->points[i].normal_y *= -1.0;
		normals->points[i].normal_z *= -1.0;
	}
	pcl::io::savePCDFile(sessionName + "_cloud_normals.pcd", *normals, saveBinaryFormat);
	pcl::io::savePLYFile(sessionName + "_cloud_normals.ply", *normals, saveBinaryFormat);

	// Save cloud with normals
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
	pcl::concatenateFields(*concatenatedClouds, *normals, *cloudWithNormals);
	pcl::io::savePCDFile(sessionName + "_cloud_with_normals.pcd", *cloudWithNormals, saveBinaryFormat);
	pcl::io::savePLYFile(sessionName + "_cloud_with_normals.ply", *cloudWithNormals, saveBinaryFormat);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::Normal>::Ptr downsampledNormals(new pcl::PointCloud<pcl::Normal>());
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr downsampledCloudWithNormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>());

	// Voxel Grid filter
	if (vgEnabled)
	{
		PointCloudOperations::vg_cloud
			(
				concatenatedClouds,
				downsampledCloud,
				vgLeafSize
			);

		// Save downsampled clouds
		pcl::io::savePCDFile(sessionName + "_cloud_downsampled.pcd", *downsampledCloud, saveBinaryFormat);
		pcl::io::savePLYFile(sessionName + "_cloud_downsampled.ply", *downsampledCloud, saveBinaryFormat);

		// Save downsampled clouds normals
		PointCloudOperations::compute_normals(downsampledCloud, downsampledNormalEstimationK, downsampledNormals);
		for (size_t i = 0; i < downsampledNormals->size(); ++i)
		{
			downsampledNormals->points[i].normal_x *= -1.0;
			downsampledNormals->points[i].normal_y *= -1.0;
			downsampledNormals->points[i].normal_z *= -1.0;
		}
		pcl::io::savePCDFile(sessionName + "_cloud_downsampled_normals.pcd", *downsampledNormals, saveBinaryFormat);
		pcl::io::savePLYFile(sessionName + "_cloud_downsampled_normals.ply", *downsampledNormals, saveBinaryFormat);

		// Save downsampled clouds with normals
		pcl::concatenateFields(*downsampledCloud, *downsampledNormals, *downsampledCloudWithNormals);
		pcl::io::savePCDFile(sessionName + "_cloud_downsampled_with_normals.pcd", *downsampledCloudWithNormals, saveBinaryFormat);
		pcl::io::savePLYFile(sessionName + "_cloud_downsampled_with_normals.ply", *downsampledCloudWithNormals, saveBinaryFormat);
	}
	else
	{
		//pcl::copyPointCloud(*concatenatedClouds, *downsampledCloud);
		downsampledCloud = concatenatedClouds;
		downsampledNormals = normals;
		downsampledCloudWithNormals = downsampledCloudWithNormals;
	}

	// Poisson Mesh reconstruction
	if (pmrEnabled)
	{
		pcl::PolygonMesh mesh;

		PointCloudOperations::pmr_cloud
			(
				downsampledCloud,
				mesh,
				pmrSrchRadM,
				pmrPolyFit,
				pmrPolyOrder,
				pmrUpSmpRad,
				pmrUpSmpStp,
				pmrSrchRadN,
				pmrDepth
			);

		// Save reconstructed mesh
		pcl::io::savePLYFile(sessionName + "_mesh.ply", mesh, 5);

		viewer.addPolygonMesh(mesh);
	}

	// Set up the viewer
	if (enabledViewer)
	{
		std::cout << "Adding clouds to viewer...\n";

		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> cloudColorHandler(downsampledCloud);
		viewer.addPointCloud(downsampledCloud, cloudColorHandler, "cloud");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
		viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(downsampledCloud, downsampledNormals, 100, 0.05, "cloudNormals");

		viewer.addCoordinateSystem(0.75, 0);
		viewer.setBackgroundColor(0.8, 0.8, 0.8, 0);

		while (!viewer.wasStopped())
		{
			viewer.spinOnce();
		}
	}

	return 0;
}