#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/console/time.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/conversions.h>
 
#include <iostream>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void
showHelp(char *filename)
{
  std::cout << std::endl;
  std::cout << "***************************************************************************" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "*                   PCD SEGMENTATION (OBJECT DATASET)                     *" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "***************************************************************************" << std::endl << std::endl;
  std::cout << "Usage: " << filename << " --dir C:\\PCDdir\ [Options]" << std::endl << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "     -h:                     Show this help." << std::endl;
  std::cout << "     -p:                     Plane segmentation" << std::endl;
  std::cout << "     -f:                     Statistical filtering." << std::endl;
  std::cout << "     -c:                     Chroma key segmentation" << std::endl;
  std::cout << "     -batch:                 Batch mode (no viewer)" << std::endl;
  std::cout << "                             " << std::endl;
  std::cout << "     --chroma_threshold val  Chroma threshold (default 35)." << std::endl;
  std::cout << "     --chroma_cutoff val     Chroma cutoff (default 0)." << std::endl;
  std::cout << "     --center_x val:         Table centroid (x-coordinate) (default 0.022)" << std::endl;
  std::cout << "     --center_y val:         Table centroid (y-coordinate) (default 0.088)" << std::endl;
  std::cout << "     --center_z val:         Table centroid (z-coordinate) (default 0.665)" << std::endl; 
  std::cout << "     --bb_radius val:        Bounding box radius (default 0.2 meters)" << std::endl;
}

// Parameters
float center_x = 0.02216f;
float center_y = 0.08803f;
float center_z = 0.66500f;

float deviation = 0.20;

bool bPlaneExtraction = false;
bool bStatisticalFiltering = false;
bool bChromaSegmentation = false;
bool bBatch = false;

std::string dir;

// chroma segmentation parameters
float maximumCutoff = 0;
float differenceThreshold = 35;

void
saveImage(const std::string &filename, const pcl::PCLImage& image)
{
  TicToc tt;
  tt.tic();
  print_highlight("Saving "); print_value("%s ", filename.c_str());
  savePNGFile(filename, image);
  print_info("[done, "); print_value("%g", tt.toc()); print_info(" ms : "); print_value("%d", image.width * image.height); print_info(" points]\n");
}

bool
loadCloud(const std::string &filename, pcl::PCLPointCloud2 &cloud)
{
  TicToc tt;
  print_highlight("Loading "); print_value("%s ", filename.c_str());

  tt.tic();
  if (loadPCDFile(filename, cloud) < 0)
    return (false);
  print_info("[done, "); print_value("%g", tt.toc()); print_info(" ms : "); print_value("%d", cloud.width * cloud.height); print_info(" points]\n");
  print_info("Available dimensions: "); print_value("%s\n", pcl::getFieldsList(cloud).c_str());

  return (true);
}

void writePBM(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, std::string file)
{
  ofstream fFile;
  fFile.open(file);

  fFile << cloud.width << " " << cloud.height << std::endl;

  for (size_t i = 0; i < cloud.size(); i++)
  {
    if (pcl::isFinite(cloud.points[i]))
    {
      fFile << "1"; 
    }
    else
    {
      fFile << "0";
    }
    if ((i+1)%cloud.width == 0)
      fFile << std::endl;
    else
      fFile << " ";
  }
  fFile.close();
}

std::vector<std::string> getPcdFilesInDir(const std::string& directory)
{
  namespace fs = boost::filesystem;
  fs::path dir(directory);
  std::cout << "path: " << directory << std::endl;
  if (directory.empty() || !fs::exists(dir) || !fs::is_directory(dir))
    PCL_THROW_EXCEPTION(pcl::IOException, "No valid PCD directory given!\n");

  std::vector<std::string> result;
  fs::directory_iterator pos(dir);
  fs::directory_iterator end;

  for (; pos != end; ++pos)
    if (fs::is_regular_file(pos->status()))
      if (fs::extension(*pos) == ".pcd")
      {
#if BOOST_FILESYSTEM_VERSION == 3
    result.push_back(pos->path().string());
#else
    result.push_back(pos->path());
#endif
    //cout << "added: " << result.back() << endl;
      }

  return result;
}

int
main(int argc, char** argv)
{
  //Show help
  if (pcl::console::find_switch(argc, argv, "-h"))
  {
    showHelp(argv[0]);
    exit(0);
  }

	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud_z(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud_y(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud_x(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr aux(new pcl::PointCloud<pcl::PointXYZRGB>);
 
  if (pcl::console::find_switch(argc, argv, "-p"))
    bPlaneExtraction = true;

  if (pcl::console::find_switch(argc, argv, "-f"))
    bStatisticalFiltering = true;

  if (pcl::console::find_switch(argc, argv, "-c"))
    bChromaSegmentation = true;

  if (pcl::console::find_switch(argc, argv, "-batch"))
    bBatch = true;

  pcl::console::parse_argument(argc, argv, "--chroma_threshold", differenceThreshold);
  pcl::console::parse_argument(argc, argv, "--chroma_cutoff", maximumCutoff);
  pcl::console::parse_argument(argc, argv, "--center_x", center_x);
  pcl::console::parse_argument(argc, argv, "--center_y", center_y);
  pcl::console::parse_argument(argc, argv, "--center_z", center_z);
  pcl::console::parse_argument(argc, argv, "--bb_radius", deviation);

  pcl::console::parse_argument(argc, argv, "--dir", dir);

	// Read a PCD file from disk.
  std::vector< std::string > files;
  files = getPcdFilesInDir(dir);

  if (files.size() == 0)
  {
    showHelp(argv[0]);
    exit(0);
  }

  std::string original_path = dir + "\\original_clouds\\";
  std::string segmented_path = dir + "\\segmented_clouds\\";

  std::string original_clouds_dir = dir + "\\original_clouds\\pcd\\";
  std::string original_clouds_dir_ply = dir + "\\original_clouds\\ply\\";
  std::string segmented_clouds_dir = dir + "\\segmented_clouds\\pcd\\";
  std::string segmented_clouds_dir_ply = dir + "\\segmented_clouds\\ply\\";
  std::string segmentation_masks_dir = dir + "\\segmentation_masks\\";
  std::string color_maps_dir = dir + "\\color_images\\";
  std::string depth_maps_dir = dir + "\\depth_images\\";

  boost::filesystem::path path_orig(original_path);
  if (!boost::filesystem::create_directory(path_orig)) {
    std::cout << "Error creating original_clouds folder" << "\n";
  }

  boost::filesystem::path path_segmented(segmented_path);
  if (!boost::filesystem::create_directory(path_segmented)) {
    std::cout << "Error creating segmented_clouds folder" << "\n";
  }

  boost::filesystem::path path(segmented_clouds_dir);
  if (!boost::filesystem::create_directory(path)) {
    std::cout << "Error creating segmented_clouds pcd folder" << "\n";
  }

  boost::filesystem::path path_(segmented_clouds_dir_ply);
  if (!boost::filesystem::create_directory(path_)) {
    std::cout << "Error creating segmented_clouds ply folder" << "\n";
  }

  boost::filesystem::path path2(segmentation_masks_dir);
  if (!boost::filesystem::create_directory(path2)) {
    std::cout << "Error creating segmentation_masks folder" << "\n";
  }

  boost::filesystem::path path3(original_clouds_dir);
  if (!boost::filesystem::create_directory(path3)) {
    std::cout << "Error creating original_clouds folder pcd" << "\n";
  }

  boost::filesystem::path path4(original_clouds_dir_ply);
  if (!boost::filesystem::create_directory(path4)) {
    std::cout << "Error creating original_clouds_ply folder ply" << "\n";
  }

  boost::filesystem::path path5(color_maps_dir);
  if (!boost::filesystem::create_directory(path5)) {
    std::cout << "Error creating color_maps folder" << "\n";
  }

  boost::filesystem::path path6(depth_maps_dir);
  if (!boost::filesystem::create_directory(path6)) {
    std::cout << "Error creating depth_maps folder" << "\n";
  }

  for (std::vector<std::string>::iterator it = files.begin(); it != files.end(); it++)
  {
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(*it, *cloud) != 0)
    {
      std::cout << "Error loading " << *it << std::endl;
    }

    boost::filesystem::path p(*it);
    std::string fnamecloud = original_clouds_dir + p.filename().string();
    pcl::io::savePCDFileASCII<pcl::PointXYZRGB>(fnamecloud, *cloud);

    fnamecloud = original_clouds_dir_ply + p.filename().string();
    pcl::io::savePLYFileASCII<pcl::PointXYZRGB>(fnamecloud, *cloud);

    // Passthrough using a bounding box (table center) 0.65m for Z
    float depth_limit_min = center_z - deviation;
    float depth_limit_max = center_z + deviation;

    float x_limit_min = center_x - deviation;
    float x_limit_max = center_x + deviation;

    float y_limit_min = center_y - deviation;
    float y_limit_max = center_y + deviation;

    // Bounding box filtering
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setKeepOrganized(true);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(depth_limit_min, depth_limit_max);
    pass.filter(*filtered_cloud_z);

    // passthrough Y
    pass.setInputCloud(filtered_cloud_z);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_limit_min, x_limit_max);
    pass.filter(*filtered_cloud_x);

    // passthrough Z
    pass.setInputCloud(filtered_cloud_x);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_limit_min, y_limit_max);
    pass.filter(*filtered_cloud_y);

    if (bPlaneExtraction)
    {
      aux = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

      // Object for storing the plane model coefficients.
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;
      segmentation.setInputCloud(filtered_cloud_y);
      segmentation.setModelType(pcl::SACMODEL_PLANE);
      segmentation.setMethodType(pcl::SAC_RANSAC);
      segmentation.setDistanceThreshold(0.009);
      segmentation.setMaxIterations(100);
      segmentation.setOptimizeCoefficients(true);

      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      segmentation.segment(*inliers, *coefficients);

      if (inliers->indices.size() == 0)
        std::cout << "Could not find any points that fitted the plane model." << std::endl;
      /*else
      {
        std::cerr << "Model coefficients: " << coefficients->values[0] << " "
          << coefficients->values[1] << " "
          << coefficients->values[2] << " "
          << coefficients->values[3] << std::endl;
      }*/

      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud(filtered_cloud_y);
      extract.setIndices(inliers);
      extract.setNegative(true);
      extract.setKeepOrganized(true);
      extract.filter(*aux);

      filtered_cloud_y = aux;
    }

    if (bStatisticalFiltering)
    {
      aux = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

      // Create the filtering object
      pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
      sor.setInputCloud(filtered_cloud_y);
      sor.setMeanK(70);
      sor.setStddevMulThresh(0.75);
      sor.setKeepOrganized(true);
      sor.filter(*aux);

      filtered_cloud_y = aux;
    }

    const float bad_point = std::numeric_limits<float>::quiet_NaN();
    pcl::PointXYZRGB pbad_point;
    pbad_point.x = pbad_point.y = pbad_point.z = pbad_point.r = pbad_point.g = pbad_point.b = pbad_point.a = bad_point;

    if (bChromaSegmentation)
    {
      for (size_t i = 0; i != filtered_cloud_y->size(); i++)
      {
        if (pcl::isFinite(filtered_cloud_y->points[i]))
        {
          if (filtered_cloud_y->points[i].y = 0.0f && filtered_cloud_y->points[i].z == 0.0f && filtered_cloud_y->points[i].x == 0) 
          {
            filtered_cloud_y->points[i] = pbad_point;
          }
          else
          {
            uint8_t r = filtered_cloud_y->points[i].r;  uint8_t g = filtered_cloud_y->points[i].g;  uint8_t b = filtered_cloud_y->points[i].b;
            uint8_t min = (r < g) ? r : g;
            min = (min < b) ? min : b;
            uint8_t max = (r > g) ? r : g;
            max = (max > b) ? max : b;

            bool key =
              b != min &&
              (b == max || max - b < maximumCutoff) &&
              (max - min) > differenceThreshold;

            if (key)
            { //chroma segmentation
              filtered_cloud_y->points[i] = pbad_point;
            }
          }
        }
      }
    }

    std::string fname = segmented_clouds_dir + p.filename().string();
    pcl::io::savePCDFileASCII<pcl::PointXYZRGB>(fname, *filtered_cloud_y);

    std::string fnamePBM = segmentation_masks_dir + p.stem().string() + ".pbm";
    writePBM(*filtered_cloud_y, fnamePBM);

    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(*filtered_cloud_y, *filtered_cloud_y, mapping);

    std::string fnamePLY = segmented_clouds_dir_ply + p.stem().string() + ".ply";
    pcl::io::savePLYFileASCII<pcl::PointXYZRGB>(fnamePLY, *filtered_cloud_y);

    if (!bBatch)
    {
      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
      viewer->setBackgroundColor(0.7, 0.7, 0.7);
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(filtered_cloud_y);
      viewer->addPointCloud<pcl::PointXYZRGB>(filtered_cloud_y, rgb, "sample cloud");
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
      viewer->addCoordinateSystem(0.25);
      viewer->initCameraParameters();

      while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
      }
    }
  
    // Load the input file
    pcl::PCLPointCloud2::Ptr blob(new pcl::PCLPointCloud2);
    if (!loadCloud(*it, *blob))
    {
      print_error("Unable to load PCD file.\n");
      return (-1);
    }

    // Check if the cloud is organized
    if (blob->height == 1)
    {
      print_error("Input cloud is not organized.\n");
      return (-1);
    }

    bool paint_nans_with_black = false; //pcl::console::find_switch(argc, argv, "--no-nan");
    //print_info("Paint infinite points with black: "); print_value("%s\n", paint_nans_with_black ? "YES" : "NO");

    std::string field_name = "rgb";
    parse_argument(argc, argv, "--field", field_name);
    print_info("Field name: "); print_value("%s\n", field_name.c_str());


    pcl::PCLImage image;
    bool extracted;

    // save color 
    {
      PointCloud<PointXYZRGB> cloud;
      fromPCLPointCloud2(*blob, cloud);
      PointCloudImageExtractorFromRGBField<PointXYZRGB> pcie;
      pcie.setPaintNaNsWithBlack(paint_nans_with_black);
      extracted = pcie.extract(cloud, image);
    }

    std::string png_filename_color = color_maps_dir + p.stem().string() + ".png";
    saveImage(png_filename_color, image);

    // save depth
    {
      PointCloud<PointXYZ> cloud;
      fromPCLPointCloud2(*blob, cloud);
      PointCloudImageExtractorFromZField<PointXYZ> pcie;
      pcie.setPaintNaNsWithBlack(paint_nans_with_black);
      extracted = pcie.extract(cloud, image);
    }

    std::string png_filename_depth = depth_maps_dir + p.stem().string() + ".png";
    saveImage(png_filename_depth, image);

    // removing original file from source dir.. hack! 
    // TODO remove this...
    boost::filesystem::wpath file(*it);
    if (boost::filesystem::exists(file))
      boost::filesystem::remove(file);

  }
}