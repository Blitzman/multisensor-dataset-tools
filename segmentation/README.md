# README #

Tool for object segmentation. Objects were captured using differents 3D sensors (Primesense carmine 1.09, Kinect v1 and Kinect v2). In addition, we used an electronic turntable and a blue chroma background to help segmenting the object from the scene.

Instructions :


```
#!c++

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
```
