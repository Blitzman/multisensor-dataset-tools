Tool for object registration. PCD Point clouds taken from different points of view are provided to generate a 360-registered point cloud and a reconstructed mesh of the object.

Instructions:

```
#!c++
Allowed options:
  --help                                produce help message
  --name arg (=noname)                  Name for the session
  --v arg (=1)                          Activate viewer
  --r arg (=-1)                         Rotation modifier
  --dir arg                             PCD files directory
  --n arg (=64)                         Number of clouds
  --s arg (=5.625)                      Step rotation angle
  --c_x arg (=0.001557)                 Ground truth table center X
  --c_y arg (=0.136046)                 Ground truth table center Y
  --c_z arg (=-0.80500000000000005)     Ground truth table center Z
  --n_x arg (=0.021264000000000002)     Ground truth table normal X
  --n_y arg (=-0.92883000000000004)     Ground truth table normal Y
  --n_z arg (=0.36989499999999997)      Ground truth table normal Z
  --ne_k arg (=25)                      Normal estimation neighbors
  --ne_d_k arg (=25)                    Normal estimation neighbors
                                        (downsampled cloud)
  --sor arg (=1)                        Activate Statistical Outlier Removal
  --sor_mk arg (=50)                    SOR mean K
  --sor_sd arg (=0.899999976)           SOR standard deviation
  --ror arg (=0)                        Activate Radial Outlier Removal
  --ror_n arg (=10)                     ROR number of neighbors
  --ror_r arg (=0.00800000038)          ROR search radius
  --ece arg (=1)                        Activate Euclidean Cluster Extraction
  --ece_t arg (=0.0199999996)           ECE cluster tolerance
  --ece_mns arg (=100)                  ECE minimum cluster size
  --ece_mxs arg (=25000)                ECE maximum cluster size
  --ece_c arg (=1)                      ECE clusters selected
  --vg arg (=0)                         Activate Voxel Grid downsampling
  --vg_ls arg (=0.0500000007)           VG leaf size
  --pmr arg (=0)                        Activate Poisson Mesh Reconstruction
  --pmr_mls_sr arg (=0.00999999978)     PMR MLS search radius
  --pmr_mls_pf arg (=1)                 PMR MLS polynomial fit
  --pmr_mls_po arg (=2)                 PMR MLS polynomial order
  --pmr_mls_ur arg (=0.00499999989)     PMR MLS upsampling radius
  --pmr_mls_us arg (=0.00300000003)     PMR MLS step size
  --pmr_ne_rs arg (=0.00999999978)      PMR NE search radius
  --pmr_p_d arg (=9)                    PMR Poisson depth
  --icp arg (=0)                        Activate Iterative Closest Point
  --icp_i arg (=1000)                   ICP maximum iterations
  --icp_e arg (=9.9999999999999995e-007)
                                        ICP epsilon
  --cut arg (=0)                        Activate Cloud Cut
  --cut_a arg (=50)                     CUT amount
  --bin arg (=0)                        Save clouds in binary format
```