# Registration Module

This module provides functions for point cloud registration using Open3D. It includes functions for preprocessing point clouds, executing global registration, refining registration using ICP, and performing fast global registration.

## Functions

- `draw_registration_result(source, target, transformation, window_name="open3d")`: Visualize registration results.
- `preprocess_point_cloud(pcd, voxel_size) -> Tuple[PointCloud, Feature]`: Downsample and compute FPFH feature for a point cloud.
- `prepare_dataset(voxel_size) -> Tuple[PointCloud, PointCloud, Feature, Feature]`: Load two point clouds and prepare the dataset for registration.
- `execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size, method="RANSAC") -> RegistrationResult`: Execute global registration using RANSAC or FAST.
- `refine_registration(source, target, result_ransac, voxel_size) -> RegistrationResult`: Refine registration using ICP.
- `execute_fast_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size) -> RegistrationResult`: Execute fast global registration.

## Dependencies

- open3d
- numpy



## How to Run

1. Ensure you have the required dependencies installed. You can install them using:

    ```bash
    pip install open3d numpy
    ```

2. Clone the repository or download the Registration Module file.

3. Create a Python script or use an interactive Python environment.

4. Import the module and use the provided functions

    ```
    # Example usage
    ```



## Author

Shashwat Kumar


