# Registration Module

This module provides functions for point cloud registration using Open3D. It includes functions for preprocessing point clouds, executing global registration using RANSAC or fast global registration, and refining registration using ICP.

## Functions

- `draw_registration_result(source, target, transformation, window_name="open3d")`: Visualize registration results.
- `preprocess_point_cloud(pcd, voxel_size) -> Tuple[PointCloud, Feature]`: Downsample and compute FPFH feature for a point cloud.
- `load_dataset(source_path, target_path, voxel_size) -> Tuple[PointCloud, PointCloud, PointCloud, PointCloud, Feature, Feature]`: Load two point clouds and prepare the dataset for registration.
- `global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size, method="RANSAC") -> RegistrationResult`: Execute global registration using RANSAC or FAST.
- `refine_registration(source, target, global_transformation_result, voxel_size) -> RegistrationResult`: Refine registration using ICP.

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
    Check main.py for example usage.
    ```

## Author

Shashwat Kumar


