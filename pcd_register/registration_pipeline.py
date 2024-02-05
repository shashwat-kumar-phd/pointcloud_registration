"""
Registration Module

This module provides functions for point cloud registration using Open3D.
It includes functions for preprocessing point clouds, executing global 
registration, refining registration using ICP, and performing fast global 
registration.

Functions:
- draw_registration_result(source, target, transformation): Visualize registration
 results.
- preprocess_point_cloud(pcd, voxel_size): Downsample and compute FPFH feature
 for a point cloud.
- prepare_dataset(voxel_size): Load two point clouds and prepare the dataset
 for registration.
- execute_global_registration(source_down, target_down, source_fpfh, target_fpfh,
 voxel_size): 
  Execute global registration using RANSAC.
- refine_registration(source, target, source_fpfh, target_fpfh, result_ransac,
 voxel_size): 
  Refine registration using ICP.
- execute_fast_global_registration(source_down, target_down, source_fpfh,
 target_fpfh, voxel_size): 
  Execute fast global registration.

Dependencies:
- open3d
- numpy
- RANSAC from open3d
- copy

Author: Shashwat Kumar
"""

import open3d as o3d
from open3d.geometry import PointCloud
from open3d.pipelines.registration import Feature, RegistrationResult
import numpy as np
import copy
from typing import Tuple

def draw_registration_result(source: PointCloud, target: PointCloud,
                              transformation: np.ndarray, window_name: str = "open3d") -> None:
    """
    Visualize the registration result.

    Parameters:
    - source: PointCloud
        Source point cloud.
    - target: PointCloud
        Target point cloud.
    - transformation: ndarray
        Transformation matrix.
    """

    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom = 0.4559,
                                      front = [0.6452, -0.3036, -0.7011],
                                      lookat = [1.9892, 2.0208, 1.8945],
                                      up = [-0.2779, -0.9482, 0.1556],
                                      window_name=window_name
                                      )

def preprocess_point_cloud(pcd: PointCloud, voxel_size: float) -> Tuple[PointCloud, 
                                                                        Feature]:
    """
    Preprocess the point cloud by downsampling and computing FPFH feature.

    Parameters:
    - pcd: open3d.geometry.PointCloud
        Input point cloud.
    - voxel_size: float
        Voxel size for downsampling.

    Returns:
    - Tuple[open3d.geometry.PointCloud , open3d.pipelines.registration.Feature]
        Tuple containing downsampled point cloud and FPFH feature.
    """

    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius = radius_normal, max_nn = 30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius = radius_feature, max_nn = 100))
    return pcd_down, pcd_fpfh

def load_dataset(source_path: str, target_path: str, voxel_size: float) -> Tuple[
                                             PointCloud, PointCloud, 
                                             PointCloud, PointCloud, 
                                             Feature, Feature]:
    """
    Load two point clouds and prepare the dataset for registration.

    Parameters:
    - source_path: str
        path of source PointCloud file
    - target_path: str
        path of source PointCloud file
    - voxel_size: float
        Voxel size for downsampling.

    Returns:
    - Tuple[o3d.geometry.PointCloud, o3d.geometry.PointCloud, 
    o3d.geometry.PointCloud, o3d.geometry.PointCloud, 
    o3d.pipelines.registration.Feature, 
    o3d.pipelines.registration.Feature]
        Tuple containing source, target, downsampled source, downsampled target,
      source FPFH, and target FPFH.
    """

    print("Load two point clouds")
    source = o3d.io.read_point_cloud(source_path)
    target = o3d.io.read_point_cloud(target_path)
    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh


def global_registration(source_down: PointCloud, 
                                target_down: PointCloud,
                                source_fpfh: Feature,
                                target_fpfh: Feature,
                                voxel_size: float, method: str = "RANSAC") -> RegistrationResult:
    """
    Execute global registration using RANSAC or FAST (default RANSAC).

    Parameters:
    - source_down: open3d.geometry.PointCloud
        Downsampled source point cloud.
    - target_down: open3d.geometry.PointCloud
        Downsampled target point cloud.
    - source_fpfh: open3d.pipelines.registration.Feature
        FPFH feature of the source point cloud.
    - target_fpfh: open3d.pipelines.registration.Feature
        FPFH feature of the target point cloud.
    - voxel_size: float
        Voxel size for downsampling.
    - method: str, optional
        Global registration method, valid values "RANSAC", "FAST" (default RANSAC)
    - open3d.pipelines.registration.RegistrationResult
        The result of the registration process, containing transformation information and other details.
    """

    distance_threshold = voxel_size * 1.5
    if method =="RANSAC":    
        print(f"RANSAC registration on downsampled point clouds with distance \
              threshold {distance_threshold:.3f}")
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh, True, distance_threshold,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            3, [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)], 
                o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
        return result
    elif method == "FAST":
        print(f"Apply fast global registration with distance threshold {distance_threshold:.3f}")
        result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh,
            o3d.pipelines.registration.FastGlobalRegistrationOption(
                maximum_correspondence_distance=distance_threshold))
        return result
    else:
        print(f"Invalid method type. Valid types are \"RANSAC\" or \"FAST\"")
        return np.nan

def refine_registration(source: PointCloud, target: PointCloud, global_result: np.ndarray,
                        voxel_size: float) -> RegistrationResult:
    """
    Refine registration using ICP.

    Parameters:
    - source: o3d.geometry.PointCloud
        Source point cloud.
    - target: o3d.geometry.PointCloud
        Target point cloud.
    - global_result: ndarray
        Transformation matrix from global registration results.
    - voxel_size: float
        Voxel size for downsampling.

    Returns:
    - o3d.pipelines.registration.RegistrationResult
        The result of the refine registration process, containing transformation information and other details.
    """

    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    # result = o3d.pipelines.registration.registration_icp(
        # source, target, distance_threshold, result_ransac.transformation,
        # o3d.pipelines.registration.TransformationEstimationPointToPlane())
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, global_result,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result