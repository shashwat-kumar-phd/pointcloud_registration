import unittest
import open3d as o3d
import numpy as np
import sys
import os

# Assuming that your src folder is one level up from the tests folder
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from pcd_register.registration_pipeline import (
    draw_registration_result,
    preprocess_point_cloud,
    load_dataset,
    global_registration,
    refine_registration,
)

class TestRegistrationModule(unittest.TestCase):

    def setUp(self):
        # Feature parameters
        self.voxel_size = 0.05
        demo_icp_pcds = o3d.data.DemoICPPointClouds()  
        self.source_path = demo_icp_pcds.paths[0]
        self.target_path = demo_icp_pcds.paths[1]
        # Source and target point clouds
        self.source_cloud = o3d.io.read_point_cloud(self.source_path)
        self.target_cloud = o3d.io.read_point_cloud(self.target_path)
        self.source_down , self.source_fpfh = preprocess_point_cloud(self.source_cloud, 
                                                                     self.voxel_size)
        self.target_down, self.target_fpfh = preprocess_point_cloud(self.target_cloud, 
                                                                    self.voxel_size)

    def test_draw_registration_result(self):
        # Ensure draw_registration_result does not raise any errors
        draw_registration_result(self.source_cloud, self.target_cloud, np.eye(4),
                                 window_name = "test")

    def test_preprocess_point_cloud(self):
        # Test preprocess_point_cloud function does not raise any error
        downsampled_cloud, fpfh_feature = preprocess_point_cloud(self.source_cloud, 
                                                                 self.voxel_size)

        # Assert the types of returned values
        self.assertIsInstance(downsampled_cloud, o3d.geometry.PointCloud)
        self.assertIsInstance(fpfh_feature, o3d.pipelines.registration.Feature)

    def test_load_dataset(self):
        # Test prepare_dataset function
        source, target, source_down, target_down, source_fpfh, target_fpfh = load_dataset(
            self.source_path, self.target_path, self.voxel_size
        )

        # Assert the types of returned values
        self.assertIsInstance(source, o3d.geometry.PointCloud)
        self.assertIsInstance(target, o3d.geometry.PointCloud)
        self.assertIsInstance(source_down, o3d.geometry.PointCloud)
        self.assertIsInstance(target_down, o3d.geometry.PointCloud)
        self.assertIsInstance(source_fpfh, o3d.pipelines.registration.Feature)
        self.assertIsInstance(target_fpfh, o3d.pipelines.registration.Feature)

    def test_execute_global_registration(self):
        # Test global_registration function with RANSAC
        result_ransac = global_registration( self.source_down, self.target_down, 
                                            self.source_fpfh, self.target_fpfh,
                                            self.voxel_size, method = "RANSAC")
        # Assert the type of the result
        self.assertIsInstance(result_ransac, o3d.pipelines.registration.RegistrationResult)

        # Test global_registration function with RANSAC
        result_fast = global_registration( self.source_down, self.target_down, 
                                          self.source_fpfh, self.target_fpfh,
                                          self.voxel_size, method = "RANSAC")
        # Assert the type of the result
        self.assertIsInstance(result_fast, o3d.pipelines.registration.RegistrationResult)

    def test_refine_registration(self):
        # Test refine_registration function
        global_result = global_registration(self.source_cloud, self.target_cloud,
                                            self.source_fpfh, self.target_fpfh, 
                                            self.voxel_size, method = "FAST")
        result_refined = refine_registration( self.source_cloud, self.target_cloud, 
                                            global_result.transformation, self.voxel_size)

        # Assert the type of the result
        self.assertIsInstance(result_refined, o3d.pipelines.registration.RegistrationResult)

if __name__ == '__main__':
    unittest.main()