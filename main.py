# here you import from your library "pcd_register"

import open3d as o3d
import numpy as np
from pcd_register import registration_pipeline

def main():
    """
    Main function contating the demo application of registration pipeline
    module.
    
    
    """

    # load the pointcloud dataset
    demo_icp_pcds = o3d.data.DemoICPPointClouds()
    source_path = demo_icp_pcds.paths[0]
    target_path = demo_icp_pcds.paths[1]

    # set voxel size for downsampling while 
    voxel_size = 0.05
    
    # load dataset, downsample it for global registration and calculate FPFH feature
    source, target, source_down, target_down, source_fpfh, target_fpfh\
        = registration_pipeline.load_dataset(source_path,target_path,voxel_size)
    #visualize pointcloud
    registration_pipeline.draw_registration_result(source, target, np.identity(4), "Raw PointCloud")
    

    # global registration; method can be either "RANSAC" or "FAST"
    result_global = registration_pipeline.global_registration(source_down, target_down,
                                            source_fpfh, target_fpfh,
                                            voxel_size,method="FAST")
    print(result_global)
    # visualize global registration results
    registration_pipeline.draw_registration_result(source_down, target_down, result_global.transformation,
                                          window_name = "Global Registration Results")
    
    # refine registration using ICP
    result_icp = registration_pipeline.refine_registration(source, target, 
                                                           result_global.transformation, 
                                                           voxel_size)
    print(result_icp)
    
    # visualize refine registration results
    registration_pipeline.draw_registration_result(source, target, result_icp.transformation, 
                                          window_name = "Local Registration Results")


if __name__ == "__main__":
    main()