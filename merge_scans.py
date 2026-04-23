import open3d as o3d
import numpy as np

def register_gicp(source, target, threshold=0.15):
    source.estimate_normals()
    target.estimate_normals()

    trans_init = np.eye(4)
    trans_init[0, 3] = 0.60  # initial X offset guess (metres)
    trans_init[1, 3] = 0.0
    trans_init[2, 3] = 0.0

    reg_gicp = o3d.pipelines.registration.registration_generalized_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
    )
    print(f"  fitness={reg_gicp.fitness:.4f}  rmse={reg_gicp.inlier_rmse*1000:.1f}mm  "
          f"t={np.linalg.norm(reg_gicp.transformation[:3,3]):.4f}m")
    return reg_gicp.transformation  # <-- return the 4x4 matrix, not the result object

# 1. Load scans
p1 = o3d.io.read_point_cloud("./scans/pos1.pcd")
p2 = o3d.io.read_point_cloud("./scans/pos2.pcd")
p3 = o3d.io.read_point_cloud("./scans/pos3.pcd")

# 2. Register p2 to p2
t21 = register_gicp(p2, p1)
p2.transform(t21)

# 3. Register p3 to the (now aligned) p2 or p1
combined_12 = p1+ p2
t31 = register_gicp(p3, combined_12)
p3.transform(t31)

# 4. Merge and Save
final_cloud = p1 + p2 + p3
# Optional: Downsample to remove redundant points in overlap areas
final_cloud = final_cloud.voxel_down_sample(voxel_size=0.005)


o3d.io.write_point_cloud("merged.pcd", final_cloud)
print("Merged point cloud saved as merged.pcd")