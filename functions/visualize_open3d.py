import open3d as o3d
import numpy as np

def main():
    points_data = np.loadtxt("data.txt", delimiter=",", dtype=np.float32)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_data[:, :3])
    o3d.visualization.draw_geometries([pcd])

if __name__ == '__main__':
    main()

