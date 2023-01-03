import open3d as o3d
import numpy as np
import pandas as pd

def filterPoints(path, output, config):
    # Load xyz values from config
    xMin,yMin,zMin = map(int,config['filter']['xyzMin'].split(','))
    xMax,yMax,zMax = map(int,config['filter']['xyzMax'].split(','))
    # Load colour values from config
    rMin,gMin,bMin = map(int,config['filter']['rgbMin'].split(','))
    rMax,gMax,bMax = map(int,config['filter']['rgbMax'].split(','))
    # Voxel size for simplification
    voxelSize = int(config['filter']['sampleSize'])

    # removing the new line characters
    with open(path) as f:
        lines = [line.rstrip() for line in f]

    df = None
    header = []

    # Load ply file
    for i in range(0, len(lines)):
        if len(lines[i].split(" ")) > 5:
            df = np.asarray([l.split(" ") for l in lines[i:]])
            df = pd.DataFrame(df).astype(float)
            break
        else:
            header.append(lines[i])

    # Filter xyz
    df = df[(df[2] > zMin) & (df[2] < zMax) & (df[1] > yMin) & (df[1] < yMax) & (df[0] < xMax) & (df[0] > xMin)]
    # Filter colour
    #df = df[(df[6] > rMin) & (df[7] > gMin) & (df[8] > bMin) & (df[6] < rMax) & (df[7] < gMax) & (df[8] < bMax)]

    # Revert back
    df = df.to_numpy()

    # convert to open 3d
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(df[:,0:3])
    pcd.colors = o3d.utility.Vector3dVector(df[:,6:] / 255)

    # Simplify
    pcd = pcd.voxel_down_sample(voxel_size=voxelSize)

    # Visualise
    #o3d.visualization.draw_geometries([pcd])

    # Create file and write header
    with open(output, "w") as f:
        for line in header:
            if "element vertex" in line:
                f.write("element vertex " + str(len(df)) +  "\n")
            else:
                f.write(line + "\n")

    # Append data
    with open(output,'a') as f:
        np.savetxt(f, df, delimiter=' ', fmt='%1.2f %1.2f %1.2f %1.4f %1.4f %1.4f %i %i %i')
