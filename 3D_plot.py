import cv2
import matplotlib.pyplot as plt
import glob
cv_img = []

"""
Reading filtered images and saving
in 'cv_img' array
"""
for i in range(1,17):
  for img in glob.glob(f"output/crop{i}.png"):
    n= cv2.imread(img)
    cv_img.append(n)


z_cordinate=[]
x_cordinate=[]
for i in range(0,len(cv_img)):
  h,w,_=cv_img[i].shape
  z=[]
  x=[]
  
  """
  Iterating each pixel and appending locations where edge exits

  get image (2d plane)
  get angle of image (theta)
    if angle overlap --> some x,z pixels will be ignored

  """

  for l in range(0,h):
    for j in range(0,w):
      b,g,r=cv_img[i][l,j]
      if (b,g,r)!=(0,0,0):
        z.append(j)
        x.append(l)
  #z.append(h)
  #x.append(w)
  z_cordinate.append(z)
  x_cordinate.append(x)


s=1.5
y_axis=[]
# Y-axis given range 16 for 16 croped imgs in "output/"
# for only 3 imgs in crop set range(1, 3)

# for i in range(1, 3):
for i in range(1,17):
  y_axis.append(s)
  s=s+1.5



import math
import numpy as np



from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

fig = plt.figure()

# syntax for 3-D projection
ax = plt.axes(projection ='3d')

# defining axes
test=[]

print(f'length of y_axis: {len(y_axis)}')
print(y_axis)
print(f'length of x_axis: {len(x_cordinate)}')
print(f'length of z_axis: {len(z_cordinate)}')

for i in range(0,len(y_axis)):
  ax.scatter(x_cordinate[i], y_axis[i], z_cordinate[i],  color='red',s=1)

  for k in range(len(x_cordinate[i])):
    test.append([x_cordinate[i][k],y_axis[i], z_cordinate[i][k]])


# syntax for plotting
# ax.set_title('3d Scatter conical flask') 
# ax.set_xlabel('X Label')
# ax.set_ylabel('Y Label')
# ax.set_zlabel('Z Label')
# ax.view_init(30, -60)
# plt.show()


# Pass numpy array to Open3D.o3d.geometry.PointCloud and visualize
# import open3d as o3d
# #xyz = np.random.rand(100, 3)
# pcd = o3d.geometry.PointCloud()
# pcd.points = o3d.utility.Vector3dVector(test)
# o3d.io.write_point_cloud('data2.ply', pcd)
# o3d.visualization.draw_geometries([pcd])

"""
modeling on pyvista
"""
import numpy as np
import pyvista as pv

# points is a 3D numpy array (n_points, 3) coordinates of a sphere
cloud = pv.PolyData(test)
#cloud.plot()

volume = cloud.delaunay_3d(alpha=5., progress_bar=True)
shell = volume.extract_geometry()
shell.plot()