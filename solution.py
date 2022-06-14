import open3d as o3d
import numpy as np

#reading oint cloud and mesh data from test file
pcd = o3d.io.read_point_cloud("test_mesh.ply")
mesh = o3d.io.read_triangle_mesh("test_mesh.ply")
z= np.asarray(pcd.colors) # reading color of each point

r1 = z[:,0]
g1 = z[:,1]
b1 = z[:,2]

r= []
g = []
b= []
for i in range(len(r1)):
    if(r1[i]==1):
        r.append(i)
    if(g1[i]==1):
        g.append(i)
    if(b1[i]==1):
        b.append(i)
        
points = np.asarray(mesh.triangles) #triangles

#identifying connected points for each color
def identical_connected_points(points, r,g,b):
    max = np.max(points)
    num = max + 1
    len_p = len(points)
    arr = np.array([[0,0,0]])
    lt = np.zeros((abs(num-len_p), 3))
    for j in range((num-len_p)):
        for i in range(3):
            lt[j,i] = 0
    point=np.append(points,lt,axis=0)
    
    Adj = np.zeros((np.max(points)+1, np.max(points)+1)) #matrix to store the connections 
    a = np.zeros(3)
    for i in range(0,np.max(points)+1):
        #print(i)
        for j in range(3):
            a[j] = point[i,j]
        Adj[np.int(a[0]),np.int(a[1])] = 1
        Adj[np.int(a[0]),np.int(a[2])] = 1
        Adj[np.int(a[1]),np.int(a[0])] = 1
        Adj[np.int(a[2]),np.int(a[0])] = 1
        Adj[np.int(a[1]),np.int(a[2])] = 1
        Adj[np.int(a[2]),np.int(a[1])] = 1
        Adj[0,0] = 0
        
        
    red = []

    for i in range(len(r)):
        for j in range(len(r)):
            if(Adj[np.int(r[i]), np.int(r[j])] == 1):
                red.append(r[j]) #red connected points

    res_red = []
    for i in red:
        if i not in res_red:
            res_red.append(i)
    res_red.sort()
    print("connected red points:",res_red)
    
    blue = []
    for i in range(len(b)):
        for j in range(len(b)):
            if(Adj[np.int(b[i]), np.int(b[j])] == 1):
                blue.append(b[j]) #blue connected points
    if(len(blue) == 0):
        blue = b

    res_blue = []
    for i in blue:
        if i not in res_blue:
            res_blue.append(i)
    res_blue.sort()
    print("connected blue points:",res_blue)
    
    green = []
    for i in range(len(g)):
        for j in range(len(g)):
            if(Adj[np.int(g[i]), np.int(g[j])] == 1):
                green.append(g[j]) #green connected points

    res_green = []
    for i in green:
        if i not in res_green:
            res_green.append(i)
    res_green.sort()
    print("connected green points:",res_green) 
    return res_red, res_green, res_blue

[red, green, blue] = identical_connected_points(points, r,g,b)
np.savetxt('result.txt', np.array([red, green, blue]).T, delimiter='\t', fmt="%s") # saving results to result.txt