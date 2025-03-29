import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3D

def plot_environment_3d(env, ax=None):
    if ax is None:
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
    
    # Plot start and goal
    ax.scatter(*env.start, color='green', s=100, label='Start')
    ax.scatter(*env.goal, color='red', s=100, label='Goal')
    
    # Plot obstacles as wireframe spheres
    for obs in env.obstacles:
        u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
        x = obs.center[0] + obs.radius * np.cos(u) * np.sin(v)
        y = obs.center[1] + obs.radius * np.sin(u) * np.sin(v)
        z = obs.center[2] + obs.radius * np.cos(v)
        ax.plot_wireframe(x, y, z, color='blue', alpha=0.3)
    
    ax.set_xlim(0, env.width)
    ax.set_ylim(0, env.height)
    ax.set_zlim(0, env.depth)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    return ax

def plot_path_3d(sol, ax=None, color='b', linewidth=2):
    path = sol.get_path()
    if ax is None:
        ax = plt.gca()
    
    line = Line3D(path[:,0], path[:,1], path[:,2], 
                 color=color, linewidth=linewidth)
    ax.add_line(line)
    return line

def update_path_3d(sol, line):
    path = sol.get_path()
    line.set_data(path[:,0], path[:,1])
    line.set_3d_properties(path[:,2])
    plt.draw()