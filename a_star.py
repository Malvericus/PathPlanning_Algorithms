import numpy as np
from heapq import heappush, heappop
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

class AStar3D:
    def __init__(self, env, grid_resolution=5):
        self.env = env
        self.res = grid_resolution
        
    def plan(self, callback=None):
        # Convert to grid coordinates
        start = (self.env.start / self.res).astype(int)
        goal = (self.env.goal / self.res).astype(int)
        
        open_set = []
        heappush(open_set, (0, tuple(start)))
        came_from = {}
        g_score = {tuple(start): 0}
        visited = set()
        
        while open_set:
            _, current = heappop(open_set)
            visited.add(current)
            
            if callback and (len(visited) % 50 == 0):
                callback({
                    'iteration': len(visited),
                    'current': current,
                    'visited': visited,
                    'came_from': came_from
                })
            
            if np.allclose(current, goal):
                path = self._reconstruct_path(came_from, current)
                return path * self.res
                
            for neighbor in self._get_neighbors(current):
                if neighbor in visited:
                    continue
                    
                tentative_g = g_score[current] + self.res
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self._heuristic(neighbor, goal)
                    heappush(open_set, (f_score, neighbor))
        
        return None

    def _heuristic(self, a, b):
        return np.linalg.norm(np.array(a) - np.array(b))
    
    def _get_neighbors(self, pos):
        neighbors = []
        for dx, dy, dz in [(1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1)]:
            neighbor = (pos[0]+dx, pos[1]+dy, pos[2]+dz)
            grid_point = np.array(neighbor) * self.res
            if self.env.in_environment(grid_point) and not self.env.in_collision(grid_point):
                neighbors.append(neighbor)
        return neighbors
    
    def _reconstruct_path(self, came_from, current):
        path = [np.array(current) * self.res]
        while current in came_from:
            current = came_from[current]
            path.append(np.array(current) * self.res)
        return np.array(path[::-1])

def astar_callback(env, data):
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect([1, 1, 1])
    
    # Environment
    ax.scatter(*env.start, color='red', s=150)
    ax.scatter(*env.goal, color='green', s=150)
    for obs in env.obstacles:
        u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
        x = obs.center[0] + obs.radius * np.cos(u) * np.sin(v)
        y = obs.center[1] + obs.radius * np.sin(u) * np.sin(v)
        z = obs.center[2] + obs.radius * np.cos(v)
        ax.plot_surface(x, y, z, color='black', alpha=0.5)
    
    # Visited nodes
    visited = np.array(list(data['visited'])) * 5
    ax.scatter(visited[:,0], visited[:,1], visited[:,2], 
               color='blue', s=10, alpha=0.3)
    
    # Current best path
    current = np.array(data['current']) * 5
    ax.scatter(*current, color='orange', s=100)
    
    ax.set_title(f"A* Iteration: {data['iteration']}")
    plt.savefig(f'astar_frames/frame_{data["iteration"]:04d}.png')
    plt.close()