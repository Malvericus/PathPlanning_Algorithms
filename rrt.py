import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class RRT:
    def __init__(self, env, max_nodes=2000, step_size=7, goal_sample_rate=0.1):
        self.env = env
        self.max_nodes = max_nodes
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        
    def plan(self, callback=None):
        nodes = [np.array(self.env.start)]
        parents = [-1]
        final_path = None
        
        for iteration in range(self.max_nodes):
            # Sample random point
            if np.random.rand() < self.goal_sample_rate:
                rand_point = np.array(self.env.goal)
            else:
                rand_point = np.random.rand(3) * [self.env.width, self.env.height, self.env.depth]
            
            # Find nearest node
            nearest_idx = np.argmin([np.linalg.norm(node - rand_point) for node in nodes])
            nearest = nodes[nearest_idx]
            
            # Steer toward random point
            direction = (rand_point - nearest)
            dist = np.linalg.norm(direction)
            if dist > 0:
                direction = direction / dist
                new_point = nearest + direction * min(self.step_size, dist)
                
                # Collision check
                if not self.env.path_in_collision([nearest, new_point]):
                    nodes.append(new_point)
                    parents.append(nearest_idx)
                    
                    # Visualization
                    if callback:
                        callback({
                            'iteration': iteration,
                            'nodes': nodes,
                            'parents': parents,
                            'new_point': new_point,
                            'path_found': False
                        })
                    
                    # Goal check
                    if np.linalg.norm(new_point - self.env.goal) < self.step_size:
                        final_path = self._build_path(nodes, parents)
                        if callback:
                            callback({
                                'iteration': iteration,
                                'nodes': nodes,
                                'parents': parents,
                                'path': final_path,
                                'path_found': True
                            })
                        return final_path
        
        # Return closest path if goal not reached
        if final_path is None:
            closest_idx = np.argmin([np.linalg.norm(node - self.env.goal) for node in nodes])
            final_path = self._build_path(nodes, parents, closest_idx)
            if callback:
                callback({
                    'iteration': self.max_nodes,
                    'nodes': nodes,
                    'parents': parents,
                    'path': final_path,
                    'path_found': False
                })
        
        return final_path

    def _build_path(self, nodes, parents, end_idx=None):
        path = []
        current = len(nodes)-1 if end_idx is None else end_idx
        while current != -1:
            path.append(nodes[current])
            current = parents[current]
        return np.array(path[::-1])
    
def compute_path_length(path):
    return np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))

def path_smoothness(path):
    if len(path) < 3:
        return 0  # No turns = perfectly smooth
    angles = []
    for i in range(1, len(path) - 1):
        v1 = path[i] - path[i - 1]
        v2 = path[i + 1] - path[i]
        angle = np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
        angles.append(angle)
    return np.mean(angles)

def rrt_callback(env, data, evaluation_metrics):
    fig = plt.figure(figsize=(12, 9), dpi=100)
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect([1, 1, 1])

    # Plot start and goal
    ax.scatter(*env.start, color='red', s=200, label='Start')
    ax.scatter(*env.goal, color='green', s=200, label='Goal')

    # Plot obstacles
    for obs in env.obstacles:
        u, v = np.mgrid[0:2*np.pi:50j, 0:np.pi:50j]
        x = obs.center[0] + obs.radius * np.cos(u) * np.sin(v)
        y = obs.center[1] + obs.radius * np.sin(u) * np.sin(v)
        z = obs.center[2] + obs.radius * np.cos(v)
        ax.plot_surface(x, y, z, color='black', alpha=0.5, edgecolor='k', linewidth=0.3)

    # Plot RRT tree
    nodes = data['nodes']
    parents = data['parents']
    for i in range(1, len(nodes)):
        ax.plot([nodes[i][0], nodes[parents[i]][0]],
                [nodes[i][1], nodes[parents[i]][1]],
                [nodes[i][2], nodes[parents[i]][2]],
                'b-', linewidth=1, alpha=0.5)

    # Highlight new point
    if 'new_point' in data:
        ax.scatter(*data['new_point'], color='orange', s=50)

    # Check if path exists
    path = None  # Initialize path variable
    if 'path' in data and data.get('path_found', False):
        path = np.array(data['path'])
        ax.plot(path[:, 0], path[:, 1], path[:, 2], 'r-', linewidth=3, label='Optimal Path')
        ax.scatter(path[:, 0], path[:, 1], path[:, 2], color='red', s=30)

    ax.set_title(f"RRT Iteration: {data['iteration']}")
    ax.legend()
    plt.tight_layout()
    plt.savefig(f'rrt_frames/frame_{data["iteration"]:04d}.png', dpi=120)
    plt.close()

    # Only compute metrics if path exists
    if path is not None:
        path_length = compute_path_length(path)
        smoothness = path_smoothness(path)

        evaluation_metrics['RRT'].append({
            'iteration': len(evaluation_metrics['RRT']) + 1,
            'path_length': path_length,
            'smoothness': smoothness
        })

        print(f"RRT Iteration {len(evaluation_metrics['RRT'])}: Path Length = {path_length:.2f}, Smoothness = {smoothness:.2f}")
    else:
        print(f"RRT Iteration {data['iteration']}: No path found yet.")