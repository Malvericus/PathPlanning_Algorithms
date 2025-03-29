import numpy as np

class Environment:
    """Class to represent the 3D environment with obstacles."""
    
    def __init__(self, width=100, height=100, depth=100, robot_radius=0, obstacles=[], start=None, goal=None):
        self.width = width
        self.height = height
        self.depth = depth
        self.robot_radius = robot_radius
        self.obstacles = obstacles
        self.start = np.array(start) if start is not None else None
        self.goal = np.array(goal) if goal is not None else None

    def add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)
    
    def clear_obstacles(self):
        """Remove all obstacles"""
        self.obstacles = []

    def in_collision(self, point):
        for obstacle in self.obstacles:
            if obstacle.in_collision(point, self.robot_radius):
                return True
        return False

    def path_in_collision(self, path):
        for point in path:
            if self.in_collision(point):
                return True
        return False

    def in_environment(self, point):
        return (0 <= point[0] <= self.width and 
                0 <= point[1] <= self.height and 
                0 <= point[2] <= self.depth)

    def clip_point(self, point):
        return np.array([
            np.clip(point[0], 0, self.width),
            np.clip(point[1], 0, self.height),
            np.clip(point[2], 0, self.depth)
        ])

    def clip_path(self, path):
        return np.array([self.clip_point(p) for p in path])

    def in_goal(self, point):
        return np.linalg.norm(point - self.goal) <= self.robot_radius

    def in_start(self, point):
        return np.linalg.norm(point - self.start) <= self.robot_radius

    def count_violations(self, path):
        violations = 0
        details = {
            'start_violation': not self.in_start(path[0]),
            'goal_violation': not self.in_goal(path[-1]),
            'env_violations': 0,
            'collision_violations': 0
        }
        
        for point in path:
            if not self.in_environment(point):
                details['env_violations'] += 1
            if self.in_collision(point):
                details['collision_violations'] += 1
                
        violations = (details['start_violation'] + details['goal_violation'] + 
                     details['env_violations'] + details['collision_violations'])
        return violations, details

    def path_length(self, path):
        return sum(np.linalg.norm(path[i+1] - path[i]) for i in range(len(path)-1))

class Obstacle:
    """3D obstacle (sphere)"""
    def __init__(self, center, radius):
        self.center = np.array(center)
        self.radius = radius
    
    def in_collision(self, point, robot_radius=0):
        return np.linalg.norm(point - self.center) <= self.radius + robot_radius