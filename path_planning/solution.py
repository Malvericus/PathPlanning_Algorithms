import numpy as np
from scipy.interpolate import CubicSpline

class SplinePath3D:
    """3D cubic spline path through control points"""
    
    def __init__(self, environment, control_points=None, resolution=100):
        self.environment = environment
        self.control_points = np.array(control_points) if control_points is not None else None
        self.resolution = resolution
        
    @staticmethod
    def random(environment, num_control_points=5, resolution=100):
        control_points = np.random.rand(num_control_points, 3) * [
            environment.width, environment.height, environment.depth
        ]
        return SplinePath3D(environment, control_points, resolution)
    
    def get_path(self):
        if self.control_points is None:
            return np.array([self.environment.start, self.environment.goal])
            
        points = np.vstack([self.environment.start, 
                          self.control_points, 
                          self.environment.goal])
        
        t = np.linspace(0, 1, len(points))
        cs = CubicSpline(t, points, bc_type='clamped')
        path = cs(np.linspace(0, 1, self.resolution))
        return self.environment.clip_path(path)

# Explicitly export the class
__all__ = ['SplinePath3D']