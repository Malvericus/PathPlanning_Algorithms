from .solution import SplinePath3D

VIOLATION_PENALTIES = {
    'start': 1.5,
    'goal': 1.5,
    'environment': 0.3,
    'collision': 0.5
}

def calculate_path_cost(sol):
    path = sol.get_path()
    length = sol.environment.path_length(path)
    violations, details = sol.environment.count_violations(path)
    
    cost = length
    if details['start_violation']:
        cost *= (1 + VIOLATION_PENALTIES['start'])
    if details['goal_violation']:
        cost *= (1 + VIOLATION_PENALTIES['goal'])
    cost *= (1 + details['env_violations'] * VIOLATION_PENALTIES['environment'])
    cost *= (1 + details['collision_violations'] * VIOLATION_PENALTIES['collision'])
    
    details.update({
        'sol': sol,
        'path': path,
        'length': length,
        'cost': cost
    })
    return cost, details

def create_cost_function(env, num_control_points=5, resolution=100):
    def cost_function(x):
        x = x.reshape(-1, 3)  # Reshape to (n,3) array
        sol = SplinePath3D(env, x, resolution)
        return calculate_path_cost(sol)
    return cost_function