import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import seaborn as sns
import os
from PIL import Image

# Create output directory
output_dir = "analysis_results"
os.makedirs(output_dir, exist_ok=True)

results = {
    "PSO": {"Path Length": 141.79, "Smoothness": 0.59, "Time (s)": 153.78},
    "RRT": {"Path Length": 190.00, "Smoothness": 0.42, "Time (s)": 80.93},
    "A*": {"Path Length": 1200.00, "Smoothness": 1.44, "Time (s)": 41.21},
}

# Generate 3D paths for visualization
def generate_random_3d_path(length, seed=0):
    """Generate a random 3D path for demonstration."""
    np.random.seed(seed)
    return np.cumsum(np.random.randn(length, 3) * 10, axis=0)

paths = {
    "PSO": generate_random_3d_path(50, seed=1),
    "RRT": generate_random_3d_path(70, seed=2),
    "A*": generate_random_3d_path(40, seed=3),
}

# 3D Path Quality Plot
fig = plt.figure(figsize=(18, 6))
for i, (algo, path) in enumerate(paths.items()):
    ax = fig.add_subplot(1, 3, i + 1, projection='3d')
    ax.plot(path[:, 0], path[:, 1], path[:, 2], 'r-', lw=2)
    ax.set_title(f"{algo} Path\nLength: {results[algo]['Path Length']:.2f}")
    ax.set_xlabel("X Axis")
    ax.set_ylabel("Y Axis")
    ax.set_zlabel("Z Axis")
plt.savefig(f"{output_dir}/path_quality.png")
plt.close()

# Convert results to structured format
algorithms = list(results.keys())
metrics = ["Time (s)", "Path Length", "Smoothness"]
values = {metric: [results[algo][metric] for algo in algorithms] for metric in metrics}

# Generate bar plots for comparison
for metric in metrics:
    plt.figure(figsize=(8, 5))
    sns.barplot(x=algorithms, y=values[metric], palette="viridis")
    plt.ylabel(metric)
    plt.title(f"Comparison of {metric}")
    plt.savefig(f"{output_dir}/{metric.lower().replace(' ', '_')}_comparison.png")
    plt.close()

def is_inside_obstacle(point, obstacles):
    """Check if a point is inside any obstacle."""
    for obs in obstacles:
        center = np.array(obs['center'])
        radius = obs['radius']
        if np.linalg.norm(point - center) < radius:
            return True
    return False

def generate_pso_particles(num_particles, start, goal, obstacles):
    """Generate PSO particles avoiding obstacles."""
    particles = []
    np.random.seed(42)  # For consistency
    while len(particles) < num_particles:
        x = np.random.uniform(start[0], goal[0])
        y = np.random.uniform(start[1], goal[1])
        z = np.random.uniform(start[2], goal[2])
        point = np.array([x, y, z])
        
        if not is_inside_obstacle(point, obstacles):
            particles.append(point)
    
    return np.array(particles)

# Define obstacles
obstacles = [
    {'center': [15, 25, 30], 'radius': 14},
    {'center': [10, 60, 50], 'radius': 12},
    {'center': [85, 35, 60], 'radius': 16},
    {'center': [90, 65, 40], 'radius': 10},
    {'center': [50, 50, 20], 'radius': 18},
    {'center': [50, 50, 80], 'radius': 18}
]

# Start and Goal points
start = np.array([5, 5, 5])
goal = np.array([95, 95, 95])

# Generate particles
num_particles = 50  # Reduced for clarity
particles = generate_pso_particles(num_particles, start, goal, obstacles)

# Plot PSO Particles with Obstacles
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
ax.scatter(particles[:, 0], particles[:, 1], particles[:, 2], c='b', alpha=0.5, label='Particles')

ax.set_title("PSO Particle Distribution Around Obstacles")
ax.set_xlabel("X Axis")
ax.set_ylabel("Y Axis")
ax.set_zlabel("Z Axis")
ax.legend()
plt.savefig(f"{output_dir}/pso_particles.png")
plt.close()

print(f"All plots saved in '{output_dir}/'.")