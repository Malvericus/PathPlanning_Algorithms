import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import seaborn as sns
import pandas as pd
from pathlib import Path

class PathfindingAnalyzer:
    def __init__(self, pso_results, rrt_results, astar_results):
        """
        Initialize with algorithm results dictionaries containing:
        - 'path': numpy array of path points
        - 'time': computation time in seconds
        - 'iterations': list of iteration counts
        - 'costs': list of costs per iteration
        - 'expanded_nodes': (for A*) array of explored nodes
        - 'particles': (for PSO) array of particle positions
        """
        self.results = {
            'PSO': pso_results,
            'RRT': rrt_results,
            'A*': astar_results
        }
        self.metrics = self._calculate_metrics()
        
    def _calculate_metrics(self):
        """Compute comparative metrics from raw results"""
        metrics = []
        for algo, data in self.results.items():
            path = data['path']
            metrics.append({
                'Algorithm': algo,
                'Time (s)': data['time'],
                'Path Length': self._path_length(path),
                'Smoothness': self._path_smoothness(path),
                'Success': 1 if path is not None else 0,
                'Memory Usage': len(data.get('expanded_nodes', [])) or len(data.get('particles', []))
            })
        return pd.DataFrame(metrics)
    
    def _path_length(self, path):
        """Calculate total path length"""
        if path is None or len(path) < 2:
            return np.nan
        return sum(np.linalg.norm(path[i+1]-path[i]) for i in range(len(path)-1))
    
    def _path_smoothness(self, path):
        """Calculate sum of angle changes (lower is smoother)"""
        if path is None or len(path) < 3:
            return np.nan
        angles = []
        for i in range(1, len(path)-1):
            v1 = path[i] - path[i-1]
            v2 = path[i+1] - path[i]
            angles.append(np.arccos(np.dot(v1,v2)/(np.linalg.norm(v1)*np.linalg.norm(v2))))
        return sum(angles)
    
    def generate_all_plots(self, output_dir="analysis_results"):
        """Generate and save all comparison plots"""
        Path(output_dir).mkdir(exist_ok=True)
        
        # 1. Performance Metrics Comparison
        plt.figure(figsize=(12,6))
        metrics = self.metrics.set_index('Algorithm')
        metrics[['Time (s)', 'Path Length', 'Smoothness']].plot(kind='bar', subplots=True, layout=(1,3))
        plt.suptitle('Algorithm Comparison Metrics')
        plt.tight_layout()
        plt.savefig(f"{output_dir}/metrics_comparison.png")
        plt.close()
        
        # 2. Convergence Analysis
        plt.figure(figsize=(10,6))
        for algo, data in self.results.items():
            if 'costs' in data and 'iterations' in data:
                plt.plot(data['iterations'], data['costs'], label=algo)
        plt.yscale('log')
        plt.xlabel('Iterations')
        plt.ylabel('Path Cost')
        plt.title('Convergence Comparison')
        plt.legend()
        plt.savefig(f"{output_dir}/convergence.png")
        plt.close()
        
        # 3. Path Quality Visualization
        fig = plt.figure(figsize=(18,6))
        for i, (algo, data) in enumerate(self.results.items()):
            ax = fig.add_subplot(1,3,i+1, projection='3d')
            if data['path'] is not None:
                ax.plot(data['path'][:,0], data['path'][:,1], data['path'][:,2], 'r-', lw=3)
            ax.set_title(f'{algo} Path\nLength: {self.metrics.loc[i,"Path Length"]:.2f}')
        plt.savefig(f"{output_dir}/path_quality.png")
        plt.close()
        
        # 4. Algorithm-Specific Visualizations
        self._generate_algorithm_specific_plots(output_dir)
        
        print(f"Analysis complete! Results saved to {output_dir}/")
    
    def _generate_algorithm_specific_plots(self, output_dir):
        """Generate specialized plots for each algorithm"""
        # PSO Particle Swarm
        if 'particles' in self.results['PSO']:
            plt.figure(figsize=(8,6))
            particles = self.results['PSO']['particles']
            plt.scatter(particles[:,0], particles[:,1], alpha=0.5)
            plt.title('PSO Final Particle Distribution')
            plt.savefig(f"{output_dir}/pso_particles.png")
            plt.close()
        
        # RRT Tree Growth
        if 'nodes' in self.results['RRT']:
            fig = plt.figure(figsize=(8,6))
            ax = fig.add_subplot(111, projection='3d')
            nodes = np.array(self.results['RRT']['nodes'])
            parents = self.results['RRT']['parents']
            for i in range(1, len(nodes)):
                ax.plot([nodes[i,0], nodes[parents[i],0]],
                        [nodes[i,1], nodes[parents[i],1]],
                        [nodes[i,2], nodes[parents[i],2]], 'b-', alpha=0.3)
            plt.title('RRT Exploration Tree')
            plt.savefig(f"{output_dir}/rrt_tree.png")
            plt.close()
        
        # A* Expanded Nodes
        if 'expanded_nodes' in self.results['A*']:
            fig = plt.figure(figsize=(8,6))
            ax = fig.add_subplot(111, projection='3d')
            nodes = self.results['A*']['expanded_nodes']
            ax.scatter(nodes[:,0], nodes[:,1], nodes[:,2], s=1, alpha=0.3)
            plt.title('A* Explored Nodes')
            plt.savefig(f"{output_dir}/astar_exploration.png")
            plt.close()

# Example Usage
if __name__ == "__main__":
    # Mock data - replace with your actual algorithm results
    pso_data = {
        'path': np.random.rand(50,3)*100,
        'time': 45.2,
        'iterations': list(range(100)),
        'costs': np.logspace(3,1,100),
        'particles': np.random.rand(100,2)*100
    }
    
    rrt_data = {
        'path': np.random.rand(70,3)*100,
        'time': 12.7,
        'iterations': list(range(500)),
        'costs': np.linspace(500,120,500),
        'nodes': np.random.rand(500,3)*100,
        'parents': list(range(500))
    }
    
    astar_data = {
        'path': np.random.rand(40,3)*100,
        'time': 38.5,
        'iterations': list(range(3000)),
        'costs': np.linspace(3000,150,3000),
        'expanded_nodes': np.random.rand(3000,3)*100
    }
    
    analyzer = PathfindingAnalyzer(pso_data, rrt_data, astar_data)
    analyzer.generate_all_plots()