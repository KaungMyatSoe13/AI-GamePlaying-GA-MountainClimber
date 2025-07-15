# === Enhanced Mountain Climbing Genetic Algorithm ===

import pybullet as p
import pybullet_data
import time
import numpy as np
import random
import math
import matplotlib.pyplot as plt
from datetime import datetime
import csv
import os
import xml.etree.ElementTree as ET
from xml.dom import minidom

from creature import Creature
import population
import simulation

from prepare_shapes import (
    make_pyramid,
    make_rocky_moutain,
    generate_gaussian_pyramid, 
    generate_gaussian_pyramid2, 
    generate_gaussian_pyramid3, 
    generate_gaussian_pyramid4
)

# =====================================================
# [MY CODE] Utility: Clamp joint parent value in DNA
# =====================================================
def clamp_joint_parent(dna, spec):
    """Clamp the joint-parent index in the DNA to a maximum allowed value."""
    idx = spec["joint-parent"]["ind"]
    if isinstance(dna, np.ndarray):
        for gene in dna:
            gene[idx] = min(gene[idx], 0.9999)
    else:
        for gene in dna:
            gene[idx] = min(gene[idx], 0.9999)

# =====================================================
# [MY CODE] Utility: Show mountain in PyBullet GUI
# =====================================================
def camera_test(mountain_type):
    """Display the selected mountain type in the PyBullet GUI for visualization."""
    import pybullet as p
    import pybullet_data
    import time
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    p.setAdditionalSearchPath('shapes/')
    p.resetDebugVisualizerCamera(cameraDistance=30, cameraYaw=45, cameraPitch=-25, cameraTargetPosition=[0, 0, 3])
    mountain_position = (0, 0, 0)
    mountain_orientation = p.getQuaternionFromEuler((0, 0, 0))
    urdf_map = {
        'default': 'gaussian_pyramid.urdf',
        'steep': 'mountain_steep.urdf',
        'ridge': 'mountain_ridge.urdf',
        'plateau': 'mountain_plateau.urdf',
    }
    urdf_file = urdf_map.get(mountain_type, 'gaussian_pyramid.urdf')
    try:
        p.loadURDF(urdf_file, mountain_position, mountain_orientation, useFixedBase=1)
    except Exception as e:
        print(f"Could not load URDF: {urdf_file}, error: {e}")
    print("Camera test running. Close the window to continue...")
    while p.isConnected():
        p.stepSimulation()
        time.sleep(1/240)

class MountainClimbingSimulation:
    def __init__(self, arena_size=20, mountain_height=5, simulation_time=15, mountain_type='default'):
        """Initialize the simulation environment and parameters."""
        self.arena_size = arena_size
        self.mountain_height = mountain_height
        self.simulation_time = simulation_time
        self.mountain_center = (0, 0)
        self.mountain_radius = arena_size / 4
        self.mountain_type = mountain_type
        
        # Different starting positions to test climbing ability
        self.starting_positions = [
            (0, 0, 7),  # Only drop from above
        ]

    def setup_environment(self, gui=False):
        """Set up the PyBullet simulation environment and load the mountain."""
        if p.isConnected():
            p.disconnect()
        if gui:
            p.connect(p.GUI)
            p.resetDebugVisualizerCamera(
                cameraDistance=30,
                cameraYaw=45,
                cameraPitch=-55,
                cameraTargetPosition=[0, 0, 3]
            )
        else:
            p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -10)

        wall_height = 8 if gui else 12
        self.make_arena(wall_height=wall_height, gui=gui)
        
        # Add ceiling to prevent flying
        if not gui:
            self.make_ceiling(height=10, gui=gui)

        # Create shapes directory if it doesn't exist
        os.makedirs("./shapes", exist_ok=True)

        # Mountain generation mapping
        mountain_generators = {
            'default': (generate_gaussian_pyramid4, 'gaussian_complete.obj', 'gaussian_pyramid.urdf'),
            'steep': (generate_gaussian_pyramid2, 'mountain_steep.obj', 'mountain_steep.urdf'),
            'ridge': (generate_gaussian_pyramid3, 'mountain_ridge.obj', 'mountain_ridge.urdf'),
            'plateau': (generate_gaussian_pyramid, 'mountain_plateau.obj', 'mountain_plateau.urdf'),
        }

        # Generate mountain if needed
        if self.mountain_type in mountain_generators:
            generator_func, obj_filename, urdf_filename = mountain_generators[self.mountain_type]
            obj_path = f'./shapes/{obj_filename}'
            urdf_path = f'./shapes/{urdf_filename}'
            
            # Generate OBJ file if it doesn't exist
            if not os.path.exists(obj_path):
                print(f"Generating {self.mountain_type} mountain...")
                generator_func(obj_path)
            
            # Create URDF file if it doesn't exist
            if not os.path.exists(urdf_path):
                print(f"Creating URDF for {self.mountain_type} mountain...")
                self.create_urdf_from_obj(obj_path, urdf_path)
            
            # Load the mountain
            mountain_position = (0, 0, 0)
            mountain_orientation = p.getQuaternionFromEuler((0, 0, 0))
            p.setAdditionalSearchPath('shapes/')
            
            try:
                self.mountain = p.loadURDF(urdf_filename, mountain_position, mountain_orientation, useFixedBase=1)
                print(f"Loaded {self.mountain_type} mountain successfully")
            except Exception as e:
                print(f"Warning: Could not load mountain URDF {urdf_filename}. Creating procedural mountain. Error: {e}")
                self.make_mountain()
        else:
            print(f"Unknown mountain type: {self.mountain_type}. Creating procedural mountain.")
            self.make_mountain()

    def make_arena(self, wall_height=12, gui=False):
        """Create the simulation arena with walls and floor."""
        wall_thickness = 0.5
        # Floor
        floor_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[self.arena_size/2, self.arena_size/2, wall_thickness])
        floor_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[self.arena_size/2, self.arena_size/2, wall_thickness], rgbaColor=[0.8, 0.6, 0.4, 1])
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=floor_collision_shape, baseVisualShapeIndex=floor_visual_shape, basePosition=[0, 0, -wall_thickness])

        # Walls
        wall_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[self.arena_size/2, wall_thickness/2, wall_height/2])
        wall_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[self.arena_size/2, wall_thickness/2, wall_height/2], rgbaColor=[0.7, 0.7, 0.7, 1])

        positions = [
            [0, self.arena_size/2, wall_height/2],
            [0, -self.arena_size/2, wall_height/2],
            [self.arena_size/2, 0, wall_height/2],
            [-self.arena_size/2, 0, wall_height/2]
        ]

        for i, pos in enumerate(positions):
            if i < 2:
                p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=pos)
            else:
                wall_collision_shape_ew = p.createCollisionShape(p.GEOM_BOX, halfExtents=[wall_thickness/2, self.arena_size/2, wall_height/2])
                wall_visual_shape_ew = p.createVisualShape(p.GEOM_BOX, halfExtents=[wall_thickness/2, self.arena_size/2, wall_height/2], rgbaColor=[0.7, 0.7, 0.7, 1])
                p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape_ew, baseVisualShapeIndex=wall_visual_shape_ew, basePosition=pos)

    def make_ceiling(self, height=10, gui=False):
        """Add a ceiling to the simulation arena to prevent flying."""
        ceiling_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[self.arena_size/2, self.arena_size/2, 0.1])
        if gui:
            ceiling_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[self.arena_size/2, self.arena_size/2, 0.1], rgbaColor=[0.9, 0.9, 0.9, 0.1])
        else:
            ceiling_visual_shape = -1
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=ceiling_collision_shape, baseVisualShapeIndex=ceiling_visual_shape, basePosition=[0, 0, height])

    def make_mountain(self, num_rocks=250):
        """Create a procedural mountain with varied terrain."""
        def gaussian_height(x, y):
            distance_sq = x**2 + y**2
            return self.mountain_height * math.exp(-distance_sq / (2 * self.mountain_radius**2))

        # Create base mountain structure
        for _ in range(num_rocks):
            # Create varied distribution - more rocks near center, fewer at edges
            if random.random() < 0.7:  # 70% near center
                x = random.uniform(-self.arena_size/4, self.arena_size/4)
                y = random.uniform(-self.arena_size/4, self.arena_size/4)
            else:  # 30% scattered around
                x = random.uniform(-self.arena_size/3, self.arena_size/3)
                y = random.uniform(-self.arena_size/3, self.arena_size/3)
            
            z = gaussian_height(x, y)
            
            # Add some randomness to height
            z += random.uniform(-0.2, 0.2)
            z = max(0, z)  # Don't go below ground
            
            # Vary rock sizes based on height and position
            size_factor = 1 - (z / self.mountain_height) * 0.5
            size = random.uniform(0.08, 0.35) * size_factor
            
            orientation = p.getQuaternionFromEuler([random.uniform(0, math.pi) for _ in range(3)])
            
            # Create rock with varied shape
            if random.random() < 0.8:  # 80% boxes
                rock_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size, size*random.uniform(0.7, 1.3), size*random.uniform(0.7, 1.3)])
                rock_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[size, size*random.uniform(0.7, 1.3), size*random.uniform(0.7, 1.3)], 
                                                rgbaColor=[0.4 + z/self.mountain_height*0.3, 0.3, 0.2, 1])
            else:  # 20% spheres for variety
                rock_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=size*0.8)
                rock_visual = p.createVisualShape(p.GEOM_SPHERE, radius=size*0.8, 
                                                rgbaColor=[0.5 + z/self.mountain_height*0.2, 0.35, 0.25, 1])
            
            p.createMultiBody(baseMass=0, baseCollisionShapeIndex=rock_shape, baseVisualShapeIndex=rock_visual, 
                            basePosition=[x, y, z], baseOrientation=orientation)

    def create_urdf_from_obj(self, obj_path, urdf_path):
        """Create a URDF file from an OBJ mesh file for the mountain."""
        obj_filename = os.path.basename(obj_path)
        
        urdf_content = f'''<?xml version="1.0"?>
<robot name="mountain">
  <link name="baseLink">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="{obj_filename}" scale="1 1 1"/>
      </geometry>
      <material name="gray">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="{obj_filename}" scale="1 1 1"/>
      </geometry>
    </collision>
  </link>
</robot>'''
        
        with open(urdf_path, 'w') as f:
            f.write(urdf_content)
        print(f"Created URDF file: {urdf_path}")

    def get_mountain_height_at_position(self, x, y):
        """Get the theoretical mountain height at a specific (x, y) position."""
        distance_sq = x**2 + y**2
        return self.mountain_height * math.exp(-distance_sq / (2 * self.mountain_radius**2))

    def evaluate_creature(self, creature_xml, population_id=0, creature_id=0, gui=False):
        """Evaluate a creature's climbing ability and return fitness and other metrics."""
        
        temp_filename = f'temp_creature_{population_id}_{creature_id}.urdf'
        with open(temp_filename, 'w') as f:
            f.write(creature_xml)
        
        try:
            # Test creature from multiple starting positions
            all_results = []
            starting_positions = random.sample(self.starting_positions, min(3, len(self.starting_positions)))
            
            for pos_idx, start_pos in enumerate(starting_positions):
                robot_id = p.loadURDF(temp_filename, start_pos)
                
                # Track performance metrics
                max_height = start_pos[2]  # Start with initial height
                max_mountain_progress = 0
                positions = []
                heights = []
                distances_to_center = []
                contact_times = 0
                climbing_progress = []
                
                # Add realistic physics
                p.changeDynamics(robot_id, -1, linearDamping=0.05, angularDamping=0.05, lateralFriction=0.8)
                
                # Simulation loop
                steps = int(self.simulation_time * 240)
                for step in range(steps):
                    p.stepSimulation()
                    
                    pos, _ = p.getBasePositionAndOrientation(robot_id)
                    x, y, z = pos
                    
                    # Early termination if creature falls off significantly
                    if z < -2 or abs(x) > self.arena_size/2 or abs(y) > self.arena_size/2:
                        break
                    
                    # Calculate metrics
                    current_height = z
                    distance_to_center = math.sqrt(x**2 + y**2)
                    theoretical_mountain_height = self.get_mountain_height_at_position(x, y)
                    mountain_progress = max(0, current_height - theoretical_mountain_height)
                    
                    # Check if creature is in contact with mountain surface
                    if abs(current_height - theoretical_mountain_height) < 0.3:
                        contact_times += 1
                    
                    # Update metrics
                    max_height = max(max_height, current_height)
                    max_mountain_progress = max(max_mountain_progress, mountain_progress)
                    
                    positions.append((x, y, z))
                    heights.append(current_height)
                    distances_to_center.append(distance_to_center)
                    climbing_progress.append(mountain_progress)
                    
                    if not gui:
                        time.sleep(1/4800)  # Fast simulation
                    else:
                        time.sleep(1/240)
                
                # Calculate fitness for this starting position
                fitness = self.calculate_enhanced_fitness(
                    max_height, max_mountain_progress, positions, heights, 
                    distances_to_center, contact_times, climbing_progress, start_pos
                )
                
                all_results.append({
                    'fitness': fitness,
                    'max_height': max_height,
                    'max_mountain_progress': max_mountain_progress,
                    'contact_ratio': contact_times / len(positions) if positions else 0,
                    'start_pos': start_pos
                })
                
                p.removeBody(robot_id)
            
            # Use best result from all starting positions
            best_result = max(all_results, key=lambda x: x['fitness'])
            
            return {
                'fitness': best_result['fitness'],
                'max_height': best_result['max_height'],
                'max_mountain_progress': best_result['max_mountain_progress'],
                'contact_ratio': best_result['contact_ratio'],
                'num_tests': len(all_results),
                'best_start_pos': best_result['start_pos']
            }
            
        except Exception as e:
            print(f"Error evaluating creature: {e}")
            return {
                'fitness': 0,
                'max_height': 0,
                'max_mountain_progress': 0,
                'contact_ratio': 0,
                'num_tests': 0,
                'best_start_pos': (0, 0, 0)
            }
        finally:
            if os.path.exists(temp_filename):
                os.remove(temp_filename)

    def calculate_enhanced_fitness(self, max_height, max_mountain_progress, positions, 
                                 heights, distances, contact_times, climbing_progress, start_pos):
        """Compute the fitness score for a creature based on climbing performance and stability."""
        
        if not positions:
            return 0
        
        # Calculate contact ratio
        contact_ratio = contact_times / len(positions)
        
        # === STABILITY APPROACH ===
        # Creatures MUST maintain reasonable contact to get any significant fitness
        
        # 1. Contact requirement (CRITICAL)
        if contact_ratio < 0.15:  # Less than 15% contact
            # Severe penalty for poor contact - almost no fitness
            contact_multiplier = contact_ratio / 0.15  # 0.0 to 1.0 scaling
            contact_multiplier = contact_multiplier ** 2  # Square to make penalty more severe
        else:
            # Bonus for good contact
            contact_multiplier = 1.0 + (contact_ratio - 0.15) * 2  # Up to 2.7x multiplier for perfect contact
        
        # 2. Height gained from starting position (moderate reward)
        start_height = start_pos[2]
        height_gain = max(0, max_height - start_height)
        height_fitness = height_gain * 8  # Reduced from 15
        
        # 3. Mountain climbing progress (good reward, but contact-dependent)
        mountain_bonus = max_mountain_progress * 12  # Reduced from 25
        
        # 4. Sustained climbing - reward consistent progress
        avg_climbing_progress = np.mean([max(0, cp) for cp in climbing_progress])
        sustained_climbing = avg_climbing_progress * 10  # Reduced from 20
        
        # 5. Movement efficiency - penalize excessive speed/erratic movement
        if len(positions) > 1:
            total_movement = 0
            vertical_movement = 0
            for i in range(1, len(positions)):
                movement = math.sqrt(sum((a-b)**2 for a, b in zip(positions[i], positions[i-1])))
                vertical_movement += abs(positions[i][2] - positions[i-1][2])
                total_movement += movement
            
            avg_speed = total_movement / len(positions)
            avg_vertical_speed = vertical_movement / len(positions)
            
            # Penalize if moving too fast (likely jumping/launching)
            if avg_speed > 0.8:  # More strict speed limit
                speed_penalty = (avg_speed - 0.8) * 20
            else:
                speed_penalty = 0
                
            # Reward efficient vertical movement vs horizontal movement
            if total_movement > 0:
                vertical_efficiency = vertical_movement / total_movement
                efficiency_bonus = vertical_efficiency * 15
            else:
                efficiency_bonus = 0
        else:
            speed_penalty = 0
            efficiency_bonus = 0
        
        # 6. Stability bonus - reward creatures that don't fall off
        if positions and len(positions) > self.simulation_time * 240 * 0.8:
            stability_bonus = 20  # Increased stability reward
        else:
            stability_bonus = 0
        
        # 7. Contact consistency bonus
        # Check for consistent contact throughout simulation
        if len(positions) >= 100:  # Enough data points
            contact_windows = []
            window_size = 50
            for i in range(0, len(positions) - window_size, window_size // 2):
                window_contacts = sum(1 for j in range(i, i + window_size) 
                                    if j < len(climbing_progress) and 
                                    abs(heights[j] - self.get_mountain_height_at_position(positions[j][0], positions[j][1])) < 0.3)
                window_contact_ratio = window_contacts / window_size
                contact_windows.append(window_contact_ratio)
            
            if contact_windows:
                consistency_bonus = (1.0 - np.std(contact_windows)) * 30  # Reward consistent contact
                min_contact_window = min(contact_windows)
                if min_contact_window < 0.1:  # Penalize if any window has very poor contact
                    consistency_bonus -= 50
            else:
                consistency_bonus = 0
        else:
            consistency_bonus = 0
        
        # 8. Anti-launch penalty
        # Detect if creature is spending significant time high above the mountain
        if positions:
            high_altitude_time = 0
            for i, pos in enumerate(positions):
                theoretical_height = self.get_mountain_height_at_position(pos[0], pos[1])
                if pos[2] > theoretical_height + 2.0:  # More than 2 units above mountain
                    high_altitude_time += 1
            
            high_altitude_ratio = high_altitude_time / len(positions)
            if high_altitude_ratio > 0.3:  # More than 30% time in air
                launch_penalty = (high_altitude_ratio - 0.3) * 100
            else:
                launch_penalty = 0
        else:
            launch_penalty = 0
        
        # 9. Progressive climbing bonus
        # Reward creatures that show steady upward progress
        if len(heights) > 10:
            height_progression = []
            window_size = max(10, len(heights) // 10)
            for i in range(0, len(heights) - window_size, window_size):
                window_avg = np.mean(heights[i:i+window_size])
                height_progression.append(window_avg)
            
            if len(height_progression) > 1:
                progressive_gains = sum(max(0, height_progression[i] - height_progression[i-1]) 
                                      for i in range(1, len(height_progression)))
                progressive_bonus = progressive_gains * 5
            else:
                progressive_bonus = 0
        else:
            progressive_bonus = 0
        
        # Calculate base fitness
        base_fitness = (height_fitness + mountain_bonus + sustained_climbing + 
                       stability_bonus + consistency_bonus + efficiency_bonus + 
                       progressive_bonus - speed_penalty - launch_penalty)
        
        # Apply contact multiplier (this is key!)
        final_fitness = base_fitness * contact_multiplier
        
        # Additional severe penalty for creatures with almost no contact
        if contact_ratio < 0.05:  # Less than 5% contact
            final_fitness *= 0.1  # Reduce to 10% of calculated fitness
        
        return max(0, final_fitness)

class MountainClimbingGA:
    """Enhanced Genetic Algorithm for mountain climbing creatures"""
    
    def __init__(self, population_size=24, gene_count=6, mutation_rate=0.2, 
                 crossover_rate=0.75, elite_count=3, simulation_time=15, mountain_type='default'):
        """Initialize the genetic algorithm parameters and simulation."""
        self.population_size = population_size
        self.gene_count = gene_count
        self.mutation_rate = mutation_rate
        self.crossover_rate = crossover_rate
        self.elite_count = elite_count
        self.simulation_time = simulation_time
        
        self.simulation = MountainClimbingSimulation(simulation_time=simulation_time, mountain_type=mountain_type)
        self.generation = 0
        self.fitness_history = []
        self.best_creature_history = []
        
    def run_evolution(self, generations=40, save_results=True, show_gui_every=5):
        """Run the genetic algorithm evolution process for a set number of generations."""
        
        print("Setting up enhanced mountain climbing environment...")
        self.simulation.setup_environment(gui=False)
        
        # Initialize population with more diversity
        print(f"Creating diverse initial population of {self.population_size} creatures...")
        pop = population.Population(self.population_size, self.gene_count)
        
        # Add diversity to initial population
        for creature in pop.creatures:
            # Add random mutations to increase initial diversity
            current_dna = creature.get_dna()
            if isinstance(current_dna, np.ndarray):
                mutated_dna = current_dna + np.random.normal(0, 0.3, current_dna.shape)
                mutated_dna = np.clip(mutated_dna, -1, 1)
            else:
                mutated_dna = [g + np.random.normal(0, 0.3, g.shape) for g in current_dna]
                mutated_dna = [np.clip(g, -1, 1) for g in mutated_dna]
            clamp_joint_parent(mutated_dna, creature.spec)
            creature.set_dna(mutated_dna)
        
        # Evolution loop
        for gen in range(generations):
            self.generation = gen
            show_gui = (gen % show_gui_every == 0)
            
            print(f"\n=== Generation {gen + 1}/{generations} ===")
            
            # Evaluate population
            fitnesses = []
            detailed_results = []
            
            for i, creature_genome in enumerate(pop.creatures):
                if show_gui and i == 0 and gen > 0:  # Show best creature
                    self.simulation.setup_environment(gui=True)
                
                # Create creature from genome
                cr = Creature(gene_count=self.gene_count)
                dna = creature_genome.get_dna()
                clamp_joint_parent(dna, cr.spec)
                cr.set_dna(dna)
                creature_xml = cr.to_xml()
                
                # Evaluate creature
                result = self.simulation.evaluate_creature(
                    creature_xml, gen, i, gui=(show_gui and i == 0 and gen > 0)
                )
                
                fitnesses.append(result['fitness'])
                detailed_results.append(result)
                
                if show_gui and i == 0 and gen > 0:
                    time.sleep(2)  # Show result for a moment
                    p.disconnect()
                    self.simulation.setup_environment(gui=False)
                
                print(f"Creature {i+1:2d}: Fitness={result['fitness']:6.2f}, "
                      f"Height={result['max_height']:.2f}, "
                      f"MtnProg={result['max_mountain_progress']:.2f}, "
                      f"Contact={result['contact_ratio']:.4f}")
            
            # Update fitness in population
            for i, fitness in enumerate(fitnesses):
                pop.creatures[i].fitness = fitness
            
            # Track statistics
            best_fitness = max(fitnesses)
            avg_fitness = np.mean(fitnesses)
            worst_fitness = min(fitnesses)
            
            self.fitness_history.append({
                'generation': gen,
                'best_fitness': best_fitness,
                'avg_fitness': avg_fitness,
                'worst_fitness': worst_fitness,
                'std_fitness': np.std(fitnesses)
            })
            
            # Save best creature
            best_idx = fitnesses.index(best_fitness)
            best_creature = pop.creatures[best_idx]
            dna = best_creature.get_dna()
            clamp_joint_parent(dna, best_creature.spec)
            best_creature.set_dna(dna)
            
            self.best_creature_history.append({
                'generation': gen,
                'dna': dna.copy(),
                'fitness': best_fitness,
                'details': detailed_results[best_idx]
            })
            
            print(f"Gen {gen+1} Summary: Best={best_fitness:.2f}, Avg={avg_fitness:.2f}, "
                  f"Worst={worst_fitness:.2f}, Std={np.std(fitnesses):.2f}")
            
            # Adaptive mutation rate
            if gen > 5:
                recent_best = [h['best_fitness'] for h in self.fitness_history[-5:]]
                if max(recent_best) - min(recent_best) < 5:  # Stagnation
                    self.mutation_rate = min(0.4, self.mutation_rate * 1.1)
                    print(f"  Increasing mutation rate to {self.mutation_rate:.3f}")
                else:
                    self.mutation_rate = max(0.1, self.mutation_rate * 0.95)
            
            # Create next generation
            if gen < generations - 1:
                pop = self.create_next_generation(pop)
        
        # Cleanup
        p.disconnect()
        
        # Save results
        if save_results:
            self.save_results()
        
        return self.fitness_history, self.best_creature_history
    
    def create_next_generation(self, current_pop):
        """Create the next generation of the population using selection, crossover, and mutation."""
        new_pop = population.Population(self.population_size, self.gene_count)
        
        # Sort by fitness
        sorted_creatures = sorted(current_pop.creatures, key=lambda x: x.fitness, reverse=True)
        
        # Elitism - keep best creatures
        for i in range(self.elite_count):
            new_pop.creatures[i] = sorted_creatures[i]
        
        # Generate rest through crossover and mutation
        for i in range(self.elite_count, self.population_size):
            # Fitness proportionate selection with tournament
            if random.random() < 0.7:  # 70% tournament selection
                parent1 = self.tournament_selection(sorted_creatures, tournament_size=4)
                parent2 = self.tournament_selection(sorted_creatures, tournament_size=4)
            else:  # 30% fitness proportionate
                parent1 = self.fitness_proportionate_selection(sorted_creatures)
                parent2 = self.fitness_proportionate_selection(sorted_creatures)
            
            # Crossover
            if random.random() < self.crossover_rate:
                child_dna = self.enhanced_crossover(parent1.get_dna(), parent2.get_dna())
            else:
                child_dna = parent1.get_dna().copy() if isinstance(parent1.get_dna(), np.ndarray) else [g.copy() for g in parent1.get_dna()]
            
            # Mutation
            if random.random() < self.mutation_rate:
                child_dna = self.enhanced_mutation(child_dna)
            
            # Create new creature
            clamp_joint_parent(child_dna, new_pop.creatures[i].spec)
            new_pop.creatures[i].set_dna(child_dna)
        
        return new_pop
    
    def tournament_selection(self, creatures, tournament_size=4):
        """Select a parent using tournament selection."""
        tournament = random.sample(creatures, min(tournament_size, len(creatures)))
        return max(tournament, key=lambda x: x.fitness)
    
    def fitness_proportionate_selection(self, creatures):
        """Select a parent using fitness proportionate (roulette wheel) selection."""
        total_fitness = sum(c.fitness for c in creatures)
        if total_fitness <= 0:
            return random.choice(creatures)
        
        pick = random.uniform(0, total_fitness)
        current = 0
        for creature in creatures:
            current += creature.fitness
            if current >= pick:
                return creature
        return creatures[-1]
    
    def enhanced_crossover(self, dna1, dna2):
        """Perform crossover between two DNA sequences using various strategies."""
        crossover_type = random.choice(['single_point', 'two_point', 'uniform'])
        
        if isinstance(dna1, np.ndarray):
            if crossover_type == 'single_point':
                point = np.random.randint(1, len(dna1))
                return np.concatenate((dna1[:point], dna2[point:]))
            elif crossover_type == 'two_point':
                point1, point2 = sorted(np.random.choice(len(dna1), 2, replace=False))
                return np.concatenate((dna1[:point1], dna2[point1:point2], dna1[point2:]))
            else:  # uniform
                mask = np.random.random(len(dna1)) < 0.5
                return np.where(mask, dna1, dna2)
        else:
            if crossover_type == 'single_point':
                point = random.randint(1, len(dna1) - 1)
                return dna1[:point] + dna2[point:]
            elif crossover_type == 'two_point':
                point1, point2 = sorted(random.sample(range(len(dna1)), 2))
                return dna1[:point1] + dna2[point1:point2] + dna1[point2:]
            else:  # uniform
                return [d1 if random.random() < 0.5 else d2 for d1, d2 in zip(dna1, dna2)]
    
    def enhanced_mutation(self, dna):
        """Apply mutation to a DNA sequence with adaptive strength."""
        mutation_strength = 0.1 + random.random() * 0.2  # Variable strength
        
        if isinstance(dna, np.ndarray):
            mutated_dna = dna.copy()
            for i in range(len(mutated_dna)):
                if random.random() < 0.15:  # 15% chance per gene
                    if random.random() < 0.8:  # 80% Gaussian mutation
                        mutated_dna[i] += random.gauss(0, mutation_strength)
                    else:  # 20% random reset
                        mutated_dna[i] = random.uniform(-1, 1)
                    mutated_dna[i] = np.clip(mutated_dna[i], -1, 1)
            return mutated_dna
        else:
            mutated_dna = [g.copy() for g in dna]
            for i in range(len(mutated_dna)):
                if random.random() < 0.15:
                    if random.random() < 0.8:
                        mutated_dna[i] += random.gauss(0, mutation_strength)
                    else:
                        mutated_dna[i] = np.random.uniform(-1, 1, mutated_dna[i].shape)
                    mutated_dna[i] = np.clip(mutated_dna[i], -1, 1)
            return mutated_dna
    
    def save_results(self):
        """Save the results of the evolution, including fitness history and best creature."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Save fitness history
        with open(f'mountain_climbing_fitness_{timestamp}.csv', 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['generation', 'best_fitness', 'avg_fitness', 'worst_fitness', 'std_fitness'])
            writer.writeheader()
            writer.writerows(self.fitness_history)
        
        # Save best creature from final generation
        if self.best_creature_history:
            best_final = self.best_creature_history[-1]
            cr = Creature(gene_count=self.gene_count)
            dna = best_final['dna']
            clamp_joint_parent(dna, cr.spec)
            cr.set_dna(dna)
            
            with open(f'best_mountain_climber_{timestamp}.urdf', 'w') as f:
                f.write(cr.to_xml())
        
        # Create comprehensive plots
        self.plot_fitness_evolution(timestamp)
        
        print(f"Results saved with timestamp: {timestamp}")
    
    def plot_fitness_evolution(self, timestamp):
        """Plot the fitness evolution over generations and save the figure."""
        if not self.fitness_history:
            print("No fitness history to plot!")
            return
            
        plt.figure(figsize=(12, 8))
        
        # Plot average fitness
        generations = [h['generation'] for h in self.fitness_history]
        avg_fitness = [h['avg_fitness'] for h in self.fitness_history]
        plt.plot(generations, avg_fitness, label='Average Fitness', 
                   alpha=0.7, color='blue')
        plt.xlabel('Generation (Every 5th)')
        plt.ylabel('Best Fitness')
        plt.title('Fitness Milestones')
        plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(f'mountain_climbing_detailed_{timestamp}.png', dpi=300, bbox_inches='tight')
        plt.show()

    def demonstrate_best_creature(self, show_multiple=True):
        """Demonstrate the best evolved creature in the simulation GUI."""
        if not self.best_creature_history:
            print("No evolved creatures to demonstrate!")
            return
        
        best = self.best_creature_history[-1]
        print(f"\nDemonstrating best creature from generation {best['generation']}")
        print(f"Fitness: {best['fitness']:.2f}")
        print(f"Max height: {best['details']['max_height']:.2f}")
        print(f"Mountain progress: {best['details']['max_mountain_progress']:.2f}")
        print(f"Contact ratio: {best['details']['contact_ratio']:.2f}")
        
        # Setup GUI environment
        self.simulation.setup_environment(gui=True)
        
        # Create creature
        cr = Creature(gene_count=self.gene_count)
        dna = best['dna']
        clamp_joint_parent(dna, cr.spec)
        cr.set_dna(dna)
        creature_xml = cr.to_xml()
        
        if show_multiple:
            # Test from multiple positions
            positions_to_test = [
                (0, -6, 1, "South"),
                (6, 0, 1, "East"),
                (0, 0, 1, "Center"),
                (-4, -4, 1, "Southwest")
            ]
        else:
            positions_to_test = [(0, -6, 1, "South")]
        
        for pos_x, pos_y, pos_z, pos_name in positions_to_test:
            print(f"\nTesting from {pos_name} position...")
            
            # Load creature
            temp_filename = 'demo_creature.urdf'
            with open(temp_filename, 'w') as f:
                f.write(creature_xml)
            
            robot_id = p.loadURDF(temp_filename, (pos_x, pos_y, pos_z))
            p.changeDynamics(robot_id, -1, linearDamping=0.05, angularDamping=0.05, lateralFriction=0.8)
            
            # Run simulation
            max_height = pos_z
            for step in range(int(self.simulation_time * 240)):
                p.stepSimulation()
                pos, _ = p.getBasePositionAndOrientation(robot_id)
                max_height = max(max_height, pos[2])
                time.sleep(1/240)
            
            print(f"Final position: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
            print(f"Max height reached: {max_height:.2f}")
            
            p.removeBody(robot_id)
            if os.path.exists(temp_filename):
                os.remove(temp_filename)
            
            if show_multiple:
                input("Press Enter to test next position...")
        
        p.disconnect()

# === Systematic Experiments ===
class CourseWorkExperiments:
    """Systematic experiments for coursework requirements"""
    
    def __init__(self):
        """Initialize the coursework experiments and results storage."""
        self.results = []
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    def run_population_size_experiment(self, sizes=[10, 12, 24], generations=20):
        """Run experiments to test different population sizes."""
        print("=== POPULATION SIZE EXPERIMENT ===")
        
        # Define optimized parameters for each population size
        population_configs = {
            10: {'mutation_rate': 0.3, 'gene_count': 4},  # Higher mutation, fewer genes for small population
            12: {'mutation_rate': 0.25, 'gene_count': 5},  # Medium mutation, medium genes
            24: {'mutation_rate': 0.2, 'gene_count': 6}    # Lower mutation, more genes for large population
        }
        
        for pop_size in sizes:
            config = population_configs[pop_size]
            print(f"\nTesting population size: {pop_size}")
            print(f"  Mutation rate: {config['mutation_rate']}")
            print(f"  Gene count: {config['gene_count']}")
            
            ga = MountainClimbingGA(
                population_size=pop_size, 
                gene_count=config['gene_count'], 
                mutation_rate=config['mutation_rate'],
                simulation_time=10
            )
            fitness_history, _ = ga.run_evolution(generations=generations, save_results=False, show_gui_every=999)
            
            final_fitness = fitness_history[-1]['best_fitness'] if fitness_history else 0
            avg_fitness = np.mean([h['best_fitness'] for h in fitness_history]) if fitness_history else 0
            
            self.results.append({
                'experiment': 'population_size',
                'parameter': pop_size,
                'mutation_rate': config['mutation_rate'],
                'gene_count': config['gene_count'],
                'final_fitness': final_fitness,
                'avg_fitness': avg_fitness,
                'generations': generations
            })
    
    def run_mutation_rate_experiment(self, rates=[0.1, 0.2, 0.3, 0.4], generations=20):
        """Run experiments to test different mutation rates."""
        print("=== MUTATION RATE EXPERIMENT ===")
        for rate in rates:
            print(f"\nTesting mutation rate: {rate}")
            ga = MountainClimbingGA(population_size=24, gene_count=6, mutation_rate=rate, simulation_time=10)
            fitness_history, _ = ga.run_evolution(generations=generations, save_results=False, show_gui_every=999)
            
            final_fitness = fitness_history[-1]['best_fitness'] if fitness_history else 0
            avg_fitness = np.mean([h['best_fitness'] for h in fitness_history]) if fitness_history else 0
            
            self.results.append({
                'experiment': 'mutation_rate',
                'parameter': rate,
                'final_fitness': final_fitness,
                'avg_fitness': avg_fitness,
                'generations': generations
            })
    
    def run_gene_count_experiment(self, gene_counts=[4, 6, 8], generations=20):
        """Run experiments to test different gene counts."""
        print("=== GENE COUNT EXPERIMENT ===")
        for gene_count in gene_counts:
            print(f"\nTesting gene count: {gene_count}")
            ga = MountainClimbingGA(population_size=24, gene_count=gene_count, simulation_time=10)
            fitness_history, _ = ga.run_evolution(generations=generations, save_results=False, show_gui_every=999)
            
            final_fitness = fitness_history[-1]['best_fitness'] if fitness_history else 0
            avg_fitness = np.mean([h['best_fitness'] for h in fitness_history]) if fitness_history else 0
            
            self.results.append({
                'experiment': 'gene_count',
                'parameter': gene_count,
                'final_fitness': final_fitness,
                'avg_fitness': avg_fitness,
                'generations': generations
            })
    
    def save_and_plot_results(self):
        """Save experiment results to CSV and plot comparison graphs."""
        # Save to CSV
        csv_filename = f'coursework_experiments_{self.timestamp}.csv'
        with open(csv_filename, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['experiment', 'parameter', 'mutation_rate', 'gene_count', 'final_fitness', 'avg_fitness', 'generations'])
            writer.writeheader()
            writer.writerows(self.results)
        
        # Create comparison plots
        experiments = {}
        for result in self.results:
            exp_name = result['experiment']
            if exp_name not in experiments:
                experiments[exp_name] = {'params': [], 'final': [], 'avg': []}
            experiments[exp_name]['params'].append(result['parameter'])
            experiments[exp_name]['final'].append(result['final_fitness'])
            experiments[exp_name]['avg'].append(result['avg_fitness'])
        
        fig, axes = plt.subplots(1, len(experiments), figsize=(15, 5))
        if len(experiments) == 1:
            axes = [axes]
        
        for i, (exp_name, data) in enumerate(experiments.items()):
            axes[i].plot(data['params'], data['final'], 'o-', label='Final Fitness', linewidth=2)
            axes[i].plot(data['params'], data['avg'], 's--', label='Avg Fitness', alpha=0.7)
            axes[i].set_xlabel(exp_name.replace('_', ' ').title())
            axes[i].set_ylabel('Fitness')
            axes[i].set_title(f'{exp_name.replace("_", " ").title()} Experiment')
            axes[i].legend()
            axes[i].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(f'coursework_experiments_{self.timestamp}.png', dpi=300, bbox_inches='tight')
        plt.show()
        
        print(f"\nExperiment results saved to: {csv_filename}")
        print("Comparison plots saved and displayed.")
        
        # Print summary with optimized parameters
        print("\n=== EXPERIMENT SUMMARY ===")
        for exp_name, data in experiments.items():
            print(f"\n{exp_name.replace('_', ' ').title()}:")
            for i, param in enumerate(data['params']):
                result = self.results[i] if exp_name == 'population_size' else None
                if result and 'mutation_rate' in result:
                    print(f"  Population {param}: Best fitness {data['final'][i]:.2f} "
                          f"(Mutation: {result['mutation_rate']}, Genes: {result['gene_count']})")
                else:
                    print(f"  {param}: Best fitness {data['final'][i]:.2f}")

def run_coursework_experiments():
    """Run all coursework experiments and save results."""
    print("="*60)
    print("COURSEWORK SYSTEMATIC EXPERIMENTS")
    print("="*60)
    
    experiments = CourseWorkExperiments()
    
    # Run only population size experiments (3 tests)
    experiments.run_population_size_experiment()
    # experiments.run_mutation_rate_experiment()  # Commented out
    # experiments.run_gene_count_experiment()     # Commented out
    
    # Save and plot results
    experiments.save_and_plot_results()
    
    return experiments
# === END COURSEWORK REQUIREMENT ===

def run_mountain_climbing_evolution():
    """Main entry point for running the mountain climbing genetic algorithm."""
    
    print("="*60)
    print("COURSEWORK: MOUNTAIN CLIMBING GENETIC ALGORITHM")
    print("="*60)
    print("This system meets all coursework requirements:")
    print("✓ Integration with prepare_shapes.py for mountain generation")
    print("✓ Fitness based on mountain climbing ability")
    print("✓ Systematic experiments with different GA settings")
    print("✓ Comprehensive results analysis and visualization")
    print()
    
    # Mountain type selection with prepare_shapes integration
    print("Choose mountain type:")
    print("  1. Default (Gaussian)")
    print("  2. Steep")
    print("  3. Ridge") 
    print("  4. Plateau")
    print("  5. Run Systematic Experiments")
    
    choice = input("Enter choice [1-5]: ").strip()
    
    if choice == '5':
        # Run systematic experiments
        experiments = run_coursework_experiments()
        return None, experiments.results, None
    
    # Handle mountain generation
    mountain_type = 'default'
    if choice == '2':
        mountain_type = 'steep'
    elif choice == '3':
        mountain_type = 'ridge'
    elif choice == '4':
        mountain_type = 'plateau'
    
    # Camera test - automatically show mountain for options 1-4
    print("\nShowing mountain view...")
    camera_test(mountain_type if mountain_type in ['default','steep','ridge','plateau'] else 'default')
    print("Mountain view closed. Continuing with evolution setup...")
    
    # Ask for GA parameters
    def get_int(prompt, default):
        try:
            return int(input(f"{prompt} [{default}]: ") or default)
        except:
            return default
    def get_float(prompt, default):
        try:
            return float(input(f"{prompt} [{default}]: ") or default)
        except:
            return default
    population_size = get_int("Population size", 24)
    gene_count = get_int("Gene count", 6)
    generations = get_int("Number of generations", 40)
    mutation_rate = get_float("Mutation rate", 0.2)
    crossover_rate = get_float("Crossover rate", 0.75)
    elite_count = get_int("Elite count", 3)
    simulation_time = get_int("Simulation time (seconds)", 15)
    show_gui_every = get_int("Show GUI every N generations", 2)
    config = {
        'population_size': population_size,
        'gene_count': gene_count,
        'generations': generations,
        'mutation_rate': mutation_rate,
        'crossover_rate': crossover_rate,
        'elite_count': elite_count,
        'simulation_time': simulation_time,
        'show_gui_every': show_gui_every,
        'mountain_type': mountain_type
    }
    print("Evolution Configuration:")
    for key, value in config.items():
        print(f"  {key}: {value}")
    print()
    ga = MountainClimbingGA(
        population_size=config['population_size'],
        gene_count=config['gene_count'],
        mutation_rate=config['mutation_rate'],
        crossover_rate=config['crossover_rate'],
        elite_count=config['elite_count'],
        simulation_time=config['simulation_time'],
        mountain_type=config['mountain_type']
    )
    try:
        print("Starting evolution...")
        fitness_history, best_creatures = ga.run_evolution(
            generations=config['generations'],
            save_results=True,
            show_gui_every=config['show_gui_every']
        )
        
        # Print final results
        print("\n" + "="*60)
        print("EVOLUTION COMPLETED!")
        print("="*60)
        
        if fitness_history:
            final_stats = fitness_history[-1]
            print(f"Final Generation: {final_stats['generation'] + 1}")
            print(f"Best Fitness: {final_stats['best_fitness']:.2f}")
            print(f"Average Fitness: {final_stats['avg_fitness']:.2f}")
            print(f"Fitness Standard Deviation: {final_stats['std_fitness']:.2f}")
            
            if best_creatures:
                best_final = best_creatures[-1]
                details = best_final['details']
                print(f"\nBest Creature Performance:")
                print(f"  Max Height: {details['max_height']:.2f}")
                print(f"  Mountain Progress: {details['max_mountain_progress']:.2f}")
                print(f"  Contact Ratio: {details['contact_ratio']:.3f}")
                print(f"  Tests Passed: {details['num_tests']}")
        
        # Ask if user wants to see demonstration
        print("\n" + "-"*40)
        response = input("Would you like to see the best creature in action? (y/n): ").lower().strip()
        if response.startswith('y'):
            ga.demonstrate_best_creature(show_multiple=True)
        
        return ga, fitness_history, best_creatures
        
    except KeyboardInterrupt:
        print("\nEvolution interrupted by user.")
        return ga, ga.fitness_history, ga.best_creature_history
    except Exception as e:
        print(f"\nError during evolution: {e}")
        import traceback
        traceback.print_exc()
        return None, None, None

def analyze_evolution_results(fitness_history, best_creatures):
    """Analyze and print summary statistics for the evolution results."""
    if not fitness_history or not best_creatures:
        print("No results to analyze!")
        return
    
    print("\n" + "="*50)
    print("DETAILED EVOLUTION ANALYSIS")
    print("="*50)
    
    # Fitness progression analysis
    best_fitnesses = [h['best_fitness'] for h in fitness_history]
    avg_fitnesses = [h['avg_fitness'] for h in fitness_history]
    
    print(f"Fitness Progression:")
    print(f"  Initial Best: {best_fitnesses[0]:.2f}")
    print(f"  Final Best: {best_fitnesses[-1]:.2f}")
    print(f"  Total Improvement: {best_fitnesses[-1] - best_fitnesses[0]:.2f}")
    print(f"  Average Improvement per Generation: {(best_fitnesses[-1] - best_fitnesses[0]) / len(best_fitnesses):.2f}")
    
    # Find biggest improvements
    improvements = [best_fitnesses[i] - best_fitnesses[i-1] for i in range(1, len(best_fitnesses))]
    best_improvement_gen = improvements.index(max(improvements)) + 2  # +2 because we start from index 1, and generations are 1-indexed
    
    print(f"\nBiggest Single Generation Improvement:")
    print(f"  Generation: {best_improvement_gen}")
    print(f"  Improvement: {max(improvements):.2f}")
    
    # Performance milestones
    print(f"\nPerformance Milestones:")
    milestones = [10, 25, 50, 75, 100, 150, 200]
    for milestone in milestones:
        gen_reached = next((i for i, f in enumerate(best_fitnesses) if f >= milestone), None)
        if gen_reached is not None:
            print(f"  Fitness {milestone}: Reached in generation {gen_reached + 1}")
    
    # Climbing performance analysis
    max_heights = [bc['details']['max_height'] for bc in best_creatures]
    mountain_progress = [bc['details']['max_mountain_progress'] for bc in best_creatures]
    
    print(f"\nClimbing Performance:")
    print(f"  Initial Max Height: {max_heights[0]:.2f}")
    print(f"  Final Max Height: {max_heights[-1]:.2f}")
    print(f"  Best Mountain Progress: {max(mountain_progress):.2f}")
    print(f"  Final Mountain Progress: {mountain_progress[-1]:.2f}")
    
    print("\n" + "="*50)

if __name__ == "__main__":
    # Set random seeds for reproducibility (optional)
    # random.seed(42)
    # np.random.seed(42)
    
    print("Starting Mountain Climbing Evolution Experiment...")
    print("Make sure you have the required modules: creature, population, simulation")
    print()
    
    # Run the evolution
    ga, fitness_history, best_creatures = run_mountain_climbing_evolution()
    
    if ga and fitness_history and best_creatures:
        # Analyze results
        analyze_evolution_results(fitness_history, best_creatures)
        
        print("\n✓ Coursework completed successfully!")
        print("✓ Check generated files for detailed results and visualizations.")
    elif fitness_history:  # Systematic experiments were run
        print("\n✓ Systematic experiments completed successfully!")
        print("✓ Check generated CSV and PNG files for results.")
    else:
        print("Evolution failed or was interrupted.")

