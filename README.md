Overview

The EV Path Finder is a Python-based application designed to compute energy-efficient paths for electric vehicles (EVs) across a generated map of cities. 
The program uses graph theory to determine the optimal route based on energy consumption and charging constraints, ensuring that EVs can reach their destination 
efficiently while considering charging station availability.

Features

1)Graph-based Pathfinding: Utilizes a constrained shortest path algorithm to find the most efficient route.

2)Energy Consumption Calculation: Computes energy usage considering rolling resistance, aerodynamic drag, and gravitational effects.

3)State of Charge (SoC) Tracking: Monitors battery charge levels and recharges at charging stations when needed.

4)Charging Stations: Randomly generates cities with charging stations and supports different charging modes (slow, fast, super-fast).

5)Dynamic Graph Visualization: Displays nodes (cities) and edges (roads) on an interactive map.

6)Custom Vehicle Parameters: Allows users to input vehicle parameters such as mass, drag coefficient, and battery capacity.

7)GUI with PyQt5: Provides an interactive interface for users to generate maps, select routes, and visualize results.

Requirements

Python 3.8+

Required Libraries:

numpy
matplotlib
networkx
PyQt5
geopy
mplcursors
heapq

To install dependencies, run:

pip install numpy matplotlib networkx PyQt5 geopy mplcursors

Usage

Running the Application

Clone the repository or download the source code.

Run the main script:

python Main.py

The GUI will open, allowing users to:

*Generate a random map of cities.
*Input the source and destination cities.
*Configure vehicle parameters.
*Compute and visualize the best path based on energy constraints.

Input Parameters

Number of Cities: Total number of cities to be generated.
Charging Station Ratio: Percentage of cities with charging stations.

Vehicle Parameters:

Mass (kg)
Air Density (kg/m³)
Drag Coefficient
Frontal Area (m²)
Rolling Resistance Coefficient
Recovery Efficiency
Battery Capacity (kWh)

Output

Shortest Path: Displays the optimal route taken by the EV.
Graph Representation: Shows the network of cities and roads.
State of Charge Graph: Visualizes battery SoC along the journey.
Energy Consumption Graph: Illustrates energy usage over the path.

Code Structure

Main.py: The main script containing the GUI and all computational logic.
compute_energy_consumption(): Function to estimate energy usage.
update_soc(): Function to update the battery state of charge.
shortest_path_with_energy_and_backtracking(): Core algorithm to find the optimal path considering charging constraints.
EVPathFinderApp: PyQt5-based GUI class for visualization and user interaction.
