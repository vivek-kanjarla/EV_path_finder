import sys
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import heapq
from geopy.distance import geodesic
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QLineEdit, QPushButton, QHBoxLayout, QVBoxLayout, QWidget, QMessageBox, QSizePolicy
from PyQt5.QtCore import Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
import mplcursors
import random


class CustomNavigationToolbar(NavigationToolbar):
    toolitems = [t for t in NavigationToolbar.toolitems if t[0] in ('Home', 'Back', 'Forward', 'Pan', 'Zoom', 'Save')]

    def __init__(self, canvas, parent, coordinates=True):
        super().__init__(canvas, parent, coordinates=coordinates)

def compute_energy_consumption(length, speeds, slope, vehicle_params):
    m, g = vehicle_params['mass'], 9.81
    rho, Cd, A = vehicle_params['air_density'], vehicle_params['drag_coefficient'], vehicle_params['frontal_area']
    Cr, eta = vehicle_params['rolling_resistance_coefficient'], vehicle_params['recovery_efficiency']
    total_energy = 0
    segment_length = length / len(speeds)
    for speed in speeds:
        speed_mps = speed * 0.44704
        F_drag = 0.5 * rho * Cd * A * speed_mps**2
        F_roll = Cr * m * g
        F_gravity = m * g * slope
        E_drag = F_drag * speed_mps * (segment_length / speed_mps) / 3600
        E_roll = F_roll * speed_mps * (segment_length / speed_mps) / 3600
        E_gravity = F_gravity * speed_mps * (segment_length / speed_mps) / 3600
        E_total_consumed = E_drag + E_roll + E_gravity
        E_recovery = eta * max(0, E_gravity)
        total_energy += (E_total_consumed - E_recovery)
    return total_energy

def update_soc(current_soc, energy_consumed, battery_capacity):
    new_soc = current_soc - energy_consumed
    if new_soc < 0:
        new_soc = 0  
    elif new_soc > battery_capacity:
        new_soc = battery_capacity  
    return new_soc

class Label:
    def __init__(self, vertex, soc, time, path, charging_info=None):
        self.vertex = vertex
        self.soc = soc
        self.time = time
        self.path = path
        self.charging_info = charging_info if charging_info is not None else []  
    def __lt__(self, other):
        return self.time < other.time
    
def propagate_label(current_label, target_vertex, edge, vehicle_params, current_time):
    length, slope = edge['length'], edge['slope']
    min_speed, max_speed = edge['min_speed'], edge['max_speed']
    speeds = np.linspace(min_speed, max_speed, num=5)
    best_energy = compute_energy_consumption(length, speeds, slope, vehicle_params)
    new_soc = update_soc(current_label.soc, best_energy, vehicle_params['battery_capacity'])
    average_speed = np.mean(speeds)
    travel_time = length / average_speed
    new_time = current_label.time + travel_time
    new_charging_info = current_label.charging_info.copy()

    node_info = G.nodes[target_vertex]
    if node_info['type'] == 'charging':
        window_start, window_end = time_windows[node_info['time_window']]
        if window_start <= current_time % 24 <= window_end:
            needed_charge = vehicle_params['battery_capacity'] - new_soc
            if needed_charge > 0:
                for mode in node_info['charging_modes']:
                    charging_time = charging_modes[mode]['time_to_charge'](needed_charge)
                    new_time += charging_time
                    new_soc = update_soc(current_label.soc + needed_charge, best_energy, vehicle_params['battery_capacity'])
                    new_charging_info.append((target_vertex, needed_charge))
    return Label(target_vertex, new_soc, new_time, current_label.path + [target_vertex], new_charging_info)

def shortest_path_with_energy_and_backtracking(graph, source, target, initial_SoC, vehicle_params):
    pq = [Label(source, initial_SoC, 0, [source])]
    visited = set()
    while pq:
        current_label = heapq.heappop(pq)
        if current_label.vertex in visited:
            continue
        if current_label.vertex == target:
            return current_label
        visited.add(current_label.vertex)
        current_time = current_label.time
        for next_vertex in graph.adj[current_label.vertex]:  
            edge = graph.get_edge_data(current_label.vertex, next_vertex)  
            new_label = propagate_label(current_label, next_vertex, edge, vehicle_params, current_time)
            if new_label.soc > 0:
                heapq.heappush(pq, new_label)
    return None

def display_path(path_result):
    if path_result:
        path = path_result.path
        if len(path) == 1:
            result = f"The path taken by the car is: <span style='color:green;'>{path[0]}</span>"
        else:
            first_node = f"<span style='color:green;'>{path[0]}</span>"
            last_node = f"<span style='color:red;'>{path[-1]}</span>"
            middle_nodes = " --> ".join(path[1:-1])
            if middle_nodes:
                result = f"The path taken by the car is: {first_node} --> {middle_nodes} --> {last_node}"
            else:
                result = f"The path taken by the car is: {first_node} --> {last_node}\n"

            result += f"<br>Path found with total time: {path_result.time:.2f} hours and remaining SoC: {path_result.soc:.2f} kWh"
    else:
        result = "No feasible path found."
    return result

def on_calculate():
        source = "City_" + ex.source_entry.text()
        target = "City_" + ex.target_entry.text()

        if source not in G.nodes or target not in G.nodes:
            QMessageBox.critical(ex, "Error", "Please enter valid source and target nodes.")
            return
        
        
        for param, widget in ex.vehicle_params.items():
            vehicle_params[f'{param.replace(" ", "_").lower()}'] = float(widget.text())

        path_result = shortest_path_with_energy_and_backtracking(G, source, target, vehicle_params['battery_capacity'], vehicle_params)
        ex.result_label.setText(display_path(path_result))
        ex.plot_graph(G, source, target, path_result)
        ex.update_soc_graph(path_result)
        ex.update_energy_consumption_graph(path_result)
        ex.canvas.draw()

def on_generate_map():
    try:
        num_cities = int(ex.num_cities_entry.text())
        charging_station_ratio = float(ex.charging_station_ratio_entry.text())
    except ValueError:
        QMessageBox.critical(ex, "Error", "Please enter valid numerical values for number of cities and charging station ratio.")
        return

    ex.generate_random_map(num_cities, charging_station_ratio)
    ex.plot_initial_graph(G)
    ex.canvas.draw()

class EVPathFinderApp(QMainWindow):
    def __init__(self):
        super().__init__()

        self.initUI()
        self.cursor = None
        self.soc_cursor = None
        self.ec_cursor = None
        self.generate_random_map(num_cities, charging_station_ratio)
        self.plot_initial_graph(G)

    def initUI(self):
        self.setWindowTitle('EV Path Finder')
        self.showMaximized()

        
        self.setStyleSheet("background-color: grey;")

        main_layout = QVBoxLayout()

        content_layout = QHBoxLayout()

   
        left_layout = QVBoxLayout()

      
        map_heading = QLabel('Map Generation', self)
        map_heading.setStyleSheet("color: white; font-size: 16px; font-weight: bold;")
        map_heading.setAlignment(Qt.AlignLeft)
        left_layout.addWidget(map_heading)

        map_layout = QVBoxLayout()

        cities_layout = QHBoxLayout()
        self.num_cities_label = QLabel('Number of Cities:', self)
        self.num_cities_label.setStyleSheet("color: white;")
        self.num_cities_label.setAlignment(Qt.AlignRight)
        cities_layout.addWidget(self.num_cities_label)

        self.num_cities_entry = QLineEdit(self)
        self.num_cities_entry.setStyleSheet("background-color: white; color: black;")
        self.num_cities_entry.setMaximumWidth(100)
        self.num_cities_entry.setText("200")
        cities_layout.addWidget(self.num_cities_entry)

        station_ratio_layout = QHBoxLayout()
        self.charging_station_ratio_label = QLabel('Charging Station Ratio:', self)
        self.charging_station_ratio_label.setStyleSheet("color: white;")
        self.charging_station_ratio_label.setAlignment(Qt.AlignRight)
        station_ratio_layout.addWidget(self.charging_station_ratio_label)

        self.charging_station_ratio_entry = QLineEdit(self)
        self.charging_station_ratio_entry.setStyleSheet("background-color: white; color: black;")
        self.charging_station_ratio_entry.setMaximumWidth(100)
        self.charging_station_ratio_entry.setText("0.4")
        station_ratio_layout.addWidget(self.charging_station_ratio_entry)

        generate_button_layout = QHBoxLayout()
        self.generate_map_button = QPushButton('Generate Random Map', self)
        self.generate_map_button.setStyleSheet("background-color: lightblue; color: black;")
        self.generate_map_button.setMaximumWidth(200)
        self.generate_map_button.clicked.connect(on_generate_map)
        generate_button_layout.addWidget(self.generate_map_button, alignment=Qt.AlignCenter)
        
        map_layout.addLayout(cities_layout)
        map_layout.addLayout(station_ratio_layout)
        map_layout.addLayout(generate_button_layout)

        map_container = QWidget()
        map_container.setLayout(map_layout)
        map_container.setStyleSheet("border: 1px solid white; padding: 10px;")

        left_layout.addWidget(map_container)

        input_heading = QLabel('Path Calculation', self)
        input_heading.setStyleSheet("color: white; font-size: 16px; font-weight: bold;")
        input_heading.setAlignment(Qt.AlignLeft)
        left_layout.addWidget(input_heading)

        top_layout = QVBoxLayout()

        source_layout = QHBoxLayout()
        self.source_label = QLabel('Source:', self)
        self.source_label.setStyleSheet("color: white;")
        self.source_label.setAlignment(Qt.AlignRight)
        source_layout.addWidget(self.source_label)
        
        self.source_entry = QLineEdit(self)
        self.source_entry.setStyleSheet("background-color: white; color: black;")
        self.source_entry.setMaximumWidth(150)
        source_layout.addWidget(self.source_entry)
        
        target_layout = QHBoxLayout()
        self.target_label = QLabel('Target:', self)
        self.target_label.setStyleSheet("color: white;")
        self.target_label.setAlignment(Qt.AlignRight) 
        target_layout.addWidget(self.target_label)
        
        self.target_entry = QLineEdit(self)
        self.target_entry.setStyleSheet("background-color: white; color: black;")
        self.target_entry.setMaximumWidth(150)
        target_layout.addWidget(self.target_entry)   

        top_layout.addLayout(source_layout)
        top_layout.addLayout(target_layout)

        vehicle_heading = QLabel('Vehicle Parameters', self)
        vehicle_heading.setStyleSheet("color: white; font-size: 16px; font-weight: bold;")
        vehicle_heading.setAlignment(Qt.AlignCenter)
        top_layout.addWidget(vehicle_heading)

        vehicle_layout = QVBoxLayout()

        self.vehicle_params = {}
        max_width = 80  
        for param, value in vehicle_params.items():
            param_layout = QHBoxLayout()
            label = QLabel(f'{param.replace("_", " ").capitalize()}:', self)
            label.setStyleSheet("color: white;")
            label.setAlignment(Qt.AlignRight)  
            param_layout.addWidget(label)
            widget = QLineEdit(str(value), self)
            widget.setStyleSheet("background-color: white; color: black;")
            widget.setMaximumWidth(max_width)
            param_layout.addWidget(widget)
            vehicle_layout.addLayout(param_layout)
            self.vehicle_params[param] = widget

        calculate_button_layout = QHBoxLayout()
        self.calculate_button = QPushButton('Calculate Path', self)
        self.calculate_button.setStyleSheet("background-color: lightblue; color: black;")
        self.calculate_button.clicked.connect(on_calculate)
        calculate_button_layout.addWidget(self.calculate_button, alignment=Qt.AlignCenter)

        top_layout.addLayout(vehicle_layout)
        top_layout.addLayout(calculate_button_layout)

        input_container = QWidget()
        input_container.setLayout(top_layout)
        input_container.setStyleSheet("border: 1px solid white; padding: 10px;")

        left_layout.addWidget(input_container)

        left_container = QWidget()
        left_container.setLayout(left_layout)
        left_container.setMinimumWidth(250)
        left_container.setMaximumWidth(350)

        content_layout.addWidget(left_container)

        right_layout = QVBoxLayout()
        right_layout.setSpacing(0)  
        right_layout.setContentsMargins(0, 0, 0, 0)  

        first_row_layout = QHBoxLayout()
        first_row_container = QWidget()
        first_row_container.setLayout(first_row_layout)
        first_row_container.setMinimumWidth(400)

        self.figure1, self.ax1 = plt.subplots(figsize=(4, 3))  
        self.soc_canvas = FigureCanvas(self.figure1)
        self.soc_canvas.setStyleSheet("background-color: grey;")
        first_row_layout.addWidget(self.soc_canvas)

        self.figure2, self.ax2 = plt.subplots(figsize=(4, 3))  
        self.ec_canvas = FigureCanvas(self.figure2)
        self.ec_canvas.setStyleSheet("background-color: grey;")
        first_row_layout.addWidget(self.ec_canvas)

        right_layout.addWidget(first_row_container)


        self.figure, self.ax = plt.subplots(figsize=(8, 4))  
        self.figure.subplots_adjust(left=0.01, right=0.99, top=0.94, bottom=0.01)
        self.ax.autoscale_view()

        self.canvas = FigureCanvas(self.figure)
        self.canvas.setStyleSheet("background-color: grey;")
        
        self.toolbar = CustomNavigationToolbar(self.canvas, self)
        self.toolbar.setMinimumHeight(30)
        self.toolbar.setMaximumHeight(30)
        right_layout.addWidget(self.toolbar)  

        right_layout.addWidget(self.canvas)

       
        content_layout.addLayout(right_layout)

        self.result_heading = QLabel('Route and Energy Consumption Data', self)
        self.result_heading.setStyleSheet("color: white; font-size: 14px; font-weight: bold;")

        
        self.result_label = QLabel('', self)
        self.result_label.setStyleSheet("color: white; background-color: grey;")
        self.result_label.setFixedHeight(50)  
        self.result_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.result_label.setWordWrap(True)  
        self.result_label.setTextFormat(Qt.RichText)  
                
        
        footer_layout = QHBoxLayout()
        footer_layout.addWidget(self.result_heading)
        footer_layout.addWidget(self.result_label)

    
        main_layout.addLayout(content_layout)
        main_layout.addLayout(footer_layout)

        
        central_widget = QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)


    def generate_random_map(self, num_cities, charging_station_ratio):
        self.ax.clear()
        self.ax1.clear()
        self.ax2.clear()
        self.ec_canvas.draw_idle()  
        self.soc_canvas.draw_idle()  
        self.canvas.draw_idle()
        
       
        if self.cursor:
            self.cursor.remove()
            self.cursor = None

        global G
        G = nx.Graph()
        np.random.seed(42)
        city_coords = [(np.random.uniform(24.396308, 49.384358), np.random.uniform(-125.0, -66.93457)) for _ in range(num_cities)]

        for i, coords in enumerate(city_coords):
            is_charging = np.random.rand() < charging_station_ratio
            node_type = 'charging' if is_charging else 'regular'
            available_modes = np.random.choice(list(charging_modes.keys()), size=np.random.randint(1, 4), replace=False) if is_charging else []
            time_window = np.random.choice(list(time_windows.keys())) if is_charging else None
            G.add_node(f'City_{i}', coords=coords, type=node_type, charging_modes=available_modes, time_window=time_window)

        for i, u in enumerate(G.nodes):
            distances = [(geodesic(G.nodes[u]['coords'], G.nodes[v]['coords']).km, v) for v in G.nodes if u != v]
            distances.sort()
            for _, v in distances[:5]:
                length = geodesic(G.nodes[u]['coords'], G.nodes[v]['coords']).km
                G.add_edge(u, v, length=length, slope=np.random.uniform(-0.03, 0.03), min_speed=30, max_speed=70)

    def plot_initial_graph(self, G):
        pos = {node: (attr['coords'][1], attr['coords'][0]) for node, attr in G.nodes(data=True)}
        nodes = nx.draw_networkx_nodes(G, pos, node_color='blue', node_size=50, ax=self.ax)
        nx.draw_networkx_nodes(G, pos, nodelist=[node for node, attr in G.nodes(data=True) if attr.get('type') == 'charging'], node_color='orange', node_size=50, ax=self.ax)
        nx.draw_networkx_edges(G, pos, edge_color='gray', ax=self.ax)
        label_pos = {node: (coords[0], coords[1] + 1) for node, coords in pos.items()} 
        node_labels = {node: node.split('_')[1] for node in G.nodes()}
        nx.draw_networkx_labels(G, pos=label_pos, labels=node_labels, font_size=8, font_color='black', ax=self.ax)
        self.ax.set_title("Initial Graph of Nodes")
        self.ax.axis('off')
        self.canvas.draw()
        self.cursor = mplcursors.cursor(nodes, hover=True)
        self.cursor.connect("add", lambda sel: sel.annotation.set_text(
            'Node: {} - {}'.format(
                sel.target.index, 
                "Charging Station" if G.nodes[list(G.nodes)[sel.target.index]]['type'] == 'charging' else "Regular"
            )
        ))

    def update_soc_graph(self, path_result):
        if self.soc_cursor:
            self.soc_cursor.remove()
            self.soc_cursor = None

        if path_result:
            soc_values = []
            nodes = path_result.path
            node_indices = [node.split('_')[1] for node in nodes]
            current_soc = vehicle_params['battery_capacity']
            soc_values.append(current_soc)

            for i in range(len(nodes) - 1):
                edge = G.get_edge_data(nodes[i], nodes[i+1])
                length = edge['length']
                slope = edge['slope']
                min_speed, max_speed = edge['min_speed'], edge['max_speed']
                speeds = np.linspace(min_speed, max_speed, num=5)
                energy_consumed = compute_energy_consumption(length, speeds, slope, vehicle_params)
                charge = next((charge for node, charge in path_result.charging_info if node == nodes[i+1]), 0)
                
                current_soc = update_soc(current_soc+charge, energy_consumed, vehicle_params['battery_capacity'])
                soc_values.append(current_soc)

            self.ax1.clear()
            line = self.ax1.plot(node_indices, soc_values, marker='o', linestyle='-', color='b')
            self.ax1.set_title('State of Charge (SoC) over Path')
            self.ax1.set_xlabel('Nodes (Cities)')
            self.ax1.set_ylabel('State of Charge (kWh)')
            self.ax1.grid(True)
            self.ax1.set_xticks(node_indices)
            self.soc_canvas.draw()
            self.soc_cursor = mplcursors.cursor(line, hover=True)
            self.soc_cursor.connect("add", lambda sel: sel.annotation.set_text(
                'SoC: {:.2f} kWh'.format(sel.target[1])  
            ))
        else:
            self.ax1.clear()
            self.ax1.set_title('State of Charge (SoC) over Path')
            self.ax1.set_xlabel('Nodes (Cities)')
            self.ax1.set_ylabel('State of Charge (kWh)')
            self.ax1.grid(True)
            self.soc_canvas.draw()

    def update_energy_consumption_graph(self, path_result):
        
        if self.ec_cursor:
            self.ec_cursor.remove()
            self.ec_cursor = None

        if path_result:
            energy_consumptions = [0]
            nodes = path_result.path
            node_indices = [node.split('_')[1] for node in nodes]

            for i in range(len(nodes) - 1):
                edge = G.get_edge_data(nodes[i], nodes[i + 1])
                length = edge['length']
                slope = edge['slope']
                min_speed, max_speed = edge['min_speed'], edge['max_speed']
                speeds = np.linspace(min_speed, max_speed, num=5)
                energy_consumed = compute_energy_consumption(length, speeds, slope, vehicle_params)
                energy_consumptions.append(energy_consumed)


            self.ax2.clear()
            line = self.ax2.plot(node_indices, energy_consumptions, marker='o', linestyle='-', color='r')
            self.ax2.set_title('Energy Consumption over Path')
            self.ax2.set_xlabel('Nodes (Cities)')
            self.ax2.set_ylabel('Energy Consumption (kWh)')
            self.ax2.grid(True)
            self.ax1.set_xticks(node_indices)
            self.ec_canvas.draw()

            self.ec_cursor = mplcursors.cursor(line, hover=True)
            self.ec_cursor.connect("add", lambda sel: sel.annotation.set_text(
                'SoC: {:.2f} kWh'.format(sel.target[1])  
            ))
        else:
            self.ax2.clear()
            self.ax2.set_title('Energy Consumption over Path')
            self.ax2.set_xlabel('Nodes (Cities)')
            self.ax2.set_ylabel('Energy Consumption (kWh)')
            self.ax2.grid(True)
            self.ec_canvas.draw()


    def plot_graph(self, graph, source, target, path_result=None):
        self.ax.clear()  
        
        if self.cursor:
            self.cursor.remove()
            self.cursor = None
        if self.soc_cursor:
            self.soc_cursor.remove()
            self.soc_cursor = None
        if self.ec_cursor:
            self.ec_cursor.remove()
            self.ec_cursor = None

        pos = {node: (attr['coords'][1], attr['coords'][0]) for node, attr in graph.nodes(data=True)}
        nodes = nx.draw_networkx_nodes(graph, pos, node_color='blue', node_size=50, ax=self.ax)
        nx.draw_networkx_nodes(graph, pos, nodelist=[node for node, attr in graph.nodes(data=True) if attr.get('type') == 'charging'], node_color='orange', node_size=50, ax=self.ax)
        nx.draw_networkx_nodes(graph, pos, nodelist=[source], node_color='green', node_size=100, ax=self.ax)
        nx.draw_networkx_nodes(graph, pos, nodelist=[target], node_color='red', node_size=100, ax=self.ax)
        nx.draw_networkx_edges(graph, pos, edge_color='gray', ax=self.ax)
        if path_result:
            path_edges = list(zip(path_result.path, path_result.path[1:]))
            nx.draw_networkx_edges(graph, pos, edgelist=path_edges, edge_color='red', width=2, ax=self.ax)
        
        
        label_pos = {node: (coords[0], coords[1] + 1) for node, coords in pos.items()}  
        node_labels = {node: node.split('_')[1] for node in graph.nodes()}
        nx.draw_networkx_labels(graph, pos=label_pos, labels=node_labels, font_size=8, font_color='black', ax=self.ax)

        self.cursor = mplcursors.cursor(nodes, hover=True)
        self.cursor.connect("add", lambda sel: sel.annotation.set_text(
            'Node: {} - {}'.format(
                sel.target.index, 
                "Charging Station" if graph.nodes[list(graph.nodes)[sel.target.index]]['type'] == 'charging' else "Regular"
            )
        ))

        self.ax.set_title("Graph Representation of Constrained Shortest Path Algorithm for BEVs")
        self.ax.axis('off')
        self.canvas.draw()  
    
if __name__ == '__main__':

    vehicle_params = {
        'mass': 1500,
        'air_density': 1.225,
        'drag_coefficient': 0.3,
        'frontal_area': 2.2,
        'rolling_resistance_coefficient': 0.015,
        'recovery_efficiency': 0.6,
        'battery_capacity': 200.0
    }

    num_cities = 200
    charging_station_ratio = 0.4
    
    charging_modes = {
        'slow': {'rate': 22, 'time_to_charge': lambda x: x / 22},
        'fast': {'rate': 50, 'time_to_charge': lambda x: x / 50},
        'super_fast': {'rate': 120, 'time_to_charge': lambda x: x / 120}
    }

    time_windows = {
        'morning': (6, 12),
        'afternoon': (12, 18),
        'night': (18, 24)
    }
    
    app = QApplication(sys.argv)
    ex = EVPathFinderApp()
    ex.show()

    sys.exit(app.exec_())