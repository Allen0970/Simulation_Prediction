import os
import sys
import csv
import xml.etree.ElementTree as ET
import traci
import pandas as pd

 
'''
# Change the following path to where you store Sumo tools
sumo_tools_path = '/$SUMO_HOME/tools'

# Check if the SUMO tools directory is already in sys.path
if sumo_tools_path not in sys.path:
    sys.path.append(sumo_tools_path)

try:
    import traci
except ModuleNotFoundError:
    print("traci module not found. Ensure SUMO tools path is correctly added to Python path.")
    sys.exit(1)
'''

# Get SUMO_HOME environment
sumo_home = os.environ.get('SUMO_HOME')
if sumo_home is None:
    print("SUMO_HOME environment variable not set.")
    sys.exit(1)

# Build SUMO tools directory path
sumo_tools_path = os.path.join(sumo_home, 'tools')

# Check and add to sys.path
if sumo_tools_path not in sys.path:
    sys.path.append(sumo_tools_path)

try:
    import traci
except ModuleNotFoundError:
    print("traci module not found. Ensure SUMO tools path is correctly added to Python path.")
    sys.exit(1)

# Get the turning direction of the current edge and the next edge
def get_next_edge_direction(vehicle_id, current_edge, connections):
    route = traci.vehicle.getRoute(vehicle_id)
    current_edge_index = route.index(current_edge)

    if current_edge_index < len(route) - 1:
        next_edge = route[current_edge_index + 1]
        # Find the connection direction to the next edge
        if current_edge in connections:
            for conn in connections[current_edge]:
                if conn['to'] == next_edge:
                    return conn['dir']                    
        else:
            print(f"Error. {current_edge} does not be found in dic.")
            return "unknown"
    else:
        return "end"
    
    print(f"Error. current edge {current_edge} does not find next edge {next_edge} in dic.")
    return "unknown"
    
# Extract the edge speed limit, length, and connection relationship between two edges
def parse_sumo_network(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()

    # Mapping 1: Edge attributes
    edge_attributes = {}
    for edge in root.findall('edge'):
        if 'function' not in edge.attrib:  # Ignore internal edges
            edge_id = edge.attrib['id']
            lanes = edge.findall('lane')
            total_speed = sum(float(lane.attrib['speed']) for lane in lanes)
            total_length = sum(float(lane.attrib['length']) for lane in lanes)
            avg_speed = total_speed / len(lanes) if lanes else 0
            avg_length = total_length / len(lanes) if lanes else 0
            edge_attributes[edge_id] = {'lanes': len(lanes), 'avg_speed': avg_speed, 'avg_length': avg_length}

    # Map 2: Edge Connection
    edge_connections = {}
    for connection in root.findall('connection'):
        from_edge = connection.attrib['from']
        to_edge = connection.attrib['to']
        direction = connection.attrib.get('dir', 'unknown')  # default value is 'unknown'
        if from_edge not in edge_connections:
            edge_connections[from_edge] = []
        edge_connections[from_edge].append({'to': to_edge, 'dir': direction})

    return edge_attributes, edge_connections

file_path = 'test.net.xml'
edge_attrs, edge_conns = parse_sumo_network(file_path)

# Update the information under the dictionary traffic_data
def update_vehicle_data(traffic_data, v_id, current_edge, data):
    if v_id not in traffic_data:
        traffic_data[v_id] = {}
    traffic_data[v_id][current_edge] = data

def process_single_vehicle_data(row_data, turn_stat_col_name):
    
    # map direction
    direction_mapping = {
        's': 'straight',
        't': 'turn',
        'l': 'left',
        'r': 'right',
        'L': 'partially_left',
        'R': 'partially_right',
        'end': 'end_road'
    }

    # Initialize new turn statistics fields
    for direction in direction_mapping.values():
        row_data[direction] = 0
        
    turns = row_data[turn_stat_col_name]
    for turn in turns:
        turn_col = direction_mapping.get(turn, 'invalid')
        row_data[turn_col] += 1

    return row_data

# Create a class to store vehicle information
class Vehicle:
    def __init__(self, id):
        self.id = id
        self.current_edge = None
        self.edge_enter_time = None
        # Driving time
        self.driving_time = 0
        # Stop + Red Light -> Normal Waiting Time
        self.is_waiting = False
        self.red_light_waiting_time = 0
        # Stop + Green Light -> Abnormal Waiting Time
        self.delay_time = 0
        # Low speed -> Acceleration and deceleration waiting time
        self.lowSpee_time = 0
        # Sum of all above wait time
        self.wait_sum = 0
        

def main():
    # Start the connection between SUMO and TraCI
    traci.start(['sumo', '-c', 'map.sumo.cfg'])

    print("simulation start")
    
    # Initialize a dictionary to store the edge of each vehicle in the previous time step
    previous_edges = {}
    vehicles = {}
    # Initialize a dictionary to store vehicle information vehicle_id: edge_id: vehicle_information
    traffic_data = {}
    
    # Create a file and write the header before the first write
    csv_file = "TraCI_output_adjusted.csv"
    with open(csv_file, mode='w', newline='') as file:
        fieldnames = ['Vehicle_ID', 'Edge_ID', 'Time', 'Gap', 'V_Length', 'Width', 'Imperfection', 'Height', 'Capacity', 'Lanes_TraCI', 'Lanes_Net', 'Speed_TraCI', 'Speed_Net', 'E_Length', 'Turn', 'Driving_Num', 'Turn_Stat', 'Wait_Time', 'Travel_Time', 'straight', 'turn', 'left', 'right', 'partially_left', 'partially_right', 'invalid', 'end_road','Delay_Time','LowSpee_Time','Wait_Sum']
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()

        # Simulation start
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()  # 执行一个模拟步骤

            # 1. Get the IDs of all vehicles in the simulation
            vehicle_ids = traci.vehicle.getIDList()

            # Check and delete vehicle IDs that are no longer in the simulation to reduce memory pressure
            for vehicle_id in list(traffic_data.keys()):
                if vehicle_id not in vehicle_ids:
                    del traffic_data[vehicle_id]  # Remove vehicle IDs that are no longer active from the dictionary

            for v_id in vehicle_ids:

                # 2. Get the road segment the vehicle is currently on
                current_edge = traci.vehicle.getRoadID(v_id)

                # Check if the vehicle has entered a new edge
                if v_id not in previous_edges or previous_edges[v_id] != current_edge:

                    # Update the current edge of the vehicle
                    previous_edges[v_id] = current_edge

                    # 2. Get the type of vehicle
                    # vehicle_class = traci.vehicle.getVehicleClass(v_id)                # Generalized vehicle classification, such as passenger cars, buses
                    # vehicle_type = traci.vehicle.getTypeID(v_id)                       # More detailed vehicle type, such as brand of passenger car, etc.
                    vehicle_gap = traci.vehicle.getMinGap(v_id)                        # The distance between the vehicle and the vehicle ahead
                    vehicle_length = traci.vehicle.getLength(v_id)                     # Vehicle length
                    vehicle_width = traci.vehicle.getWidth(v_id)                       # Width of vehicle
                    vehicle_height = traci.vehicle.getHeight(v_id)                     # Vehicle height
                    vehicle_capacity = traci.vehicle.getPersonCapacity(v_id)           # Number of passengers in the vehicle
                    # vehicle_speed_mode = traci.vehicle.getSpeedMode(v_id)              # The vehicle's speed pattern, e.g. obeying speed limits (bits)
                    # vehicle_lanechange_mode = traci.vehicle.getLaneChangeMode(v_id)    # The vehicle's lane change mode, such as overtaking lane change (bit)
                    
                    # 3. Get the driver's driving habits

                    dri_imperfection = traci.vehicle.getImperfection(v_id)

                    # 4. Get the current time
                    time_seconds = traci.simulation.getTime()

                    # 5. Get the number of lanes on the road segment
                    lane_count_traci = traci.edge.getLaneNumber(current_edge)
                    if current_edge in edge_attrs:
                        lane_count_net = edge_attrs[current_edge]['lanes']
                    else:
                        print(f"Error. Edge {current_edge} cannot be found in dic.")

                    # 6-7. Get the length of the road segment + the speed limit of the road segment
                    allowed_speed = round(traci.vehicle.getAllowedSpeed(v_id))

                    if current_edge in edge_attrs:
                        edge_speed = round(edge_attrs[current_edge]['avg_speed'], 4)
                        edge_length = round(edge_attrs[current_edge]['avg_length'], 4)
                    else:
                        print(f"Error. Edge {current_edge} cannot be found in dic.")

                    # 8. Get the number of vehicles currently traveling on the road
                    driving_num = traci.edge.getLastStepVehicleNumber(current_edge)

                    # 9. Get the IDs of the cars currently traveling on the road
                    driving_IDs = traci.edge.getLastStepVehicleIDs(current_edge)

                    # 10 Get the current vehicle's steering direction
                    turn = get_next_edge_direction(v_id, current_edge, edge_conns)

                    # 11. Get the turning statistics of the car currently traveling on the road segment
                    turn_stat = []
                    for veh_id in driving_IDs:
                        turn_temp = get_next_edge_direction(veh_id, current_edge, edge_conns)
                        turn_stat.append(turn_temp)

                    vehicle_info = {
                        "Time":time_seconds, "Gap":vehicle_gap, "V_Length":vehicle_length,
                        "Width":vehicle_width, "Imperfection":dri_imperfection, "Height":vehicle_height, "Capacity":vehicle_capacity,
                        "Lanes_TraCI":lane_count_traci,"Lanes_Net":lane_count_net, "Speed_TraCI":allowed_speed, "Speed_Net":edge_speed, 
                        "E_Length":edge_length,"Turn":turn, "Driving_Num":driving_num, "Turn_Stat":turn_stat,
                        "Travel_Time":None,"Wait_Time":None,"Delay_Time":None,"LowSpee_Time":None,"Wait_Sum":None
                    }
                    update_vehicle_data(traffic_data, v_id, current_edge, vehicle_info)

                # 12. Get the time the vehicle waits for traffic lights and the time it takes to pass through

                if v_id not in vehicles:
                    vehicles[v_id] = Vehicle(v_id)

                # current_edge = traci.vehicle.getRoadID(v_id)
                speed = traci.vehicle.getSpeed(v_id)
                next_tls = traci.vehicle.getNextTLS(v_id)

                # When the vehicle enters the new edge
                if vehicles[v_id].current_edge != current_edge:
                    if vehicles[v_id].current_edge is not None:
                        # The vehicle leaves the current edge and outputs the result
                        route = traci.vehicle.getRoute(v_id)
                        current_edge_index = route.index(current_edge)

                        # Check if v_id exists in traffic_data
                        if v_id not in traffic_data:
                            print(f"Error: v_id {v_id} not found in traffic_data")
                        else:
                            previous_edge_index = current_edge_index - 1

                            # Loop until previous_edge is found or the index is out of bounds
                            while previous_edge_index >= 0 and route[previous_edge_index] not in traffic_data[v_id]:
                                previous_edge_index -= 1

                            if previous_edge_index < 0:
                                previous_edge_index = 0
                                # print("Error: Previous index out of range. Resetting to 0.")
                            else:
                                previous_edge = route[previous_edge_index]

                                if previous_edge in traffic_data[v_id]:
                                    traffic_data[v_id][previous_edge]['Wait_Time'] = vehicles[v_id].red_light_waiting_time
                                    traffic_data[v_id][previous_edge]['Travel_Time'] = vehicles[v_id].driving_time
                                    traffic_data[v_id][previous_edge]['Delay_Time'] = vehicles[v_id].delay_time
                                    traffic_data[v_id][previous_edge]['LowSpee_Time'] = vehicles[v_id].lowSpee_time
                                    traffic_data[v_id][previous_edge]['Wait_Sum'] = vehicles[v_id].wait_sum

                                # Create the data for the row to be written
                                row_data = {'Vehicle_ID': v_id, 'Edge_ID': previous_edge}
                                row_data.update(traffic_data[v_id][previous_edge])
                
                                # Transform Turn_Stat and add new columns
                                row_data = process_single_vehicle_data(row_data, 'Turn_Stat')

                                # Now row_data has been updated and can be written to CSV
                                writer.writerow(row_data)

                        vehicles[v_id].red_light_waiting_time = 0
                        vehicles[v_id].driving_time = 0
                        vehicles[v_id].delay_time = 0
                        vehicles[v_id].lowSpee_time = 0
                        vehicles[v_id].wait_sum = 0

                    vehicles[v_id].current_edge = current_edge
                    vehicles[v_id].edge_enter_time = traci.simulation.getTime()


                # Check if you are waiting at a red light
                if speed < 5 and next_tls:
                    vehicles[v_id].wait_sum += 1
                    if speed < 0.1 and (next_tls[0][3] == 'G' or next_tls[0][3] == 'g'):
                        vehicles[v_id].delay_time += 1
                    elif speed < 0.1 and (next_tls[0][3] == 'R' or next_tls[0][3] == 'r'):
                        vehicles[v_id].red_light_waiting_time += 1
                        # print("red wait")
                    else:
                        vehicles[v_id].is_waiting = True
                        vehicles[v_id].lowSpee_time += 1
                else:
                    if vehicles[v_id].is_waiting:
                        vehicles[v_id].is_waiting = False
                    vehicles[v_id].driving_time += 1


    # Close TraCI connection
    traci.close()
    print("simulation done")

if __name__ == '__main__':
    main()
