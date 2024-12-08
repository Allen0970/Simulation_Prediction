
import xml.etree.ElementTree as ET
from collections import defaultdict
import argparse
import random
import networkx as nx
from xml.dom import minidom
import re
import pickle

def initialization(xml_file):
    # 1.1 edges2features_dict: dictionary of sumo edge id mapping static info

    # Parsing XML files
    tree = ET.parse(xml_file)
    root = tree.getroot()
    # Initialize dictionary
    edges2features_dict = {}
    # Traverse all edge nodes
    for edge in root.findall('edge'):
        edge_id = edge.get('id')
        edge_from = edge.get('from')
        edge_to = edge.get('to')
        total_length = 0
        total_speed = 0
        lane_count = 0
        # Traverse all lane nodes under edge
        for lane in edge.findall('lane'):
            length = float(lane.get('length'))
            speed = float(lane.get('speed'))
        
            total_length += length
            total_speed += speed
            lane_count += 1
        if lane_count > 0:
            avg_length = total_length / lane_count
            avg_speed = total_speed / lane_count
            # Assume that each car occupies a lane with a distance of 5 meters
            capacity = int((avg_length * lane_count) / 5)
            edges2features_dict[edge_id] = {'from':edge_from, 'to':edge_from, 'average_length': avg_length, 'average_speed': avg_speed, 'lane': lane_count, 'capacity': capacity}
    # Print Results
    print(f"Number of edges: {len(edges2features_dict)}")

    # 1.2 connection_forward: dictionary mapping sumo edge id to connected sumo edge id

    tree = ET.parse(xml_file)
    root = tree.getroot()
    connection_forward = {}
    total = 0
    for connection in root.findall('connection'):
        from_attr = connection.get('from')
        to_attr = connection.get('to')
        # If there is no signal light id, assign None
        tl_attr = connection.get('tl', None)
        dir_attr = connection.get('dir')        
        # Avoid duplication of connections
        # Connections may be repeated because connections between different lanes on the same road (edge) appear in the text.net.xml file.
        # But we only focus on the edges
        if from_attr not in connection_forward:
            connection_forward[from_attr] = []
        exists = False
        for conn in connection_forward[from_attr]:
            if conn['to'] == to_attr:
                exists = True
                break
        if not exists:
            total += 1
            connection_forward[from_attr].append({'to': to_attr, 'tl': tl_attr, 'dir': dir_attr})
    print(f"Length of connection relationship is: {total}")

    # 1.3 edge_2_subedges_dict: a dictionary that maps the edge id of the index after removing the # to the sequentially ordered sumo edge ids
    
    tree = ET.parse(xml_file)
    root = tree.getroot()
    # An edge may be split into N sub-edges. For example, edge id A is split into A#0, A#1, A#2.
    # Create a dictionary, the key is the edge id, the value is N small dictionaries
    edge_dict_from_to_list = defaultdict(dict)
    edge_2_subedges_dict = defaultdict(dict)
    # Traverse each edge
    for edge in root.findall('edge'):
        edge_id = edge.get('id')
        from_node = edge.get('from')
        to_node = edge.get('to')
        # Check whether the edge id contains #. If it does, split the value before # as the edge id and the part after # as the sub-edge index
        if '#' in edge_id:
            edge_id_parts = edge_id.split('#')
            edge_id = edge_id_parts[0]
            edge_index = int(edge_id_parts[1])
        else:
            edge_index = 0
        # Update edge_dict_from_to_list dictionary
        edge_dict_from_to_list[edge_id][edge_index] = [from_node, to_node]
    # Verify that the edge's node-pairs are connected in index order
    for edge_id, sub_edges in edge_dict_from_to_list.items():
        index_len = len(sub_edges)
        if len(sub_edges) > 1:
            first_sub_edge = sub_edges[0]
            first_from = first_sub_edge[0]
            first_to = first_sub_edge[1]

            second_sub_edge = sub_edges[1]
            second_from = second_sub_edge[0]
            second_to = second_sub_edge[1]
            
            last_sub_edge = sub_edges[index_len - 1]
            last_from = last_sub_edge[0]
            last_to = last_sub_edge[1]
            # If the connection is in positive sequence
            if (first_to == second_from):
                for i in range(0, index_len - 1):
                    if (sub_edges[i][1] != sub_edges[i+1][0]):
                        print(f"Connection Error. Edge ID: {edge_id}, index {i} and {i+1}, Node Pairs: {sub_edges[i]} and {sub_edges[i+1]}.")
                sub_edges_temp = []
                for i in range(0, len(sub_edges)):
                    sub_edges_temp.append(edge_id + '#' + str(i))
                edge_2_subedges_dict[edge_id] = sub_edges_temp 
            # If you connect flashback
            if (first_from == second_to):
                for i in range(1, index_len):
                    if (sub_edges[i-1][0] != sub_edges[i][1]):
                        print(f"Connection Error. Edge ID: {edge_id}, index {i} and {i+1}, Node Pairs: {sub_edges[i]} and {sub_edges[i+1]}.")
                sub_edges_temp = []
                for i in range(len(sub_edges) - 1, -1, -1):
                    sub_edges_temp.append(edge_id + '#' + str(i))
                edge_2_subedges_dict[edge_id] = sub_edges_temp
        else:
            edge_2_subedges_dict[edge_id] = [edge_id]
    
    # 1.4 keep_subEdge_list: stores folk SUMO edge ids that have connections other than the first and last SUMO edges

    # Count how many edges have other connections in sub-edges other than the first and last ones
    # For example, A#0, A#1, A#2, A#1 and other roads are connected
    total_fork_edge = 0
    total_fork_edge_u = 0
    keep_subEdge_list = []
    for edge_id, values in edge_2_subedges_dict.items():
        sub_edge_len = len(values)
        # Only check if there are other connections between non-first and last sub-edges
        if (sub_edge_len > 2):
            for i in range(1, sub_edge_len - 1):
                sub_edge = values[i]
                sub_edge_id_part = sub_edge.split('#')[0]
                # Find the connection to from the dictionary of the connection relation
                for to_edges in connection_forward[sub_edge]:
                    to_edge = to_edges['to']
                    sub_to_edge_id_part = to_edge.split('#')[0]
                    if (sub_edge_id_part != sub_to_edge_id_part):
                        # Determine whether it is a U-turn on a small road section
                        if (sub_edge_id_part.lstrip('-') == sub_to_edge_id_part.lstrip('-')):
                            total_fork_edge_u += 1
                            # print(f"sub_edge {sub_edge} has fork connection with to_sub_edge {to_edge}")
                        else:
                            total_fork_edge += 1
                            keep_subEdge_list.append(sub_edge)
    
    # 1.5 Update edge_2_subedges_dict: Value is one or more sets of mergeable subEdges in order
    # First process the subEdge in edge_2_subedges_dict
    def split_list(subEdge_list, keep_subEdge_list):
        keep_set = set(keep_subEdge_list)
        result = []
        temp = []
        for item in subEdge_list:
            temp.append(item)
            if item in keep_set:
                result.append(temp)
                temp = []
        if temp:
            result.append(temp)
        return result

    for edgeID, subEdge_list in edge_2_subedges_dict.items():
        # if len(subEdge_list) > 2:
        edge_2_subedges_dict[edgeID] = split_list(subEdge_list, keep_subEdge_list)
            # Determine whether the value in the set subEdge_list appears in keep_subEdge_list, and cut all values ​​in subEdge_list according to the position of appearance.
            # For example, subEdge_list = [1,2,3,4,5,6,7,8], keep_subEdge_list = [3,7]
            # The output is [[1,2,3],[4,5,6,7][8]]

    return edges2features_dict, connection_forward, edge_2_subedges_dict

# 2. Constructing the mapped road network structure

def project(edges2features_dict, connection_forward, edge_2_subedges_dict):

    # 2.1 edge_dict: a dictionary that maps the projected edge id to its start and end points, and static info

    # edge_dict dictionary:
    # The key is the merged edge id. The edge id is before _, and the index of the merged edge id is after it in order.
    # Value is from subEdge,to subEdge,average speed,total length,capacity

    # Create a dictionary whose key is the edge id and whose value is a dictionary containing from, to, total speed limit, total length, number of lanes and current sub-edge index
    edge_dict = defaultdict(lambda: {'from_subEdge': [], 'to_subEdge': [], 'average_speed': 0, 'total_length': 0, 'capacity':0})
    for edgeID, subEdge_list in edge_2_subedges_dict.items():
        # In subEdge, does it start from 0 or max? For example, A#0, A#1 or A#1, A#0
        for split_list in subEdge_list:
            if len(split_list) == 1:
                subEdgeID = split_list[0]
                if '#' in split_list[0]:
                    total_speed = 0
                    total_length = 0
                    total_capacity = 0
                
                    proEdgeID = edgeID + '_' + subEdgeID.split('#')[1] + '.'
                    total_speed += edges2features_dict[subEdgeID]['average_speed']
                    total_length += edges2features_dict[subEdgeID]['average_length']
                    total_capacity += edges2features_dict[subEdgeID]['capacity']
                    
                    edge_dict[proEdgeID]['from_subEdge'] = subEdgeID
                    edge_dict[proEdgeID]['to_subEdge'] = subEdgeID
                    edge_dict[proEdgeID]['average_speed'] = total_speed
                    edge_dict[proEdgeID]['total_length'] = total_length
                    edge_dict[proEdgeID]['capacity'] = total_capacity
                else:
                    edge_dict[subEdgeID]['from_subEdge'] = subEdgeID
                    edge_dict[subEdgeID]['to_subEdge'] = subEdgeID
                    edge_dict[subEdgeID]['average_speed'] = edges2features_dict[subEdgeID]['average_speed']
                    edge_dict[subEdgeID]['total_length'] = edges2features_dict[subEdgeID]['average_length']
                    edge_dict[subEdgeID]['capacity'] = edges2features_dict[subEdgeID]['capacity']
            elif len(split_list) == 2:
                total_speed = 0
                total_length = 0
                total_capacity = 0
            
                proEdgeID = edgeID + '_'
                for subEdgeID in split_list:
                    proEdgeID += subEdgeID.split('#')[1] + '.'
                    total_speed += edges2features_dict[subEdgeID]['average_speed']
                    total_length += edges2features_dict[subEdgeID]['average_length']
                    total_capacity += edges2features_dict[subEdgeID]['capacity']

                subEdgeID = proEdgeID
                edge_dict[subEdgeID]['from_subEdge'] = split_list[0]
                edge_dict[subEdgeID]['to_subEdge'] = split_list[1]
                edge_dict[subEdgeID]['average_speed'] = total_speed / 2
                edge_dict[subEdgeID]['total_length'] = total_length
                edge_dict[subEdgeID]['capacity'] = total_capacity
            else:
                total_speed = 0
                total_length = 0
                total_capacity = 0
            
                proEdgeID = edgeID + '_'
                for subEdgeID in split_list:
                    proEdgeID += subEdgeID.split('#')[1]  + '.'
                    total_speed += edges2features_dict[subEdgeID]['average_speed']
                    total_length += edges2features_dict[subEdgeID]['average_length']
                    total_capacity += edges2features_dict[subEdgeID]['capacity']

                subEdgeID = proEdgeID
                edge_dict[subEdgeID]['from_subEdge'] = split_list[0]
                edge_dict[subEdgeID]['to_subEdge'] = split_list[-1]
                edge_dict[subEdgeID]['average_speed'] = total_speed / len(split_list)
                edge_dict[subEdgeID]['total_length'] = total_length
                edge_dict[subEdgeID]['capacity'] = total_capacity

    print(f"Nubmer of projected egde id is: {len(edge_dict)}")

    # 2.2 project_connection_list: a dictionary mapping projected edge ids to connected projected edge ids

    project_connection_list = []

    for edgeID_1, features_1 in edge_dict.items():
        from_subEdge_1 = features_1['from_subEdge']
        to_subEdge_1 = features_1['to_subEdge']
        
        for edgeID_2, features_2 in edge_dict.items():
            from_subEdge_2 = features_2['from_subEdge']
            to_subEdge_2 = features_2['to_subEdge']
            
            if to_subEdge_1 in connection_forward:
                for to_list in connection_forward[to_subEdge_1]:
                    if (from_subEdge_2 == to_list['to']):
                        if (edgeID_1 != edgeID_2):
                            project_connection_list.append([edgeID_1, edgeID_2])

    print(f"Nubmer of projected egde id is: {len(project_connection_list)}")

    return edge_dict, project_connection_list

# 3. Constructing the mapped graph structure

# 3.1 Build a graph structure based on projected edge id (nodes) (the connection relationship in project_connection_list is edge)

def construct_graph(edge_dict, project_connection_list):
    # Creating a directed graph
    G = nx.DiGraph()
    
    # Add edges and corresponding attributes to the graph
    for from_to_edgeID in project_connection_list:
        fromEdge = from_to_edgeID[0]
        toEdge = from_to_edgeID[1]
        total_length = edge_dict[fromEdge]['total_length'] + edge_dict[toEdge]['total_length']
        G.add_edge(fromEdge, toEdge, total_length = total_length)

    # Manually adjust the problematic connection relationships of some signal lights
    removeList = ['806169764_0.1.2.3.', '660671382_7.', '-1080862797', '-944771877', '1080862797', '944771877',
                '226041007_0.1.2.3.4.5.', '-5670088_5.4.3.', '46613634_4.', '196117062', '-196117062', '275298541_3.',
                '528748997_6.','305323026', '-305323026','743358444_0.1.2.3.4.', '226041011_0.1.2.3.', '846358874_3.',
                '1051841853_0.1.2.3.4.', '5672487_3.', '1115153280', '497165754', '-497165754',
                '169267544', '1136460177_0.1.2.', '972926362_0.1.2.', '420882495_0.1.', '-420882495_1.0.',
                '5670574_2.', '446864089', '46694760_15.', '5671444_4.', '196116997', '104144064', '734864200_0.1.2.',
                '1189567314', '204082744_0.1.', '1080862165', '623836435_0.1.2.', '980853075_3.', '511924978_0.1.2.3.',
                '1080324903', '1051266491', '46577955', '5673305_6.', '5672941_6.', '353220074_3.', '5671222_6.',
                '658583344_12.', '5672411_21.', '497264365', '569266484_10.', '305323029_0.1.', '847952488_0.1.2.',
                '420895463_0.1.', '620748646', '1123880569', '-734640621', '-104144064', '46334580_3.', '46694757_15.',
                '46694762_15.', '46522160_9.', '1022822153_0.1.2.', '968393479_16.', '945384088_6.', '198595461_10.',
                '5671450_11.12.13.', '440122907', '1043350177_0.1.2.', '686125781_6.', '543717203_0.1.', '226007071_0.1.2.3.',
                '5671623_6.', '5671357_7.', '5670797_6.', '5670707_6.', '34080174_5.6.', '970750137_0.1.', '5669008_9.',
                '952703047']
    
    for i in removeList:
        G.remove_node(i)
        
    num_nodes = G.number_of_nodes()
    num_edges = G.number_of_edges()

    print(f"Projected graph has {num_nodes} nodes")
    print(f"Projected graph has {num_edges} edges")

    return G

# 4. Generate the shortest path that meets the conditions


def generate_path(file_path, hour_periods, max_od_number):
 
    # 4.1 Read the dictionary of pre-tested trips, where the key is the trip id and the value is the shortest_path, which contains a set of edge ids, length, and depart_time.
    # Read a dictionary from a local file
    with open('trips.pkl', 'rb') as file:
        trips = pickle.load(file)

    # 4.2 Define the number of cars generated in each time slice, and 24 time slices

    # Compute the reciprocal of each element in the list
    reciprocals = [round(1/x, 4) for x in hour_periods]
    maxRate = max(reciprocals)
    rate = [(x / maxRate) for x in reciprocals]
    # Generate a list with 24 elements, each starting from 0 and increasing by 3600
    # time_intervals = [i * 3600 for i in range(25)]
    time_intervals = [i * 3600 for i in range(len(hour_periods))]

    # 4.3 Randomly select the mapped path

    # Defines the number of od-pairs generated
    trip_id = 0
    tripsAll = {}

    for i in range(len(rate)):

        # Calculate the maximum traffic flow
        odNum = int(max_od_number * rate[i])
        print(f"Generated od-pairs number in time slice {i} is: {odNum}")
        
        # In the dictionary trips, the key is the trip id and the value is shortest_path which contains a set of edge ids, length, and depart_time.
        # Randomly select a specified number of elements odNum from the dictionary
        trips_sample = random.sample(list(trips.items()), odNum)
        
        trips_current = {}
        for trip in trips_sample:
            trip_key, trip_value = trip
            
            # Update the corresponding depart_time and trip id
            new_depart_time = int(trip_value['depart_time']) + time_intervals[i]
            new_trip_id = trip_id
            trip_id += 1
            
            # Construct a new trip_value
            new_trip_value = {
                'shortest_path': trip_value['shortest_path'],
                'length': trip_value['length'],
                'depart_time': new_depart_time
            }
            
            # Add the new trip_id and corresponding value to trips_current
            trips_current[new_trip_id] = new_trip_value
        
        # Put it in a collection
        tripsAll.update(trips_current)
    
    return tripsAll

# 5. Convert route data into an XML file consisting of SUMO edge ids that can be read by SUMO

def back_2_sumo_routes(trips):

    # 5.1 Convert route data to SUMO edge id format readable by SUMO
    trips_sumo_version_dict = {}
    for trip_id, values in trips.items():
        path = values['shortest_path']
        departure_time = values['depart_time']
        path_sumo_version = []
        for edgeID in path:
            if '_' in edgeID:
                edgeID_main = edgeID.split('_')[0]
                edgeID_index = edgeID.split('_')[1]
                
                # Use regular expression to match parts of a number separated by periods
                number_strings = re.findall(r'\d+', edgeID_index)
                # Convert each matched numeric string into an integer and store it in a list
                number_list = [int(num) for num in number_strings]
                
                for i in number_list:
                    path_sumo_version.append(edgeID_main + '#' + str(i))
            else:
                path_sumo_version.append(edgeID)
                
        trips_sumo_version_dict[trip_id] = {'shortest_path': path_sumo_version, 'depart_time':departure_time}

    # 5.2 Verify the correctness of the connection relationship of trips_sumo_version_dict under sumo

    for routeID, subEdges in trips_sumo_version_dict.items():
        subEdges = subEdges['shortest_path']
        for i in range(0, len(subEdges) - 1):
            currentEdge = subEdges[i]
            nextEdge = subEdges[i + 1]
            valid = True
            if currentEdge in connection_forward:
                for to_list in connection_forward[currentEdge]:
                    if nextEdge == to_list['to']:
                        valid = False
            if valid == True:
                print(f"Error. {currentEdge} and {nextEdge} do not have connection.")
    
    # 5.3 Generate sumo readable xml file

    def convert_to_xml(trips_sumo_version_dict, filename="reRoute.trips.xml"):
        # Create the root element of XML
        routes = ET.Element("routes", attrib={
            "xmlns:xsi": "http://www.w3.org/2001/XMLSchema-instance",
            "xsi:noNamespaceSchemaLocation": "http://sumo.dlr.de/xsd/routes_file.xsd"
        })
        
        # Sort all vehicles by depart_time
        sorted_trips = sorted(trips_sumo_version_dict.items(), key=lambda item: item[1]['depart_time'])
        
        # Traverse the sorted vehicle information and add the vehicle element
        for vehicle_id, trip_info in sorted_trips:
            vehicle = ET.SubElement(routes, "vehicle", attrib={
                "id": str(vehicle_id),
                "depart": f"{trip_info['depart_time']:.2f}"
            })
            route = ET.SubElement(vehicle, "route", attrib={
                "edges": " ".join(trip_info["shortest_path"])
            })
        
        # Convert an ElementTree object to a string and format it
        rough_string = ET.tostring(routes, encoding="utf-8", method="xml")
        reparsed = minidom.parseString(rough_string)
        pretty_xml_as_string = reparsed.toprettyxml(indent="    ")
        
        # Save a formatted XML string to a file
        with open(filename, "w", encoding="utf-8") as f:
            f.write(pretty_xml_as_string)

    # Call the function and save the XML to a file
    convert_to_xml(trips_sumo_version_dict)

    print("XML file is saved in reRoute.trips.xml")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Count edges and nodes in a SUMO net XML file.")
    parser.add_argument('--input', type=str, required=True, help='Input XML file')
    
    args = parser.parse_args()

    edges2features_dict, connection_forward, edge_2_subedges_dict = initialization(args.input)
    
    edge_dict, project_connection_list = project(edges2features_dict, connection_forward, edge_2_subedges_dict)
    G = construct_graph(edge_dict, project_connection_list)

    odDepart_file = 'trips.pkl'
    hour_periods = [0.0823, 0.2023, 0.2395, 0.2104, 0.1447, 0.0755, 0.0456, 0.0406, 0.0421, 0.0408, 0.0384, 0.0362, 0.035, 0.0326, 0.0313, 0.0306, 0.0305, 0.0326, 0.0373, 0.0453, 0.0557, 0.0668, 0.0823, 0.1047]
    max_od_number = 13000
    tripsAll = generate_path(odDepart_file, hour_periods, max_od_number)
    back_2_sumo_routes(tripsAll)

