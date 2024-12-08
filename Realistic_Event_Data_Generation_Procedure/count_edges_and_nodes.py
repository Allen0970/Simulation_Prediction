import xml.etree.ElementTree as ET
import argparse

def count_edges_and_nodes(xml_file):
    tree = ET.parse(xml_file)
    root = tree.getroot()

    edges = root.findall('edge')
    connections = root.findall('connection')

    edge_count = len(edges)
    connections_count = len(connections)

    return edge_count, connections_count

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Count edges and nodes in a SUMO net XML file.")
    parser.add_argument('--input', type=str, required=True, help='Input XML file')
    
    args = parser.parse_args()

    edge_count, connections_count = count_edges_and_nodes(args.input)
    print(f"Number of edges: {edge_count}")
    print(f"Number of connections: {connections_count}")
