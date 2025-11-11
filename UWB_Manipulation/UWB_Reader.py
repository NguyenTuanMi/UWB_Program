# 8 JAN WORKS: main_udp.c sends the position of ALL nodes (aka tags) in a single message via UDP Port 5000. 

# TBD: Can combine/simplify get_target_position and get_all_positions? 
# TBD: Can establish just one single socket connection instead of always having to connect at every query?  

"""
README (updated 23 Jan) 

The functions in this code are meant to be imported in other codes
- get_target_position(target_id)
- get_all_positions()

However, for verification, this code can also be run by itself as main().

This code binds to the port and retrieves the necessary data.
- Without the .exe file running, can simulate UWB positions being sent via UDP by running the UWB_SendUDP.py script.
"""

import socket
import time
import pandas as pd
import numpy as np

"""
From example_copy6.c:
    node->id,       // ID
    node->role,      // Role
    node->pos_3d[0],
    node->pos_3d[1],
    node->pos_3d[2],
    node->dis_arr[0],
    node->dis_arr[1],
    node->dis_arr[2],
    node->dis_arr[3],
    node->dis_arr[4],
    node->dis_arr[5],
    node->dis_arr[6],
    node->dis_arr[7]
"""

# Define constants
PERIOD_S = 0.1
UWB_OFFSET: tuple[float] = (0, 0)       # describes the UWB's offset from actual (0,0) at bottom left

def parse_data_to_df(data):
    """
    Parse received UDP data into a pandas DataFrame.
    """
    try:
        # Split the received data into lines
        lines = data.decode().splitlines()
        
        # Create lists to store the data
        rows = []
        
        # Parse each line
        for line in lines:
            values = line.split(',')
            if len(values) == 13:  # Ensure we have all expected values
                row = {
                    'id': int(values[0]),
                    'role': int(values[1]),
                    'x': float(values[2]) + UWB_OFFSET[0],
                    'y': float(values[3]) + UWB_OFFSET[1],
                    'z': float(values[4]),
                    'dist1': float(values[5]),
                    'dist2': float(values[6]),
                    'dist3': float(values[7]),
                    'dist4': float(values[8]),
                    'dist5': float(values[9]),
                    'dist6': float(values[10]),
                    'dist7': float(values[11]),
                    'dist8': float(values[12])
                }
                rows.append(row)
        
        # Create DataFrame
        df = pd.DataFrame(rows)
        return df
    
    except Exception as e:
        print(f"Error parsing data to DataFrame: {e}")
        return None

def get_target_position(target_id, max_retries=3, timeout=0.1):
    """
    Get the position of a specific target ID via UDP communication.
    
    :param target_id: Static ID of the UWB Tag/Node set using NAssistant.
    :param max_retries: Maximum number of retries for finding the position.
    :param timeout: Timeout in seconds for socket operations.
    :return: 3D coordinates in cm. (Data Precision: 1cm; Stated 2D Accuracy: 10cm) or (0, 0, 0) if unsuccessful.
    """
    GOT_POS = False
    retry_count = 0

    # Define the UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow socket reuse

    server_address = ('0.0.0.0', 5000)  # Bind to all available interfaces
    sock.bind(server_address)  # Bind the socket to the address and port
    sock.settimeout(timeout)  # Set the timeout

    while not GOT_POS and retry_count < max_retries:
        try:
            # Attempt to receive data
            data, address = sock.recvfrom(4096)
            lines = data.decode().splitlines()

            # Process the received data
            for line in lines:
                parts = line.split(',')
                node_id = int(parts[0])
                if node_id == target_id:
                    GOT_POS = True
                    pos = (float(parts[2])+UWB_OFFSET[0] , float(parts[3])+UWB_OFFSET[1], float(parts[4]))
                    print(f"Target {target_id}: {pos}")
                    sock.close()
                    return pos

            # Increment retry count if no matching target_id found
            retry_count += 1

        except socket.timeout:
            # Increment retry count on timeout
            print(f"[WARNING] Timeout occurred. Retry {retry_count + 1} of {max_retries}.")
            retry_count += 1

        except Exception as e:
            # Handle other exceptions
            print(f"Error parsing data: {e}")
            retry_count += 1

    # Return default position if retries are exhausted
    print(f"[WARNING] Failed to get position for ID{target_id}. Returning pos = (0, 0, 0).")
    sock.close()
    return (0, 0, 0)


def get_all_positions(max_retries=3, timeout=0.2):
    """
    Get positions of all tags in one UDP receive operation.
    :param max_retries: Maximum number of retries for receiving data.
    :param timeout: Timeout in seconds for receiving data.
    :return: DataFrame with all tag positions and distances.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_address = ('0.0.0.0', 5000)
    sock.bind(server_address)
    sock.settimeout(timeout)  # Set the timeout

    retry_count = 0
    while retry_count < max_retries:
        try:
            data, address = sock.recvfrom(4096)
            df = parse_data_to_df(data)
            
            if df is not None and not df.empty:
                sock.close()
                return df
            
        except socket.timeout:
            # print(f"[WARNING] Timeout occurred. Retry {retry_count + 1} of {max_retries}.")
            retry_count += 1
        except Exception as e:
            print(f"Error receiving/processing data: {e}")
            retry_count += 1    
    
    print(f"[WARNING] Failed to get positions after {max_retries} retries.")
    sock.close()
    return pd.DataFrame()  # Return empty DataFrame if failed

# For verification, this code can also be run by itself.
if __name__ == "__main__":

    while True:

        ## Option 1: To Query Individual Tag Location in a (x,y,z) coordinate ##
        pos0 = get_target_position(target_id=0, max_retries=5)
        pos1 = get_target_position(target_id=1, max_retries=5)
        time.sleep(PERIOD_S)

        ## Option 2: To Query All Tags' Locations in a Dataframe ##
        df = get_all_positions()
        if not df.empty:
            print("\nCurrent positions:")
            print(df[['id', 'x', 'y', 'z']].to_string(index=False))
        time.sleep(PERIOD_S)