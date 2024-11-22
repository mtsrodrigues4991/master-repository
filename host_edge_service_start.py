import subprocess as sp
import sys
import time as t
import requests as r
import os
import random as rand
from threading import Thread

testbed_dir = os.getenv("TESTBED_DIR")
service_count = 1

def check_wifi_connectivity(hostname):
    cmd_to_run = f"iw dev {hostname}-wlan0 link"

    debut = t.perf_counter()

    while True:
        ecoule = t.perf_counter() - debut

        if ecoule > 1 :
            output = sp.getoutput(cmd_to_run)
            print("host connection status : \n")
            print(output)
            if output.find("Not connected.") == -1 :
                return True
            else :
                debut = t.perf_counter()

def get_last_position(pos_file):
        pos = list()
        try:
            with open(pos_file, "r") as f:
                contains = f.readlines()
                # lines = contains.splitlines()

                last_line = contains[-1]
                positions = last_line.split(",")
                pos_x = float(positions[0])
                pos_y = float(positions[1])
                pos_z = 0
                pos.append(pos_x)
                pos.append(pos_y)
                pos.append(pos_z)

        except:
            pos = [-1,-1,-1]

        return pos

def is_in_simulation(node, x_min, x_max, y_min, y_max):
    """Get the distance between two nodes, read nodes positions from telemetry file
    :param self:
    :param src: source destion
    :param dst: destination node"""
    node_pos_file = testbed_dir+"/position-%s-mn-telemetry.txt"%node
    
    """ Read the last position of both nodes from telemetry position data"""
    node_pos = get_last_position(node_pos_file)
    print("Node position : ")
    print(node_pos)
    
    result = False
    if x_min <= int(node_pos[0]) <= x_max and y_min <= int(node_pos[1]) <= y_max :
        result = True
    else:
        result = False
    
    print("Is in simulation area ? : " +str(result))
    return result

def start_service(hostname, cpu_share, memory_limit, bw, priority, eo, latence_data_dir):
    global service_count

    vnf_name = hostname+str(service_count)+"s"
    volume = latence_data_dir+":/compute/data"
    d = {'vnf_name' : vnf_name,
            'image' : 'computing',
            'volume' : volume,
            'cpu_shares' : cpu_share,
            'memory_limit' : memory_limit,
            'bw': bw,
            'priority': priority,
            'node' : hostname       
        }

    queryString = f"http://{eo}/service/api/selfish/start"
    response = r.post(queryString, json=d)
    print(response.json())
    service_count +=1

if __name__=="__main__":
    
    hostname = sys.argv[1]
    cpu_share = sys.argv[2]
    memory_limit = sys.argv[3]
    bw = sys.argv[4]
    priority = sys.argv[5]
    eo = sys.argv[6]
    x_min = int(sys.argv[7])
    x_max = int(sys.argv[8])
    y_min = int(sys.argv[9])
    y_max = int(sys.argv[10])
    run_count = int(sys.argv[11])
    latence_data_dir = sys.argv[12]
    #run_count = 1
    first_run = True
    # randsleeptime = rand.randint(0,1)
    # .sleep(randsleeptime)

    while True :
        if is_in_simulation(hostname,x_min=x_min,x_max=x_max,y_min=y_min,y_max=y_max):
            new_launch = rand.randint(60,70)
            print(f"Total run count : {run_count} \n")
            while True :
                args = (hostname,cpu_share,memory_limit,bw,priority,eo, latence_data_dir)
                
                if run_count > 0:
                    
                    if first_run == True :
                        print(f"First run : {first_run}")
                        thread = Thread(target=start_service,args=args)
                        thread.start()
                        run_count -= 1                        
                        first_run = False                    
                    else:
                        print(f"New launch in {new_launch} seconds : ")
                        t_start = t.perf_counter()
                        run_count -= 1
                        while t.perf_counter() - t_start < new_launch :
                            pass
                                
                        thread = Thread(target=start_service,args=args)
                        thread.start()                            
                    print(f"Remaining run count : {run_count}")
                elif run_count == 0 :
                    print("Run count finished ! Exiting")
                    break
            break

    print("**** Node "+hostname+" is exiting. Bye****")
    exit(0)
            
            