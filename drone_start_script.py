import subprocess as sp
import sys
import time as t
import requests as r
import os
import random as rand
from threading import Thread

testbed_dir = os.getenv("TESTBED_DIR")
service_count = 1

#This script is to lookup for wifi ssid and to connect to it if not the node is not connected.
#After wifi connected, the node run default route as 5G Gateway. One can change this behavior depending on the needs of the experiments.

def check_wifi_connectivity(hostname, gateway5G, ssid) :
    #define default gateway
    cmd_to_run = "ip route add default via %s"%gateway5G
    sp.getoutput(cmd_to_run)
    
    #cmd to force wifi connection when in simulation area
    connect_wifi_cmd = f"iw dev {hostname}-wlan0 connect {ssid}"

    wifi_link_cmd = f"iw dev {hostname}-wlan0 link"

    debut = t.perf_counter()

    while True:
        ecoule = t.perf_counter() - debut

        if ecoule > 1 :
            output = sp.getoutput(wifi_link_cmd)
            print("host connection status : \n")
            print(output)
            if output.find("Not connected.") == -1 :
                return True
            else :
                sp.getoutput(connect_wifi_cmd)
                debut = t.perf_counter()


if __name__=="__main__":
    hostname = sys.argv[1]

    gateway5G = sys.argv[2]
    
    ssid = sys.argv[3]

    while True :

        check_wifi_connectivity(hostname, gateway5G, ssid)

    

            
            
