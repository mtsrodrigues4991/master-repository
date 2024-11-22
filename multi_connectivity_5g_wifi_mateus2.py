#!/bin/python3
import sys
import re
import time
import logging
from sys import exit
from mininet.log import setLogLevel, info, error

from mininet.node import Controller, Node
from containernet.node import DockerSta
from containernet.cli import CLI
from mn_wifi.net import Mininet_wifi
from mininet.link import Intf
from mininet.util import quietRun
from containernet.term import makeTerm as makeTerm2
import random as rd
import os

logging.basicConfig(level=logging.DEBUG)
setLogLevel('info')  # set Mininet loglevel

class LinuxRouter(Node):
    def config(self, **params):
        super(LinuxRouter, self).config(**params)
        # Enable forwarding on the router
        self.cmd('sysctl net.ipv4.ip_forward=1')

    def terminate(self):
        self.cmd('sysctl net.ipv4.ip_forward=0')
        super(LinuxRouter, self).terminate()

def checkIntf(intf):
    "Make sure intf exists and is not configured."
    config = quietRun('ifconfig %s 2>/dev/null' % intf, shell=True)
    if not config:
        error('Error:', intf, 'does not exist!\n')
        exit(1)
    ips = re.findall(r'\d+\.\d+\.\d+\.\d+', config)
    if ips:
        error('Error:', intf, 'has an IP address,'
              'and is probably in use!\n')
        exit(1)

def create_topology(args):

    net = Mininet_wifi(controller=Controller)

    wifi_ssid = 'Five5G_Net'

    kwargs = {
        'ssid': wifi_ssid,
        'mode': 'n',
        'txpower': '1dBm',
        'range': 100,
        'failMode': 'standalone',
        'datapath': 'user'
    }

    defaultIP = '11.0.0.1/8'
    defaultExtIp = '15.0.0.2'
    gateway = '11.0.0.1'
    wDefaultIP = '192.168.0.1/24'
    wGateway = '192.168.0.1'

    router = net.addHost(
        "r0",
        cls=LinuxRouter,
        ip=defaultIP,
        defaultRoute='via %s' % defaultExtIp
    )
    wifi_router = net.addHost(
        "r1",
        cls=LinuxRouter,
        ip=wDefaultIP,
        defaultRoute='via %s' % defaultExtIp
    )

    info("*** Creating nodes: vehicles\n")
    # Adding drone as a station
    drones = []
    for id in range(0, 1):
        drone_id = 'drone%s' % (id + 1)
        mac = "00:00:00:12:0A:%02d" % (id + 1)
        drone = net.addStation(
            drone_id,
            defaultRoute='via %s' % wGateway,
            mac=mac
        )
        drones.append(drone)

    print("Access points loading complete.\n")

    info("Adding gnb container with ueransim image.\n")
    gnb = net.addStation(
        "gnb1",
        cls=DockerSta,
        position='200,400,0',
        volumes=['/home/wifi/mn-wifi-cnet-vimemu-install/ueransim:/ueransim'],
        devices=['/dev/net/tun:/dev/net/tun'],
        dimage="ueransim:latest",
        privileged=True,
        publish_all_ports=True
    )

    # Adding internal network switch s1 and external switch s2
    info("*** Adding switches for routing\n")

    s1 = net.addSwitch('s1')
    s2 = net.addSwitch('s2')
    s3 = net.addSwitch('s3')

    intfName = "p1net"

    # Adding link to gnb and set its IP address
    net.addLink(
        s1,
        gnb,
        intfName='gnb1-eth0',
        params2={'ip': '11.0.0.2/8'}
    )

    # Adding drones and set the first IP address from 11.0.0.10
    for id, drone in enumerate(drones):
        net.addLink(
            s1,
            drone,
            intfName='t%s-eth0' % (id),
            params2={'ip': '11.0.0.%s/8' % str(id + 10)}
        )

    # Configuring Propagation Model
    net.setPropagationModel(model="logDistance", exp=2.8)

    c0 = net.addController('c0')
    info("Adding controller ....\n")

    info("Controller info: \n")
    info(c0)

    net.configureWifiNodes()

    if '-p' not in args:
        net.plotGraph()

    info("**** Adding link between access points, switch and router\n")

    net.addLink(
        s1,
        router,
        intfName='r0-eth0',
        params2={'ip': defaultIP}
    )

    # Configure outside (host link) i.e., internet
    net.addLink(
        s2,
        router,
        intfName='r0-eth1',
        params2={'ip': '15.0.0.4/29'}
    )

    net.addLink(
        s2,
        wifi_router,
        intfName='r1-eth1',
        params2={'ip': '15.0.0.5/29'}
    )

    net.addLink(
        s3,
        wifi_router,
        intfName='r1-eth0',
        params2={'ip': wDefaultIP}
    )

    info("**** Starting network and connecting to traci\n")
    min_x = 0
    min_y = 0
    max_x = 1300
    max_y = 800
    net.setMobilityModel(
        time=10,
        model='RandomDirection',
        max_x=max_x,
        max_y=max_y,
        seed=20
    )

    net.build()

    info("*** Connecting to intf %s\n" % intfName)
    Intf(intfName, node=s2)

    c0.start()
    for ap in net.aps:
        ap.start([c0])

    for id, drone in enumerate(drones):
        drone.setIP(
            '192.168.0.{}/24'.format(id + 10),
            intf='{}'.format(drone.wintfs[0].name)
        )

    router.cmd("route add -net 12.0.0.0/24 gw 15.0.0.2")
    wifi_router.cmd("route add -net 11.0.0.0/8 gw 15.0.0.2")

    info("**** Launching GNB\n")

    dcmd = "bash -c './ueransim/run_gnb.sh'"
    makeTerm2(gnb, cmd=dcmd)
    makeTerm2(wifi_router, cmd="bash -c 'ifconfig'")

    info("**** Launching UE\n")
    # This loop over all drones and starts a python script "car_start_script.py"
    # Then runs the run_ue.sh script to start ueransim ue

    for id, drone in enumerate(drones):
        start_cmd = f"python3 car_start_script.py {drone.name} {gateway} {wifi_ssid} &"
        drone.cmd(start_cmd)
        cmd = f"bash -c './run_ue.sh {(id + 1)} {drone.name} - Emu5GNet'"
        makeTerm2(drone, cmd=cmd)

    net.start()

    info("*** Configuring %s interface\n" % intfName)
    config = quietRun('ifconfig %s up' % s2.name, shell=True)
    print(config)
    time.sleep(60)

    # Print the current working directory
    print("Current working directory:", os.getcwd())

    # Execute ping and save output to a file on the host
    info("*** Executing ping from drone1 to gnb1\n")
    drone1 = net.get('drone1')
    ping_output = drone1.cmd('ping -c 10 11.0.0.2')

    # Process the ping output to extract response times
    import re

    # Split the output into lines
    lines = ping_output.split('\n')

    # List to store response times
    response_times = []

    # Iterate over each line of the output
    for line in lines:
        # Ignore lines that do not contain 'time='
        if 'time=' in line:
            # Use regular expression to extract the time value
            match = re.search(r'time=([\d\.]+)\s*ms', line)
            if match:
                time_value = match.group(1)
                response_times.append(time_value)

    # Format the response times as a single string
    response_times_str = ', '.join(response_times)
    output_line = 'Response time: ' + response_times_str

    # Display the value on the terminal
    print(output_line)

    # Specify the absolute path to the file
    output_file_path = '/home/wifi/mn-wifi-cnet-vimemu-install/codes_example/drone1_latencia.txt'

    # Write the formatted output to the file
    try:
        with open(output_file_path, 'w') as f:
            f.write(output_line)
        print("Ping output written to drone1_latencia.txt")
    except Exception as e:
        print("An error occurred while writing the file:", e)

    # Start iperf3 as server on gnb1
    info("*** Starting iperf3 on gnb1 as server\n")
    gnb1 = net.get('gnb1')
    gnb1.cmd('iperf3 -s &')

    # Wait for the server to start
    time.sleep(2)

    # Start iperf3 as client on drone1 to measure bandwidth
    info("*** Measuring bandwidth from drone1 to gnb1\n")
    drone1 = net.get('drone1')
    iperf_output = drone1.cmd('iperf3 -c 11.0.0.2 -t 10')

    # Print the full iperf3 output
    print("Full iperf3 output:\n", iperf_output)

    # Process the iperf output to extract bandwidth
    lines = iperf_output.split('\n')
    bandwidth = None

    for line in lines:
        if 'sender' in line:
            # Use regular expression to extract bandwidth value and unit
            match = re.search(r'\s+([\d\.]+)\s+(\wbits/sec)', line)
            if match:
                bandwidth_value = match.group(1)
                bandwidth_unit = match.group(2)
                bandwidth = f"{bandwidth_value} {bandwidth_unit}"
            break

    if bandwidth:
        output_line = 'Bandwidth: ' + bandwidth
    else:
        output_line = 'Bandwidth not found in iperf output.'

    # Specify the absolute path to the file
    output_file_path = '/home/wifi/mn-wifi-cnet-vimemu-install/codes_example/drone1_iperf3.txt'

    # Write the formatted output to the file
    try:
        with open(output_file_path, 'w') as f:
            f.write(output_line)
        print("iperf3 output written to drone1_iperf3.txt")
    except Exception as e:
        print("An error occurred while writing the file:", e)

    #info("*** Simulation running for 60 seconds\n")
    #time.sleep(60)  # Simulation runs for 60 seconds

    info("*** Running CLI\n")
    CLI(net)
    # when the user types exit in the CLI, we stop the emulator

    net.stop()

if __name__ == '__main__':
    setLogLevel('info')
    create_topology(sys.argv)
