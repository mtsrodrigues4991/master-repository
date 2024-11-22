#!/bin/python3
import sys
import re
import logging
from sys import exit
from mininet.log import setLogLevel, info, error

from mininet.node import Controller, Node
from containernet.node import DockerSta
from containernet.cli import CLI
from mn_wifi.net import Mininet_wifi
from mininet.link import Intf
from mininet.util import quietRun
#from mininet.term import makeTerm
from containernet.term import makeTerm as makeTerm2
import random as rd

logging.basicConfig(level=logging.DEBUG)
setLogLevel('info')  # set Mininet loglevel

class LinuxRouter(Node):
    def config( self, **params ):
        super( LinuxRouter, self).config( **params )
        # Enable forwarding on the router
        self.cmd( 'sysctl net.ipv4.ip_forward=1' )

    def terminate( self ):
        self.cmd( 'sysctl net.ipv4.ip_forward=0' )
        super( LinuxRouter, self ).terminate()

def checkIntf( intf ):
    "Make sure intf exists and is not configured."
    config = quietRun( 'ifconfig %s 2>/dev/null' % intf, shell=True )
    if not config:
        error( 'Error:', intf, 'does not exist!\n' )
        exit( 1 )
    ips = re.findall( r'\d+\.\d+\.\d+\.\d+', config )
    if ips:
        error( 'Error:', intf, 'has an IP address,'
               'and is probably in use!\n' )
        exit( 1 )

def create_topology(args):

    net = Mininet_wifi(controller=Controller)

    wifi_ssid = 'Five5G_Wifi'

    kwargs = {'ssid':wifi_ssid, 'mode':'n','txpower':'1dBm','range': 100, 'failMode': 'standalone', 'datapath': 'user'}



    defaultIP = '11.0.0.1/8'
    defaultExtIp = '15.0.0.2'
    gateway = '11.0.0.1'
    wDefaultIP = '192.168.0.1/24'
    wGateway = '192.168.0.1'

    router = net.addHost("r0", cls=LinuxRouter, ip=defaultIP, defaultRoute='via %s' %defaultExtIp)
    wifi_router = net.addHost("r1", cls=LinuxRouter, ip=wDefaultIP, defaultRoute='via %s' %defaultExtIp)
    
    info("*** Creating nodes : vehicles\n")
    #Adding car with wifi configuration : ip : 12.0.0.0/24, gateway : 12.0.0.1 (wifi router)
    for id in range(0,10):
        car_id = 'car%s'%(id+1)
        mac = "00:00:00:12:0A:0%s"%str(car_id)
        min_x = rd.randint(40,600)
        max_x = rd.randint(600,1250)
        min_y = rd.randint(20,300)
        max_y = rd.randint(300,700)
        min_v = rd.randint(1,5)
        max_v = rd.randint(5,10)
        net.addCar(car_id, defaultRoute='via %s'%wGateway, min_x=min_x,mac=mac,
            min_y = min_y, min_v=min_v, max_x = max_x, max_y = max_y, max_v = max_v)

    # create access points
    info("Access points creation")
    ap_1 = net.addAccessPoint('ap_1', channel=6, position='70,40,0',
                              mac="00:00:00:11:00:01",**kwargs)
    ap_2 = net.addAccessPoint('ap_2', channel=11, position='70,234,0',
                              mac="00:00:00:11:00:02", **kwargs)
    ap_3 = net.addAccessPoint('ap_3', channel=1, position='70,456,0',
                              mac="00:00:00:11:00:03", **kwargs)
    ap_4 = net.addAccessPoint('ap_4', channel=13, position='250,40,0',
                              mac="00:00:00:11:00:04", **kwargs)
    ap_5 = net.addAccessPoint('ap_5', channel=6, position='250,234,0',
                              mac="00:00:00:11:00:05", **kwargs)
    ap_6 = net.addAccessPoint('ap_6', channel=11, position='250,456,0',
                              mac="00:00:00:11:00:06", **kwargs)
    ap_7 = net.addAccessPoint('ap_7', channel=13, position='430,40,0',
                              mac="00:00:00:11:00:07", **kwargs)
    ap_8 = net.addAccessPoint('ap_8', channel=13, position='430,40,0',
                              mac="00:00:00:11:00:18", **kwargs)
    ap_9 = net.addAccessPoint('ap_9', channel=1, position='430,456,0',
                              mac="00:00:00:11:00:08", **kwargs)
    ap_10 = net.addAccessPoint('ap_10', channel=6, position='610,40,0',
                              mac="00:00:00:11:00:09", **kwargs)
    ap_11 = net.addAccessPoint('ap_11', channel=11, position='610,234,0',
                              mac="00:00:00:11:00:10", **kwargs)
    ap_12 = net.addAccessPoint('ap_12', channel=1, position='610,456,0',
                              mac="00:00:00:11:00:11", **kwargs)
    ap_13 = net.addAccessPoint('ap_13', channel=13, position='800,40,0',
                              mac="00:00:00:11:00:12", **kwargs)
    ap_14 = net.addAccessPoint('ap_14', channel=6, position='800,234,0',
                              mac="00:00:00:11:00:13", **kwargs)
    ap_15 = net.addAccessPoint('ap_15', channel=11, position='800,456,0',
                              mac="00:00:00:11:00:14", **kwargs)
    ap_16 = net.addAccessPoint('ap_16', channel=1, position='980,40,0',
                              mac="00:00:00:11:00:15", **kwargs)
    ap_17 = net.addAccessPoint('ap_17', channel=13, position='980,234,0',
                              mac="00:00:00:11:00:16", **kwargs)
    ap_18 = net.addAccessPoint('ap_18', channel=6, position='980,456,0',
                              mac="00:00:00:11:00:17", **kwargs)

    print("Access points loading complete \n.")

    info("Adding gnb container with ueransim image.")
    gnb = net.addStation("gnb1", cls=DockerSta, volumes=['/home/wifi/mn-wifi-cnet-vimemu-install/ueransim:/ueransim'],
                    devices=['/dev/net/tun:/dev/net/tun'], dimage="ueransim:latest",
                    privileged=True, publish_all_ports=True)

    # iperfserver = net.addStation("is1",cls=DockerSta, volumes=["/home/wifi/db:/db"], 
    #                         dimage='iperfserver:latest',
    #                         publish_all_ports=True)

    #Adding internal network switch s1 and external switch s2
    info("*** Adding switches for routing")

    s1 = net.addSwitch('s1')

    s2 = net.addSwitch('s2')
    
    s3 = net.addSwitch('s3')

    intfName = "p1net"

    #adding link to gnb and set its ip address 
    net.addLink(s1, gnb, intfName = 'gnb1-eth0', params2={'ip':'11.0.0.2/8'})
    #net.addLink(s1, iperfserver, intfName="iperf-eth0", params2={'ip':'11.0.0.110/8'})

    #adding cars (node) and set the first ip address from 11.0.0.10. This is the 5G emulated link
    #other cars Ip address will be set by incrementing the previous one
    for id, car in enumerate(net.cars):   
        net.addLink(s1, car, intfName='t%s-eth0'%(id), params2={'ip':'11.0.0.%s/8'%str(id+10)})

    # info("*** Configuring Propagation Model\n")
    net.setPropagationModel(model="logDistance", exp=2.8)

    c0 = net.addController('c0')
    info("Adding controller ....")

    info("Controller info : \n")
    info(c0)

    net.configureWifiNodes()
    
    if '-p' not in args:
        net.plotGraph()
    
    info("**** Adding link between access points, swith and router")

    net.addLink(s1, router, intfName = 'r0-eth0',
            params2={ 'ip' : defaultIP })

    #configure outside (host link) i.e. internet
    net.addLink(s2, router, intfName = 'r0-eth1', params2={ 'ip' : '15.0.0.4/29'})

    net.addLink(s2, wifi_router, intfName = 'r1-eth1', params2={ 'ip' : '15.0.0.5/29'})

    net.addLink(s3, wifi_router, intfName = 'r1-eth0', params2={ 'ip' : wDefaultIP})

    net.addLink(s3, ap_1)
    net.addLink(ap_1, ap_2)

    net.addLink(ap_2, ap_3)
    net.addLink(ap_3, ap_4)
    net.addLink(ap_4, ap_5)
    net.addLink(ap_5, ap_6)
    net.addLink(ap_6, ap_7)
    net.addLink(ap_7, ap_8)

    net.addLink(ap_8, ap_9)
    net.addLink(ap_9, ap_10)

    net.addLink(ap_10, ap_11)
    net.addLink(ap_11, ap_12)
    net.addLink(ap_12, ap_13)

    net.addLink(ap_13, ap_14)

    net.addLink(ap_14, ap_15)
    net.addLink(ap_15, ap_16)
    net.addLink(ap_16, ap_17)
    net.addLink(ap_17, ap_18)

    info("**** Starting network and connecting to traci")
    min_x = 0
    min_y = 0
    max_x = 1300
    max_y = 800
    net.setMobilityModel(time=10, model='RandomDirection',
                        max_x=max_x, max_y=max_y, seed=20)

    net.build()

    info("*** Connecting to intf %s"%intfName)
    Intf( intfName, node=s2)

    c0.start()
    for ap in net.aps:
        ap.start([c0])

    for id, car in enumerate(net.cars):
        car.setIP('192.168.0.{}/24'.format(id+10),
                  intf='{}'.format(car.wintfs[0].name))

    

    router.cmd("route add -net 12.0.0.0/24 gw 15.0.0.2")
    wifi_router.cmd("route add -net 11.0.0.0/8 gw 15.0.0.2")

    #router.cmd("route add -net 0.0.0.0/0 gw 15.0.0.1")

    info("**** Launching GNB")

    dcmd = "bash -c './ueransim/run_gnb.sh'"
    makeTerm2(gnb,cmd=dcmd)
    makeTerm2(wifi_router, cmd="bash -c 'ifconfig'")

    info("**** Launching UE")
    #This loop over all car and start a python script "car_start_script.py"
    #car_start_py launch commands to check wifi connectivity and force it
    #Then run the run_ue.sh script to start ueransim ue

    for id, c in enumerate(net.cars):
        start_cmd = f"python3 car_start_script.py {c.name} {gateway} {wifi_ssid} &"
        c.cmd(start_cmd)
        cmd = f"bash -c './run_ue.sh {(id+1)} {c.name} - Emu5GNet'"
        makeTerm2(c, cmd=cmd)

    net.start()

    info("*** Configuring %s interface"%intfName)
    config = quietRun( 'ifconfig %s up' %s2.name, shell=True)
    print(config)


    info("*** Running CLI\n")
    CLI(net)
    # when the user types exit in the CLI, we stop the emulator
    net.stop()


if __name__ == '__main__':
    setLogLevel('info')
    create_topology(sys.argv)
