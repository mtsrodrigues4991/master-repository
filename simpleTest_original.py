#!/usr/bin/python

# Certifique-se de que o servidor está executando no CoppeliaSim
# Execute setNodePosition.py para atualizar a localização da estação
# Se houver um erro ao conectar ao socket, execute: sudo pkill -9 -f python

import sys
import time
import os

from mininet.log import info

try:
    import sim
except:
    info('--------------------------------------------------------------')
    info('"sim.py" could not be imported. This means very probably that')
    info('either "sim.py" or the remoteApi library could not be found.')
    info('Make sure both are in the same folder as this file,')
    info('or appropriately adjust the file "sim.py"')
    info('--------------------------------------------------------------')
    info('')


def drone_position(args):
    drones = [[] for i in range(3)]
    drones_names = ['Quadricopter_base', 'Quadricopter_base#0', 'Quadricopter_base#1']
    nodes = []
    data = [[] for i in range(3)]

    if len(args) > 1:
        for n in range(1, len(args)):
            nodes.append(args[n])
    else:
        info("No nodes defined")
        exit()

    info('Program started')
    sim.simxFinish(-1)  # Apenas por precaução, fecha todas as conexões abertas
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Conecta ao CoppeliaSim

    if clientID != -1:
        res = None
        info('Connected to remote API server')
        # Obtendo os IDs dos drones da simulação
        for i in range(0, len(drones)):
            [res, drones[i]] = sim.simxGetObjectHandle(clientID,
                                                       drones_names[i],
                                                       sim.simx_opmode_oneshot_wait)

        if res == sim.simx_return_ok:
            info('Connected with CoppeliaSim')
        else:
            info('Remote API function call returned with error code: ', res)

        time.sleep(2)
        # Inicia o streaming da função getPosition
        for i in range(0, len(drones)):
            sim.simxGetObjectPosition(clientID,
                                      drones[i],
                                      -1,
                                      sim.simx_opmode_streaming)

        while True:
            # Obtendo as posições como buffers
            for i in range(0, len(drones)):
                returnCode, data[i] = sim.simxGetObjectPosition(clientID,
                                                                drones[i],
                                                                -1,
                                                                sim.simx_opmode_buffer)
            # Armazenando a posição em arquivos de dados
            for i in range(0, len(data)):
                send_file(data[i], nodes[i])

            time.sleep(1)

        # Agora fecha a conexão com o CoppeliaSim:
        sim.simxFinish(clientID)
    else:
        info('Failed connecting to remote API server')
    info('Program ended')


def send_file(data, node):
    path = os.path.dirname(os.path.abspath(__file__))
    file_name = "{}/data/".format(path) + node + ".txt"
    with open(file_name, "a") as f:
        file_position = ','.join(map(str, data))  # Certifica que os dados estão separados por vírgulas
        f.write(file_position + '\n')  # Adiciona uma nova linha após cada posição


if __name__ == '__main__':
    drone_position(sys.argv)
