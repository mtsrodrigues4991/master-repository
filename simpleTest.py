#!/usr/bin/python3

# Certifique-se de que o servidor está executando no CoppeliaSim
# Execute setNodePosition.py para atualizar a localização da estação
# Se houver um erro ao conectar ao socket, execute: sudo pkill -9 -f python

import sys
import time
import os
import socket
import re

from mininet.log import info

try:
    import sim
except ImportError:
    info('--------------------------------------------------------------')
    info('"sim.py" could not be imported. This means very probably that')
    info('either "sim.py" or the remoteApi library could not be found.')
    info('Make sure both are in the same folder as this file,')
    info('or appropriately adjust the file "sim.py"')
    info('--------------------------------------------------------------')
    sys.exit(1)


def send_ping_command():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        try:
            sock.connect(('192.168.56.102', 12345))  # Conecte-se ao servidor de socket local
            # Envio do comando ping
            command = f"ping 10.0.0.2"
            print(f"Sending command: {command}")
            sock.sendall(command.encode('utf-8'))
            full_response = sock.recv(4096).decode('utf-8', errors='ignore')
            print("Full response received:", full_response)

            # Extrai todos os tempos de resposta usando expressões regulares
            response_times = re.findall(r'time=(\d+\.\d+) ms', full_response)
            if response_times:
                # Juntando todos os tempos de resposta em uma string formatada
                response_times_str = ", ".join(response_times)
                print("Response times:", response_times_str)
                return response_times_str
            else:
                print("No ping response time found.")
                return "No response time found."

        except ConnectionRefusedError:
            print("Failed to connect to the server. Is the Mininet-WiFi script running?")
            return "Ping Failed"


def drone_position(args):
    drones = []
    drones_names = []
    nodes = []
    data = []
    ping_count = 0
    max_pings = 1
    pings_done = False
    control = True

    if len(args) > 1:
        nodes = args[1:]
    else:
        info("No nodes defined")
        sys.exit(1)

    # Configurações iniciais
    for node in nodes:
        drones.append(None)
        drones_names.append('Quadricopter_base')
        data.append([])

    info('Program started\n')
    sim.simxFinish(-1)  # Fecha todas as conexões abertas
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Conecta ao CoppeliaSim

    if clientID != -1:
        info('Connected to remote API server\n')
        # Obtendo os handles dos drones
        for i in range(len(drones)):
            res, drones[i] = sim.simxGetObjectHandle(clientID, drones_names[i], sim.simx_opmode_oneshot_wait)
            if res != sim.simx_return_ok:
                print(f"Erro ao obter o handle do objeto {drones_names[i]}, código de retorno: {res}")
                sys.exit(1)

        time.sleep(2)

        # Inicia o streaming da posição dos drones
        for i in range(len(drones)):
            sim.simxGetObjectPosition(clientID, drones[i], -1, sim.simx_opmode_streaming)

        while control:
            # Obtendo as posições
            for i in range(len(drones)):
                returnCode, data[i] = sim.simxGetObjectPosition(clientID, drones[i], -1, sim.simx_opmode_buffer)
                if returnCode == sim.simx_return_ok:
                    # Armazenando a posição em arquivos de dados
                    send_file_position(data[i], nodes[i] + '_position')
                else:
                    print(f"Erro ao obter a posição do drone {nodes[i]}, código de retorno: {returnCode}")

            # Executa o ping apenas uma vez
            if not pings_done:
                if ping_count < max_pings:
                    print("*** Executando o ping")
                    latency_response = send_ping_command()
                    send_file_latency(latency_response, nodes[0] + '_latency')
                    ping_count += 1
                if ping_count >= max_pings:
                    pings_done = True

            time.sleep(1)
            if pings_done:
                control = False  # Encerra o loop após o ping ser realizado

        # Fecha a conexão com o CoppeliaSim
        sim.simxFinish(clientID)
    else:
        info('Failed connecting to remote API server\n')
    info('Program ended\n')


def send_file_position(data, node):
    path = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.join(path, 'data')
    if not os.path.exists(data_dir):
        os.makedirs(data_dir)
        print(f"O diretório {data_dir} foi criado.")

    file_name = os.path.join(data_dir, f"{node}.txt")
    with open(file_name, "a") as f:
        file_position = ','.join(map(str, data))  # Certifica-se de que os dados estão separados por vírgulas
        f.write(file_position + '\n')  # Adiciona uma nova linha após cada posição


def send_file_latency(data, node):
    path = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.join(path, 'data')
    if not os.path.exists(data_dir):
        os.makedirs(data_dir)
        print(f"O diretório {data_dir} foi criado.")

    file_name = os.path.join(data_dir, f"{node}.txt")
    with open(file_name, "a") as f:
        f.write(data + '\n')  # Escreve os tempos de latência no arquivo


if __name__ == '__main__':
    drone_position(sys.argv)
