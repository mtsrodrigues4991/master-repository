#!/usr/bin/python

# Make sure to have the simpleTest running

import sys
import socket
import time
import os

from mininet.log import info


def client(message):
    host = '192.168.56.102'
    port = 12345  # Make sure it's within the > 1024 $$ <65535 range
    s = socket.socket()
    s.connect((host, port))
    s.send(str(message).encode('utf-8'))
    data = s.recv(1024).decode('utf-8')
    info('Received from server: ' + data)
    s.close()


def read_data(file, node):
    # Abre o arquivo e lê todas as linhas
    with open(file, 'r') as f:
        lines = f.readlines()  # Lê todas as linhas do arquivo

    if lines:  # Verifica se há linhas no arquivo
        # Pega a última linha e remove espaços em branco
        last_line = lines[-1].strip()  
        # Divide a linha pelos separadores de vírgula para obter os dados de posição
        data = last_line.split(',')
        
        if len(data) >= 3:  # Verifica se há pelo menos 3 valores (x, y, z)
            data_drone = "set." + node + ".setPosition(\"" \
                         + str(50 - float(data[0]) * 20) + "," \
                         + str(50 - float(data[1]) * 20) + "," \
                         + str(50 - float(data[2]) * 20) + "\")"
            client(data_drone)
            time.sleep(0.5)
        else:
            print(f"Dados de posição inválidos no arquivo {file}.")
    else:
        print(f"O arquivo {file} está vazio.")

if __name__ == '__main__':

    nodes = []
    files = []

    if len(sys.argv) > 1:
        path = os.path.dirname(os.path.abspath(__file__))
        for n in range(1, len(sys.argv)):
            nodes.append(sys.argv[n])
            pfile = '{}/data/'.format(path) + sys.argv[n] + '.txt'
            files.append(pfile)
    else:
        info("No nodes defined")
        exit()

    while True:
        i = 0
        for x in files:
            read_data(x, nodes[i])
            i += 1
