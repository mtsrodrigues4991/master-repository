#!/usr/bin/python3

# Certifique-se de que o simpleTest_mateus.py está sendo executado
# Este script lê a posição do drone a partir do arquivo de dados e atualiza a posição do nó no Mininet-WiFi

import sys
import socket
import time
import os

from mininet.log import info


def client(message):
    host = '192.168.56.102'  # IP do servidor de socket (localhost)
    port = 12345        # Porta do servidor de socket
    s = socket.socket()
    try:
        s.connect((host, port))
        s.send(str(message).encode('utf-8'))
        data = s.recv(1024).decode('utf-8')
        # info('Received from server: ' + data)
    except Exception as e:
        print(f"Erro ao conectar ao servidor de socket: {e}")
    finally:
        s.close()


def read_data(file, node):
    # Verifica se o arquivo existe
    time.sleep(10)
    if not os.path.exists(file):
        print(f"O arquivo {file} não existe.")
        return

    # Abre o arquivo e lê todas as linhas
    with open(file, 'r') as f:
        lines = f.readlines()

    if lines:
        # Pega a última linha e remove espaços em branco
        last_line = lines[-1].strip()
        # Divide a linha pelos separadores de vírgula para obter os dados de posição
        data = last_line.split(',')

        if len(data) >= 3:
            # Calcula a nova posição do nó
            x = float(data[0])
            y = float(data[1])
            z = float(data[2])

            # Ajuste de escala e posição, se necessário
            # Você pode ajustar esses valores conforme a necessidade
            x_pos = x * 10
            y_pos = y * 10
            z_pos = z * 10

            # Envia o comando para atualizar a posição do nó no Mininet-WiFi
            data_drone = f"set.{node}.setPosition('{x_pos},{y_pos},{z_pos}')"
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
            pfile = os.path.join(path, 'data', f"{sys.argv[n]}_position.txt")
            files.append(pfile)
    else:
        info("No nodes defined")
        sys.exit(1)

    while True:
        for i, x in enumerate(files):
            read_data(x, nodes[i])
        time.sleep(1)
