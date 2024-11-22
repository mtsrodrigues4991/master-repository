#!/bin/bash

# Parâmetros de entrada
GNB_ID=$1      # ID do gNB (1 ou 2)
GNB_IP=$2      # Endereço IP do gNB (exemplo: 11.0.0.2/8)
AMF_IP=$3      # Endereço IP do AMF (exemplo: 172.20.0.2)

# Exibir os parâmetros recebidos
echo "Iniciando gNB${GNB_ID} com IP ${GNB_IP} e AMF em ${AMF_IP}"

# Limpar configuração anterior da interface eth0
ifconfig eth0 0

# Atribuir o endereço IP à interface eth0
ip addr add ${GNB_IP} dev eth0

# Ativar a interface eth0
ip link set eth0 up

# Definir o endereço do gateway (ajuste se necessário)
GATEWAY_IP='11.0.0.1'

# Configurar rotas
ip route add ${AMF_IP} via ${GATEWAY_IP}
ip route add default via ${GATEWAY_IP}

# Exibir as rotas configuradas
echo "Rotas configuradas:"
ip route

# Opcional: verificar conectividade
echo "Verificando conectividade com o gateway e o AMF..."
ping -c 5 ${GATEWAY_IP}
ping -c 5 ${AMF_IP}

# Navegar para o diretório do UERANSIM
cd /ueransim/UERANSIM/

# Iniciar o gNB com o arquivo de configuração específico
echo "Iniciando o gNB com o arquivo de configuração config/gnb${GNB_ID}-config.yaml"
./build/nr-gnb -c "config/gnb${GNB_ID}-config.yaml"

# Exibir as interfaces de rede para verificação
ifconfig
