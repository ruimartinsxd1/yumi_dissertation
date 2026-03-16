#!/usr/bin/env python3
"""
move_yumi.py — Script standalone para mover o braço direito do YuMi via RWS.

Atualiza o jointtarget na variável RAPID move_jointtarget_input, define a rotina
runMoveAbsJ e pulsa o sinal RUN_RAPID_ROUTINE para desencadear o movimento.
Útil para testes rápidos sem lançar o stack ROS 2 completo.

Part of the yumi_rws_interface package.
Project: Collaborative Robotics for RCD Pre-Assembly
Author: Rui Martins (up202108756@edu.fe.up.pt)
Supervisors: Prof. Luís F. Rocha, Prof. [co-orientador]
FEUP/INESCTEC, 2026
"""

import requests
from requests.auth import HTTPDigestAuth

# =============================
# CONFIGURAÇÃO
# =============================
ROBOT_IP = "192.168.125.1"      # IP do YuMi
USERNAME = "Default User"
PASSWORD = "robotics"

TASK = "T_ROB_R"                # Task do braço direito
MODULE = "TRobRAPID"            # Nome do módulo RAPID

BASE_URL = f"http://{ROBOT_IP}/rw/rapid/symbol/data/RAPID"

# =============================
# FUNÇÕES AUXILIARES
# =============================
def set_rapid_variable(task, module, variable, value):
    """
    Escreve uma variável RAPID via RWS
    """
    url = f"{BASE_URL}/{task}/{module}/{variable}?action=set"
    payload = {"value": value}

    r = requests.post(
        url,
        data=payload,
        auth=HTTPDigestAuth(USERNAME, PASSWORD)
    )

    if r.status_code == 204:
        print(f"✔ {variable} atualizado com sucesso")
    else:
        print(f"✖ Erro ao atualizar {variable}: {r.text}")


def pulse_digital_signal(signal_name):
    """
    Dispara a entrada digital RUN_RAPID_ROUTINE
    """
    url = f"http://{ROBOT_IP}/rw/iosystem/signals/{signal_name}?action=set"
    
    requests.post(url, data={"lvalue": "1"}, auth=HTTPDigestAuth(USERNAME, PASSWORD))
    requests.post(url, data={"lvalue": "0"}, auth=HTTPDigestAuth(USERNAME, PASSWORD))

    print(f"✔ Sinal {signal_name} ativado")

# =============================
# FUNÇÃO PRINCIPAL
# =============================
def main():
    """
    Envia o jointtarget e dispara a rotina
    """

    # -----------------------------
    # Jointtarget (6 juntas + 6 eixos externos)
    # Substitui os valores das juntas conforme necessário
    # -----------------------------
    joint_target = "[[10.0,0.0,0.0,0.0,0.0,0.0],[130.0,9E9,9E9,9E9,9E9,9E9]]"

    print("A enviar jointtarget...")
    set_rapid_variable(TASK, MODULE, "move_jointtarget_input", joint_target)

    # -----------------------------
    # Define a rotina RAPID
    # -----------------------------
    print("A definir rotina...")
    set_rapid_variable(TASK, MODULE, "routine_name_input", '"runMoveAbsJ"')

    # -----------------------------
    # Dispara a execução
    # -----------------------------
    print("A disparar execução...")
    pulse_digital_signal("RUN_RAPID_ROUTINE")

    print("🚀 Movimento enviado!")

# =============================
# EXECUÇÃO
# =============================
if __name__ == "__main__":
    main()