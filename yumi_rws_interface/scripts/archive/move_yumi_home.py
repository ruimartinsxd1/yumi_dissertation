#!/usr/bin/env python3
"""
move_yumi_home.py — Script standalone para mover o YuMi para a posição Home.

Aciona a rotina RAPID runMoveToCalibrationPosition via RWS, que move ambos os
braços para a posição de calibração definida no FlexPendant.
Útil para restaurar uma posição segura antes ou depois de operações.

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
ROBOT_IP = "192.168.125.1"       # IP do YuMi
USERNAME = "Default User"
PASSWORD = "robotics"

TASK = "T_ROB_R"                 # Task do braço direito
MODULE = "TRobRAPID"             # Nome do módulo RAPID

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
    Move o YuMi para a posição Home
    """

    # -----------------------------
    # Define a rotina RAPID de home
    # -----------------------------
    routine_name = '"runMoveToCalibrationPosition"'

    print("A definir rotina para Home...")
    set_rapid_variable(TASK, MODULE, "routine_name_input", routine_name)

    # -----------------------------
    # Dispara a execução
    # -----------------------------
    print("A disparar execução...")
    pulse_digital_signal("RUN_RAPID_ROUTINE")

    print("🚀 YuMi a caminho da posição Home!")

# =============================
# EXECUÇÃO
# =============================
if __name__ == "__main__":
    main()