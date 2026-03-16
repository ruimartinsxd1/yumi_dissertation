#!/usr/bin/env python3
"""
move_yumi_wasd_no_rapid.py — Controlo interativo do YuMi por teclado (WASD).

Permite controlar as 3 primeiras juntas do braço direito em tempo real,
sem necessidade de pressionar Enter entre comandos. Usa a biblioteca curses
para captura de teclas não-bloqueante.

Mapeamento de teclas:
  W/S → Joint 1 (±STEP graus)
  A/D → Joint 2 (±STEP graus)
  Q/E → Joint 3 (±STEP graus)
  H   → Home (runMoveToCalibrationPosition)
  X   → Sair

Part of the yumi_rws_interface package.
Project: Collaborative Robotics for RCD Pre-Assembly
Author: Rui Martins (up202108756@edu.fe.up.pt)
Supervisors: Prof. Luís F. Rocha, Prof. [co-orientador]
FEUP/INESCTEC, 2026
"""

import curses
import time

import requests
from requests.auth import HTTPDigestAuth

ROBOT_IP = "192.168.125.1"
USERNAME = "Default User"
PASSWORD = "robotics"

TASK = "T_ROB_R"
MODULE = "TRobRAPID"

BASE_RAPID = f"http://{ROBOT_IP}/rw/rapid/symbol/data/RAPID"
session = requests.Session()
session.auth = HTTPDigestAuth(USERNAME, PASSWORD)

# Template de jointtarget seguro
jointtarget_template = [
    [10.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # juntas
    [130.0, 9E9, 9E9, 9E9, 9E9, 9E9]  # eixos externos
]
STEP = 20  # Movimento brusco


def send_jointtarget(jt):
    """Atualiza jointtarget e dispara MoveAbsJ"""
    url_joint = f"{BASE_RAPID}/{TASK}/{MODULE}/move_jointtarget_input?action=set"
    session.post(url_joint, data={"value": str(jt)}, auth=session.auth)

    url_routine = f"{BASE_RAPID}/{TASK}/{MODULE}/routine_name_input?action=set"
    session.post(url_routine, data={"value": '"runMoveAbsJ"'}, auth=session.auth)

    url_signal = f"http://{ROBOT_IP}/rw/iosystem/signals/RUN_RAPID_ROUTINE?action=set"
    session.post(url_signal, data={"lvalue": "1"}, auth=session.auth)
    session.post(url_signal, data={"lvalue": "0"}, auth=session.auth)


def go_home():
    """Dispara Home usando a rotina correta"""
    routine_name = '"runMoveToCalibrationPosition"'  # nome certo da rotina
    url_routine = f"{BASE_RAPID}/{TASK}/{MODULE}/routine_name_input?action=set"
    session.post(url_routine, data={"value": routine_name}, auth=session.auth)

    url_signal = f"http://{ROBOT_IP}/rw/iosystem/signals/RUN_RAPID_ROUTINE?action=set"
    session.post(url_signal, data={"lvalue": "1"}, auth=session.auth)
    session.post(url_signal, data={"lvalue": "0"}, auth=session.auth)


def main(stdscr):
    curses.cbreak()
    stdscr.nodelay(True)  # não bloqueia o getch()
    stdscr.keypad(True)
    current_jt = [list(j) for j in jointtarget_template]

    stdscr.addstr(0, 0, "🎮 WASD LIVE - Controle rápido do YuMi")
    stdscr.addstr(1, 0, "W/S -> J1 | A/D -> J2 | Q/E -> J3 | H -> Home | X -> Sair")

    while True:
        try:
            key = stdscr.getch()
            if key == -1:
                time.sleep(0.05)
                continue

            char = chr(key).lower()

            if char == "x":
                break
            elif char == "h":
                go_home()
                stdscr.addstr(3, 0, "🏠 YuMi a caminho da Home!             ")
            elif char == "w":
                current_jt[0][0] += STEP
            elif char == "s":
                current_jt[0][0] -= STEP
            elif char == "a":
                current_jt[0][1] += STEP
            elif char == "d":
                current_jt[0][1] -= STEP
            elif char == "q":
                current_jt[0][2] += STEP
            elif char == "e":
                current_jt[0][2] -= STEP
            else:
                continue

            if char in "wasdqexh":
                send_jointtarget(current_jt)
                stdscr.addstr(2, 0, f"Tecla: {char.upper()} | Movimento enviado!        ")

        except Exception as e:
            stdscr.addstr(4, 0, f"❌ Erro: {e}")

    stdscr.addstr(5, 0, "✅ Script terminado. Pressione qualquer tecla para sair.")
    stdscr.nodelay(False)
    stdscr.getch()


# no final do teu script move_yumi_wasd_live.py

def main_wrapper():
    curses.wrapper(main)  # chama main com stdscr

if __name__ == "__main__":
    main_wrapper()