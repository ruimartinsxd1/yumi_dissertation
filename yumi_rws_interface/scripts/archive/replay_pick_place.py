#!/usr/bin/env python3
"""
replay_pick_place.py — Replay de pick and place com posições pré-gravadas.

Executa uma sequência de pick and place usando posições hardcoded (braço direito).
Para usar posições ensinadas interativamente, ver teach_pick_place.py.

Part of the yumi_rws_interface package.
Project: Collaborative Robotics for RCD Pre-Assembly
Author: Rui Martins (up202108756@edu.fe.up.pt)
Supervisors: Prof. Luís F. Rocha, Prof. [co-orientador]
FEUP/INESCTEC, 2026
"""

import os
import sys
import time

# Permitir importar yumi_utils ao correr o script diretamente
sys.path.insert(0, os.path.dirname(__file__))
from yumi_utils import (
    get_base_url,
    get_session,
    gripper_cmd,
    move_to,
    wait_position,
)

SIDE = "R"

session = get_session()
BASE = get_base_url()

# Posições gravadas
positions = {
    "approach_pick":  [100.5, -130.0, 15.9, 34.7, 19.6, 56.9, -55.5],
    "pick":           [100.5, -130.0, 6.7, 30.3, 16.8, 58.2, -55.5],
    "approach_place": [100.5, -130.0, 22.0, -11.9, -3.0, 90.3, -96.1],
    "place":          [100.5, -130.0, -5.5, -11.1, 14.2, 90.2, -96.1],
}

# Executar
print("Pick and Place — Replay")
print("Confirma: AUTO + Play no FlexPendant")
input("ENTER para executar...")

print("1/6 — Abrir gripper...")
gripper_cmd(session, BASE, SIDE, 5)
time.sleep(2.0)

print("2/6 — Approach pick...")
move_to(session, BASE, SIDE, positions["approach_pick"])
wait_position(session, BASE, SIDE, positions["approach_pick"])

print("3/6 — Pick...")
move_to(session, BASE, SIDE, positions["pick"], speed="[50,25,50,25]")
wait_position(session, BASE, SIDE, positions["pick"])

print("4/6 — Fechar gripper...")
gripper_cmd(session, BASE, SIDE, 4)
time.sleep(2.0)

print("5/6 — Approach place...")
move_to(session, BASE, SIDE, positions["approach_pick"])
wait_position(session, BASE, SIDE, positions["approach_pick"])
move_to(session, BASE, SIDE, positions["approach_place"])
wait_position(session, BASE, SIDE, positions["approach_place"])

print("6/6 — Place + largar...")
move_to(session, BASE, SIDE, positions["place"], speed="[50,25,50,25]")
wait_position(session, BASE, SIDE, positions["place"])
gripper_cmd(session, BASE, SIDE, 5)
time.sleep(2.0)
move_to(session, BASE, SIDE, positions["approach_place"])
wait_position(session, BASE, SIDE, positions["approach_place"])

print("✓ Concluído!")
session.close()
