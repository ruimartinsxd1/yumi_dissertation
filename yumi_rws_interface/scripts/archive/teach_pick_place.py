#!/usr/bin/env python3
"""
teach_pick_place.py — Teach & Replay interativo de pick and place (braço direito).

Fase 1 (Teach): move o braço manualmente com o FlexPendant e grava as posições.
Fase 2 (Replay): executa a sequência automaticamente com gripper open/close.
Posições gravadas em /tmp/teach_positions.json.

Part of the yumi_rws_interface package.
Project: Collaborative Robotics for RCD Pre-Assembly
Author: Rui Martins (up202108756@edu.fe.up.pt)
Supervisors: Prof. Luís F. Rocha, Prof. [co-orientador]
FEUP/INESCTEC, 2026
"""

import json
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
    read_cartesian,
    read_joints,
    wait_position,
)

SIDE = "R"  # R=direito, L=esquerdo

session = get_session()
BASE = get_base_url()


# ============================
# FASE 1: ENSINAR POSIÇÕES
# ============================

print("=" * 60)
print("  TEACH & REPLAY — Pick and Place (YuMi)")
print(f"  Braço: {'Direito' if SIDE == 'R' else 'Esquerdo'}")
print("=" * 60)
print()
print("INSTRUÇÕES:")
print("  1. Muda para MANUAL no FlexPendant")
print("  2. Usa o joystick para mover o braço")
print("  3. Carrega ENTER aqui para gravar cada posição")
print("  4. Muda para AUTO + Play antes de executar")
print()

positions = {}
steps = [
    ("approach_pick", "Posição de APROXIMAÇÃO ao fio (acima do fio)"),
    ("pick",          "Posição de PICK (onde agarra o fio)"),
    ("approach_place","Posição de APROXIMAÇÃO ao destino (acima do destino)"),
    ("place",         "Posição de PLACE (onde larga o fio)"),
]

for key, desc in steps:
    input(f">> Move o braço para: {desc}\n   Carrega ENTER para gravar...")
    joints = read_joints(session, BASE, SIDE)
    cart = read_cartesian(session, BASE, SIDE)
    positions[key] = joints
    print(f"   ✓ Gravado: joints={[round(j,1) for j in joints]}")
    print(f"              xyz=[{cart[0]:.1f}, {cart[1]:.1f}, {cart[2]:.1f}] mm")
    print()

# Guardar posições num ficheiro
with open("/tmp/teach_positions.json", "w") as f:
    json.dump(positions, f, indent=2)
print("Posições guardadas em /tmp/teach_positions.json")

# ============================
# FASE 2: EXECUTAR SEQUÊNCIA
# ============================

print()
print("=" * 60)
input("Muda para AUTO + Play no FlexPendant, depois ENTER para executar...")
print()

print("1/6 — Abrir gripper...")
gripper_cmd(session, BASE, SIDE, 5)
time.sleep(2.0)

print("2/6 — Mover para approach_pick...")
move_to(session, BASE, SIDE, positions["approach_pick"])
wait_position(session, BASE, SIDE, positions["approach_pick"])

print("3/6 — Mover para pick (agarrar fio)...")
move_to(session, BASE, SIDE, positions["pick"], speed="[50,25,50,25]")
wait_position(session, BASE, SIDE, positions["pick"])

print("4/6 — Fechar gripper...")
gripper_cmd(session, BASE, SIDE, 4)
time.sleep(2.0)

print("5/6 — Mover para approach_place...")
move_to(session, BASE, SIDE, positions["approach_pick"])  # subir primeiro
wait_position(session, BASE, SIDE, positions["approach_pick"])
move_to(session, BASE, SIDE, positions["approach_place"])
wait_position(session, BASE, SIDE, positions["approach_place"])

print("6/6 — Mover para place e largar...")
move_to(session, BASE, SIDE, positions["place"], speed="[50,25,50,25]")
wait_position(session, BASE, SIDE, positions["place"])

print("     — Abrir gripper...")
gripper_cmd(session, BASE, SIDE, 5)
time.sleep(2.0)

print("     — Recuar para approach_place...")
move_to(session, BASE, SIDE, positions["approach_place"])
wait_position(session, BASE, SIDE, positions["approach_place"])

print()
print("✓ Pick and Place concluído!")
print("=" * 60)

session.close()
