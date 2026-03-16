#!/usr/bin/env python3
"""
replay_handover_safe.py — Replay de handover dual-arm com movimentos seguros.

Versão com pontos intermédios de segurança (R_safe, L_safe, L_pre_handover)
para evitar colisões durante a transição entre braços.
Arquivado a partir de test_2.py — sequência validada no robô real.

Part of the yumi_rws_interface package.
Project: Collaborative Robotics for RCD Pre-Assembly
Author: Rui Martins (up202108756@edu.fe.up.pt)
Supervisors: Prof. Luís F. Rocha, Prof. [co-orientador]
FEUP/INESCTEC, 2026
"""

import re, time, requests
from requests.auth import HTTPDigestAuth

ROBOT_IP = "192.168.125.1"
session = requests.Session()
session.auth = HTTPDigestAuth("Default User", "robotics")
BASE = f"http://{ROBOT_IP}"
MODULE = "TRobRAPID"

# Posições ensinadas
pos = {
    "R_approach_pick": [78.95, -99.09, 22.21, -172.85, -8.20, 99.91, -79.04],
    "R_pick":          [91.58, -104.38, -10.30, -165.05, -33.45, 95.76, -83.06],
    "R_handover":      [96.52, -101.25, 12.57, -132.66, -16.63, 231.04, -110.54],
    "L_handover":      [-68.15, -112.34, 16.80, 33.85, 53.10, 41.82, 90.69],
    "L_approach_place":[-77.09, -112.33, 0.43, 8.63, 29.78, 41.87, 71.44],
    "L_place":         [-77.08, -112.35, -12.14, 8.63, 29.73, 41.87, 84.19],
}

# Pontos intermédios SEGUROS (braço recolhido e alto, no lado de cada braço)
# Direito: posição alta no lado direito, longe do esquerdo
R_safe = [78.95, -130.0, 30.0, -172.85, -8.20, 99.91, -79.04]
# Esquerdo: posição alta no lado esquerdo, longe do direito
L_safe = [-68.15, -130.0, 30.0, 33.85, 53.10, 41.82, 90.69]
# Esquerdo: aproximação ao handover (alto, descerá devagar)
L_pre_handover = [-68.15, -112.34, 35.0, 33.85, 53.10, 41.82, 90.69]


def read_joints(side):
    r = session.get(f"{BASE}/rw/motionsystem/mechunits/ROB_{side}/jointtarget", timeout=2)
    vals = re.findall(r'class="(?:rax_\d|eax_a)">([^<]+)<', r.text)
    return [float(v) for v in vals[:7]]

def move_to(side, joints, speed="[1000,500,1000,500]"):
    task = f"T_ROB_{side}"
    j = joints
    jt = f"[[{j[0]},{j[1]},{j[2]},{j[3]},{j[4]},{j[5]}],[{j[6]},9E9,9E9,9E9,9E9,9E9]]"
    session.post(f"{BASE}/rw/rapid/symbol/data/RAPID/{task}/{MODULE}/move_jointtarget_input?action=set",
                 data={"value": jt}, timeout=3)
    session.post(f"{BASE}/rw/rapid/symbol/data/RAPID/{task}/{MODULE}/move_speed_input?action=set",
                 data={"value": speed}, timeout=3)
    session.post(f"{BASE}/rw/rapid/symbol/data/RAPID/{task}/{MODULE}/routine_name_input?action=set",
                 data={"value": '"runMoveAbsJ"'}, timeout=3)
    session.post(f"{BASE}/rw/iosystem/signals/RUN_RAPID_ROUTINE?action=set",
                 data={"lvalue": "1"}, timeout=2)
    time.sleep(0.1)
    session.post(f"{BASE}/rw/iosystem/signals/RUN_RAPID_ROUTINE?action=set",
                 data={"lvalue": "0"}, timeout=2)

def gripper(side, cmd):
    task = f"T_ROB_{side}"
    session.post(f"{BASE}/rw/rapid/symbol/data/RAPID/{task}/TRobSG/command_input?action=set",
                 data={"value": str(cmd)}, timeout=3)
    session.post(f"{BASE}/rw/iosystem/signals/RUN_SG_ROUTINE?action=set",
                 data={"lvalue": "1"}, timeout=2)
    time.sleep(0.1)
    session.post(f"{BASE}/rw/iosystem/signals/RUN_SG_ROUTINE?action=set",
                 data={"lvalue": "0"}, timeout=2)

def wait(side, target, tol=2.0, timeout=15.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        cur = read_joints(side)
        if all(abs(c - t) < tol for c, t in zip(cur, target)):
            return True
        time.sleep(0.2)
    return False

FAST = "[800,400,800,400]"
SLOW = "[100,50,100,50]"

print("=" * 50)
print("  Handover v2 — Movimentos Seguros")
print("=" * 50)
input("Confirma AUTO + Play. ENTER...")

# 1. Preparar: abrir grippers, braços para posição segura
print(" 1/13 — Abrir grippers + posição segura...")
gripper("R", 5); gripper("L", 5); time.sleep(1.5)
move_to("R", R_safe, SLOW); wait("R", R_safe)
move_to("L", L_safe, SLOW); wait("L", L_safe)

# 2. Direito: approach pick
print(" 2/13 — [DIR] Approach pick...")
move_to("R", pos["R_approach_pick"], SLOW); wait("R", pos["R_approach_pick"])

# 3. Direito: pick
print(" 3/13 — [DIR] Pick (lento)...")
move_to("R", pos["R_pick"], SLOW); wait("R", pos["R_pick"])

# 4. Direito: fechar gripper
print(" 4/13 — [DIR] Fechar gripper...")
gripper("R", 4); time.sleep(1.5)

# 5. Direito: subir do pick
print(" 5/13 — [DIR] Subir...")
move_to("R", pos["R_approach_pick"], SLOW); wait("R", pos["R_approach_pick"])

# 6. Direito: ir para handover
print(" 6/13 — [DIR] Ir para handover...")
move_to("R", pos["R_handover"], SLOW); wait("R", pos["R_handover"])

# 7. Esquerdo: aproximar ao handover (por cima, no lado dele)
print(" 7/13 — [ESQ] Aproximar ao handover (alto)...")
move_to("L", L_pre_handover, SLOW); wait("L", L_pre_handover)

# 8. Esquerdo: descer para handover (lento)
print(" 8/13 — [ESQ] Descer para handover (lento)...")
move_to("L", pos["L_handover"], SLOW); wait("L", pos["L_handover"])

# 9. HANDOVER
print(" 9/13 — HANDOVER: esquerdo fecha, direito abre...")
gripper("L", 4); time.sleep(1.5)
gripper("R", 5); time.sleep(1.0)

# 10. Separar: esquerdo sobe, direito recua
print("10/13 — Separar braços...")
move_to("L", L_pre_handover, SLOW); wait("L", L_pre_handover)
move_to("R", pos["R_approach_pick"], SLOW); wait("R", pos["R_approach_pick"])
move_to("R", R_safe, SLOW); wait("R", R_safe)

# 11. Esquerdo: ir para approach place
print("11/13 — [ESQ] Approach place...")
move_to("L", pos["L_approach_place"], SLOW); wait("L", pos["L_approach_place"])

# 12. Esquerdo: place
print("12/13 — [ESQ] Place (lento)...")
move_to("L", pos["L_place"], SLOW); wait("L", pos["L_place"])

# 13. Esquerdo: largar e recuar
print("13/13 — [ESQ] Largar e recuar...")
gripper("L", 5); time.sleep(1.5)
move_to("L", pos["L_approach_place"], SLOW); wait("L", pos["L_approach_place"])
move_to("L", L_safe, SLOW); wait("L", L_safe)

print()
print("✓ Handover concluído!")
print("=" * 50)
session.close()
