#!/usr/bin/env python3
"""
teach_handover.py — Teach & Replay interativo de handover dual-arm.

Fase 1 (Teach): grava 6 posições (3 por braço) via FlexPendant.
Fase 2 (Replay): executa a sequência completa de handover automaticamente.
  Direito agarra → entrega ao esquerdo → esquerdo coloca.
Posições gravadas em /tmp/teach_handover.json.

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

session = get_session()
BASE = get_base_url()


def teach_point(side: str, description: str) -> list[float]:
    """Aguarda input do utilizador, lê e devolve as 7 juntas do braço.

    Args:
        side: Braço — ``"R"`` ou ``"L"``.
        description: Descrição da posição a ensinar.

    Returns:
        Lista de 7 ângulos em graus.
    """
    side_name = "DIREITO" if side == "R" else "ESQUERDO"
    input(f">> [{side_name}] Move para: {description}\n   ENTER para gravar...")
    joints = read_joints(session, BASE, side)
    cart = read_cartesian(session, BASE, side)
    print(f"   ✓ Gravado: joints={[round(j,1) for j in joints]}")
    print(f"              xyz=[{cart[0]:.1f}, {cart[1]:.1f}, {cart[2]:.1f}] mm\n")
    return joints


# ============================
# FASE 1: ENSINAR POSIÇÕES
# ============================

print("=" * 60)
print("  TEACH & REPLAY — Dual Arm Handover")
print("  Direito agarra → passa ao Esquerdo → Esquerdo coloca")
print("=" * 60)
print()
print("INSTRUÇÕES:")
print("  1. Muda para MANUAL no FlexPendant")
print("  2. Move cada braço com o joystick")
print("  3. Carrega ENTER para gravar cada posição")
print("  4. Muda para AUTO + Play antes de executar")
print()

pos = {}

# Braço direito - pick
print("--- BRAÇO DIREITO (pick) ---")
pos["R_approach_pick"] = teach_point("R", "Aproximação ao objeto (acima)")
pos["R_pick"]          = teach_point("R", "Posição de PICK (agarrar)")
pos["R_handover"]      = teach_point("R", "Ponto de ENCONTRO (onde entrega ao esquerdo)")

# Braço esquerdo - receive e place
print("--- BRAÇO ESQUERDO (receive + place) ---")
pos["L_handover"]      = teach_point("L", "Ponto de ENCONTRO (onde recebe do direito)")
pos["L_approach_place"]= teach_point("L", "Aproximação ao destino (acima)")
pos["L_place"]         = teach_point("L", "Posição de PLACE (largar)")

# Guardar
with open("/tmp/teach_handover.json", "w") as f:
    json.dump(pos, f, indent=2)
print("Posições guardadas em /tmp/teach_handover.json\n")

# ============================
# FASE 2: EXECUTAR
# ============================

print("=" * 60)
input("Muda para AUTO + Play no FlexPendant, depois ENTER...")
print()

FAST = "[1000,500,1000,500]"
SLOW = "[100,50,100,50]"

# 1. Abrir ambos os grippers
print("1/10 — Abrir ambos os grippers...")
gripper_cmd(session, BASE, "R", 5)
gripper_cmd(session, BASE, "L", 5)
time.sleep(1.5)

# 2. Direito: approach pick
print("2/10 — [DIR] Approach pick...")
move_to(session, BASE, "R", pos["R_approach_pick"], FAST)
wait_position(session, BASE, "R", pos["R_approach_pick"])

# 3. Direito: pick
print("3/10 — [DIR] Pick...")
move_to(session, BASE, "R", pos["R_pick"], SLOW)
wait_position(session, BASE, "R", pos["R_pick"])

# 4. Direito: fechar gripper
print("4/10 — [DIR] Fechar gripper...")
gripper_cmd(session, BASE, "R", 4)
time.sleep(1.5)

# 5. Direito: subir e ir para handover
print("5/10 — [DIR] Ir para ponto de encontro...")
move_to(session, BASE, "R", pos["R_approach_pick"], FAST)
wait_position(session, BASE, "R", pos["R_approach_pick"])
move_to(session, BASE, "R", pos["R_handover"], FAST)
wait_position(session, BASE, "R", pos["R_handover"])

# 6. Esquerdo: ir para handover
print("6/10 — [ESQ] Ir para ponto de encontro...")
move_to(session, BASE, "L", pos["L_handover"], FAST)
wait_position(session, BASE, "L", pos["L_handover"])

# 7. HANDOVER: esquerdo fecha, direito abre
print("7/10 — HANDOVER: esquerdo fecha, direito abre...")
gripper_cmd(session, BASE, "L", 4)  # esquerdo fecha
time.sleep(1.5)
gripper_cmd(session, BASE, "R", 5)  # direito abre
time.sleep(1.0)

# 8. Direito: recuar
print("8/10 — [DIR] Recuar...")
move_to(session, BASE, "R", pos["R_approach_pick"], FAST)
wait_position(session, BASE, "R", pos["R_approach_pick"])

# 9. Esquerdo: approach place → place
print("9/10 — [ESQ] Ir para place...")
move_to(session, BASE, "L", pos["L_approach_place"], FAST)
wait_position(session, BASE, "L", pos["L_approach_place"])
move_to(session, BASE, "L", pos["L_place"], SLOW)
wait_position(session, BASE, "L", pos["L_place"])

# 10. Esquerdo: largar e recuar
print("10/10 — [ESQ] Largar e recuar...")
gripper_cmd(session, BASE, "L", 5)
time.sleep(1.5)
move_to(session, BASE, "L", pos["L_approach_place"], FAST)
wait_position(session, BASE, "L", pos["L_approach_place"])

print()
print("✓ Handover concluído!")
print("=" * 60)
session.close()
