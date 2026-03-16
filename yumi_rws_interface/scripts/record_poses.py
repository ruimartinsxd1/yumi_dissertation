#!/usr/bin/env python3
"""
record_poses.py — Grava poses do YuMi interactivamente.

Uso:
  source ~/yumi_ws/install/setup.bash
  python3 record_poses.py                          # sequência completa
  python3 record_poses.py --update handover_left   # corrigir só uma pose

Move o robot no RViz, prime Enter para gravar.
Preserva sempre as poses já existentes no ficheiro.

Project: Collaborative Robotics for RCD Pre-Assembly
Author: Rui Martins (up202108756@edu.fe.up.pt)
FEUP/INESCTEC, 2026
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml
import threading
import os
import sys

JOINT_NAMES_LEFT  = ["yumi_joint_1_l", "yumi_joint_2_l", "yumi_joint_7_l",
                      "yumi_joint_3_l", "yumi_joint_4_l", "yumi_joint_5_l", "yumi_joint_6_l"]
JOINT_NAMES_RIGHT = ["yumi_joint_1_r", "yumi_joint_2_r", "yumi_joint_7_r",
                      "yumi_joint_3_r", "yumi_joint_4_r", "yumi_joint_5_r", "yumi_joint_6_r"]

# Guarda sempre na mesma pasta do script
OUTPUT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "poses.yaml")

SUGGESTED = [
    "home",
    "right_pick_approach",
    "right_pick",
    "handover_right",
    "handover_left",
    "left_insert_approach",
    "left_insert",
]


class PoseRecorder(Node):
    def __init__(self):
        super().__init__("pose_recorder")
        self._latest = {}
        self._lock = threading.Lock()
        self.create_subscription(JointState, "/joint_states", self._cb, 10)

    def _cb(self, msg):
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                self._latest[name] = round(pos, 6)

    def get_pose(self):
        with self._lock:
            left  = [self._latest.get(n, 0.0) for n in JOINT_NAMES_LEFT]
            right = [self._latest.get(n, 0.0) for n in JOINT_NAMES_RIGHT]
        return left, right


def load_existing():
    if os.path.exists(OUTPUT_FILE):
        with open(OUTPUT_FILE) as f:
            data = yaml.safe_load(f) or {}
        print(f"✓ Poses existentes carregadas: {list(data.keys())}")
        return data
    return {}


def save(poses):
    with open(OUTPUT_FILE, "w") as f:
        yaml.dump(poses, f, default_flow_style=False)
    print(f"\n✓ {len(poses)} poses guardadas em '{OUTPUT_FILE}'")
    print("\nConteúdo:")
    print("─" * 50)
    for name, data in poses.items():
        print(f"  {name}:")
        print(f"    left:  {data['left']}")
        print(f"    right: {data['right']}")
    print("─" * 50)


def record_one(node, name, poses):
    input(f"\n  Move o robot para '{name}' e prime Enter para gravar...")
    left, right = node.get_pose()
    poses[name] = {"left": left, "right": right}
    print(f"  ✓ '{name}' gravada:")
    print(f"     LEFT:  {[f'{v:.4f}' for v in left]}")
    print(f"     RIGHT: {[f'{v:.4f}' for v in right]}")
    return poses


def main():
    # Modo --update <nome>
    update_mode = None
    if "--update" in sys.argv:
        idx = sys.argv.index("--update")
        if idx + 1 < len(sys.argv):
            update_mode = sys.argv[idx + 1]
        else:
            print("Uso: python3 record_poses.py --update <nome_da_pose>")
            print(f"Poses disponíveis: {SUGGESTED}")
            sys.exit(1)

    rclpy.init()
    node = PoseRecorder()

    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    import time; time.sleep(1.0)

    poses = load_existing()

    if update_mode:
        # ---- Corrigir só uma pose ----
        print(f"\n=== Modo correcção: a actualizar '{update_mode}' ===")
        if update_mode in poses:
            print(f"  Valor actual left:  {poses[update_mode]['left']}")
            print(f"  Valor actual right: {poses[update_mode]['right']}")
        record_one(node, update_mode, poses)
        save(poses)

    else:
        # ---- Sequência completa ----
        print("\n=== Gravador de Poses YuMi ===")
        print("Move o robot no RViz para a pose que queres e prime Enter.")
        print("Escreve 'fim' para terminar | 'skip' para saltar.\n")

        print("Poses sugeridas:")
        for i, name in enumerate(SUGGESTED, 1):
            status = "  ← já gravada" if name in poses else ""
            print(f"  {i}. {name}{status}")
        print()

        idx_sugerida = 0

        while True:
            while idx_sugerida < len(SUGGESTED) and SUGGESTED[idx_sugerida] in poses:
                idx_sugerida += 1

            sugerida = SUGGESTED[idx_sugerida] if idx_sugerida < len(SUGGESTED) else None
            hint = f", Enter = '{sugerida}'" if sugerida else ""
            nome = input(f"Nome da pose ('fim'{hint}, 'skip'): ").strip()

            if nome.lower() == "fim":
                break
            if nome.lower() == "skip":
                if sugerida:
                    print(f"  → Saltada '{sugerida}'")
                    idx_sugerida += 1
                continue
            if not nome:
                if sugerida:
                    nome = sugerida
                    print(f"  → A usar nome sugerido: '{nome}'")
                    idx_sugerida += 1
                else:
                    nome = f"pose_{len(poses)}"

            record_one(node, nome, poses)

        save(poses)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()