#!/usr/bin/env python3
"""
add_table_collision.py — Adiciona plano de colisão da mesa ao MoveIt2.

Corre depois do launch para o MoveIt2 não planear movimentos abaixo da mesa.

Uso:
  python3 add_table_collision.py

Project: Collaborative Robotics for RCD Pre-Assembly
Author: Rui Martins (up202108756@edu.fe.up.pt)
FEUP/INESCTEC, 2026
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import time


class TableCollisionNode(Node):
    def __init__(self):
        super().__init__("add_table_collision")
        self._pub = self.create_publisher(
            CollisionObject, "/collision_object", 10)
        self._scene_pub = self.create_publisher(
            PlanningScene, "/planning_scene", 10)

    def add_table(self,
                  z_top=0.0,        # altura da superfície da mesa (frame world)
                  thickness=0.05,   # espessura do plano (m)
                  width=2.0,        # largura (m)
                  depth=2.0):       # profundidade (m)

        obj = CollisionObject()
        obj.id = "table"
        obj.header.frame_id = "world"
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.operation = CollisionObject.ADD

        # Caixa plana como mesa
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [width, depth, thickness]
        obj.primitives.append(box)

        # Centro da caixa fica meio abaixo de z_top
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = z_top - thickness / 2.0
        pose.orientation.w = 1.0
        obj.primitive_poses.append(pose)

        # Publica o objeto
        time.sleep(0.5)
        self._pub.publish(obj)
        time.sleep(0.5)
        self._pub.publish(obj)  # publica duas vezes por segurança

        self.get_logger().info(
            f"✓ Mesa adicionada: z_top={z_top:.3f}m, "
            f"{width}x{depth}x{thickness}m")


def main():
    rclpy.init()
    node = TableCollisionNode()

    # Aguarda o MoveIt2 arrancar
    print("A aguardar MoveIt2...")
    time.sleep(2.0)

    # Ajusta z_top conforme a tua montagem:
    #   z_top=0.0  → superfície da mesa ao nível da base do robot
    #   z_top=-0.1 → mesa 10cm abaixo da base
    node.add_table(z_top=-0.1)

    print("✓ Plano de colisão da mesa adicionado ao MoveIt2.")
    print("  O robot não vai planear movimentos abaixo de z=0.0m.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()