#!/usr/bin/env python3
"""
rws_trajectory_controller.py — Action server FollowJointTrajectory para MoveIt2 (ABB YuMi via RWS).

Melhorias:
- Não mistura "juntas atuais" com target: valida que para cada lado comandado existem 7 juntas.
- Debug opcional (logs do first/last waypoint + delta vs current).
- Normalização do eixo externo (eax_a / joint_7) pelo "menor caminho" (evita saltos 360° em both_arms).
- _read_joints() mais robusto: faz parse por chave (rax_1..rax_6 + eax_a), sem depender da ordem no HTML.
- Proteções: sanity check de saltos grandes (max_jump_deg), agora em todos os passos consecutivos.
- Seleção de waypoints baseada em salto articular, em vez de equally spaced por índice.

Assume o teu mapeamento do guia:
URDF order: j1, j2, j7, j3, j4, j5, j6
RWS: rax_1..rax_6 e eax_a (como "j7").
"""

import math
import re
import time
import threading
from typing import Dict, List, Optional, Tuple

import requests
from requests.auth import HTTPDigestAuth
from urllib.parse import quote

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory


DEFAULT_SPEED: str = "[200,100,200,100]"
SIGNAL_PULSE_DELAY: float = 0.1
MAX_WAYPOINTS: int = 18


class RWSTrajectoryController(Node):

    # Mapeia nomes URDF -> (lado, índice RWS interno)
    # índice interno: [rax_1..rax_6, eax_a] => [0..5, 6]
    JOINT_MAP: Dict[str, Tuple[str, int]] = {
        "yumi_joint_1_r": ("R", 0),
        "yumi_joint_2_r": ("R", 1),
        "yumi_joint_3_r": ("R", 2),
        "yumi_joint_4_r": ("R", 3),
        "yumi_joint_5_r": ("R", 4),
        "yumi_joint_6_r": ("R", 5),
        "yumi_joint_7_r": ("R", 6),

        "yumi_joint_1_l": ("L", 0),
        "yumi_joint_2_l": ("L", 1),
        "yumi_joint_3_l": ("L", 2),
        "yumi_joint_4_l": ("L", 3),
        "yumi_joint_5_l": ("L", 4),
        "yumi_joint_6_l": ("L", 5),
        "yumi_joint_7_l": ("L", 6),
    }

    TASK = {"L": "T_ROB_L", "R": "T_ROB_R"}
    MODULE = "TRobRAPID"

    def __init__(self):
        super().__init__("rws_trajectory_controller")

        # Parâmetros
        self.declare_parameter("robot_ip", "192.168.125.1")
        self.declare_parameter("username", "Default User")
        self.declare_parameter("password", "robotics")

        self.declare_parameter("position_tolerance_deg", 3.0)
        self.declare_parameter("settle_timeout", 20.0)
        self.declare_parameter("max_waypoints", 16)

        # Melhorias / debug
        self.declare_parameter("debug", True)
        self.declare_parameter("normalize_ext_axis", True)       # normaliza apenas idx=6 (eax_a)
        self.declare_parameter("max_jump_deg", 220.0)            # proteção contra saltos absurdos (por junta)
        self.declare_parameter("resample_max_joint_step_deg", 6.0)

        ip = self.get_parameter("robot_ip").value
        user = self.get_parameter("username").value
        pwd = self.get_parameter("password").value

        self.pos_tol = float(self.get_parameter("position_tolerance_deg").value)
        self.settle_timeout = float(self.get_parameter("settle_timeout").value)
        self.max_wp = min(int(self.get_parameter("max_waypoints").value), MAX_WAYPOINTS)

        self.debug = bool(self.get_parameter("debug").value)
        self.normalize_ext_axis = bool(self.get_parameter("normalize_ext_axis").value)
        self.max_jump_deg = float(self.get_parameter("max_jump_deg").value)
        self.resample_max_joint_step_deg = float(
            self.get_parameter("resample_max_joint_step_deg").value
        )

        # HTTP session
        self.session = requests.Session()
        self.session.auth = HTTPDigestAuth(user, pwd)

        self.base_url = f"http://{ip}"
        self.base_rapid = f"{self.base_url}/rw/rapid/symbol/data/RAPID"
        self.base_io = f"{self.base_url}/rw/iosystem/signals"
        self.base_motion = f"{self.base_url}/rw/motionsystem/mechunits"

        self._rws_lock = threading.Lock()

        # Action servers
        cb_group = ReentrantCallbackGroup()
        for name in ["left_arm_controller", "right_arm_controller", "both_arms_controller"]:
            ActionServer(
                self,
                FollowJointTrajectory,
                f"/{name}/follow_joint_trajectory",
                execute_callback=self.execute_callback,
                goal_callback=self.goal_callback,
                cancel_callback=self.cancel_callback,
                callback_group=cb_group,
            )
            self.get_logger().info(f"Action server: /{name}/follow_joint_trajectory")

        self.get_logger().info(
            f"RWS Trajectory Controller pronto — robot={ip}, max_wp={self.max_wp}, "
            f"resample_max_joint_step_deg={self.resample_max_joint_step_deg}"
        )

    # ================================================================
    # Action callbacks
    # ================================================================

    def goal_callback(self, goal_request):
        n = len(goal_request.trajectory.points)
        self.get_logger().info(f"Goal recebido: {n} pontos")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel recebido")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        joint_names = trajectory.joint_names
        points = trajectory.points

        if not points:
            self.get_logger().warn("Trajetória vazia")
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            goal_handle.abort()
            return result

        # Determinar lados que este goal realmente comanda
        sides_in_goal = self._infer_sides_in_goal(joint_names)
        if not sides_in_goal:
            self.get_logger().error("Goal não contém juntas conhecidas (JOINT_MAP).")
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            goal_handle.abort()
            return result

        # Ler estado atual UMA vez por lado (para debug + normalização)
        current_by_side: Dict[str, Optional[List[float]]] = {
            s: self._read_joints(s) for s in sides_in_goal
        }

        # Selecionar waypoints de forma mais inteligente
        selected_indices = self._select_waypoints_joint_based(
            joint_names,
            points,
            sides_in_goal,
            current_by_side,
        )
        self.get_logger().info(f"Multi-waypoint: {len(selected_indices)} de {len(points)} pontos")

        # Converter pontos selecionados para RWS (sem misturar com "current")
        sides_waypoints: Dict[str, List[List[float]]] = {s: [] for s in sides_in_goal}

        for pt_idx in selected_indices:
            point = points[pt_idx]
            rws_point = self._point_to_rws_strict(joint_names, point, sides_in_goal)
            if rws_point is None:
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                goal_handle.abort()
                return result
            for lado, joints_deg in rws_point.items():
                sides_waypoints[lado].append(joints_deg)

        # Normalizar eax_a (idx=6) pelo menor caminho vs current
        if self.normalize_ext_axis:
            for lado, wps in sides_waypoints.items():
                cur = current_by_side.get(lado)
                if cur is None:
                    continue
                self._normalize_ext_axis_waypoints_inplace(wps, cur_ext=cur[6])

        # Proteção: rejeitar saltos absurdos em todos os passos
        try:
            for lado, wps in sides_waypoints.items():
                cur = current_by_side.get(lado)
                if cur is not None:
                    self._sanity_check_all_steps(lado, wps, cur)
        except RuntimeError as e:
            self.get_logger().error(f"Trajetória rejeitada por segurança: {e}")
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            goal_handle.abort()
            return result

        # Debug: imprimir first/last waypoint + delta
        if self.debug:
            self.get_logger().info(f"[DEBUG] joint_names={joint_names}")
            for lado, wps in sides_waypoints.items():
                cur = current_by_side.get(lado)
                first = wps[0]
                last = wps[-1]

                self.get_logger().info(
                    f"[DEBUG] {lado} wp_count={len(wps)} first=[{', '.join(f'{v:.1f}' for v in first)}]"
                )
                self.get_logger().info(
                    f"[DEBUG] {lado} wp_count={len(wps)} last=[{', '.join(f'{v:.1f}' for v in last)}]"
                )

                if cur is not None:
                    diffs_first = [first[i] - cur[i] for i in range(7)]
                    diffs_last = [last[i] - cur[i] for i in range(7)]
                    self.get_logger().info(
                        f"[DEBUG] {lado} cur=[{', '.join(f'{v:.1f}' for v in cur)}]"
                    )
                    self.get_logger().info(
                        f"[DEBUG] {lado} delta(first-cur)=[{', '.join(f'{d:.1f}' for d in diffs_first)}]"
                    )
                    self.get_logger().info(
                        f"[DEBUG] {lado} delta(last-cur)=[{', '.join(f'{d:.1f}' for d in diffs_last)}]"
                    )

        # Escrever TODOS os lados; só pulsar se tudo correr bem
        with self._rws_lock:
            ok_all = True
            for lado, waypoints in sides_waypoints.items():
                ok_side = self._write_multi_waypoints(lado, waypoints)
                ok_all = ok_all and ok_side

            if not ok_all:
                self.get_logger().error("Abortado: falha a escrever waypoints no RAPID")
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                goal_handle.abort()
                return result

            self._pulse_signal()

        # Esperar que chegue ao ponto final
        final_targets = {lado: waypoints[-1] for lado, waypoints in sides_waypoints.items()}
        ok = self._wait_position_reached(final_targets)

        # Feedback final
        final_point = points[-1]
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = joint_names
        feedback.desired.positions = list(final_point.positions)
        feedback.actual.positions = list(final_point.positions)
        feedback.error.positions = [0.0] * len(joint_names)
        goal_handle.publish_feedback(feedback)

        result = FollowJointTrajectory.Result()
        if ok:
            self.get_logger().info("Trajetória concluída com sucesso (suave)")
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            goal_handle.succeed()
        else:
            self.get_logger().warn("Timeout — robô pode não ter chegado à posição")
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            goal_handle.abort()
        return result

    # ================================================================
    # Helpers
    # ================================================================

    def _infer_sides_in_goal(self, joint_names: List[str]) -> List[str]:
        sides = set()
        for jn in joint_names:
            if jn in self.JOINT_MAP:
                sides.add(self.JOINT_MAP[jn][0])
        return sorted(list(sides))

    def _select_waypoints_joint_based(
        self,
        joint_names: List[str],
        points,
        sides_in_goal: List[str],
        current_by_side: Dict[str, Optional[List[float]]],
    ) -> List[int]:
        """
        Seleciona waypoints com base no salto articular máximo entre pontos mantidos.
        Mantém sempre o primeiro e o último.
        """
        n = len(points)
        if n <= 2:
            return list(range(n))

        selected = [0]

        last_kept_rws = self._point_to_rws_strict(joint_names, points[0], sides_in_goal)
        if last_kept_rws is None:
            return [0, n - 1] if n > 1 else [0]

        # Se o primeiro ponto já estiver muito perto do estado atual, continua; senão mantém na mesma.
        for idx in range(1, n - 1):
            cur_rws = self._point_to_rws_strict(joint_names, points[idx], sides_in_goal)
            if cur_rws is None:
                continue

            keep = False
            for lado in sides_in_goal:
                prev = last_kept_rws[lado]
                now = cur_rws[lado]
                for j in range(7):
                    if abs(now[j] - prev[j]) >= self.resample_max_joint_step_deg:
                        keep = True
                        break
                if keep:
                    break

            if keep:
                selected.append(idx)
                last_kept_rws = cur_rws

        if selected[-1] != n - 1:
            selected.append(n - 1)

        # Se ainda houver demasiados pontos, reduz uniformemente sobre a seleção já "boa"
        if len(selected) > self.max_wp:
            step = (len(selected) - 1) / (self.max_wp - 1)
            reduced = [selected[int(i * step)] for i in range(self.max_wp - 1)]
            if reduced[-1] != selected[-1]:
                reduced.append(selected[-1])
            selected = reduced

        # Remover duplicados preservando ordem
        dedup = []
        seen = set()
        for x in selected:
            if x not in seen:
                dedup.append(x)
                seen.add(x)

        if dedup[-1] != n - 1:
            dedup.append(n - 1)

        return dedup

    # ================================================================
    # Conversão URDF → RWS (STRICT)
    # ================================================================

    def _point_to_rws_strict(
        self,
        joint_names: List[str],
        point,
        sides_in_goal: List[str],
    ) -> Optional[Dict[str, List[float]]]:
        """
        Converte JointTrajectoryPoint (rad) -> {lado: [rax1..rax6,eax_a] em graus}
        Sem misturar com juntas atuais. Se faltar alguma junta requerida, retorna None.
        """
        tmp = {lado: [None] * 7 for lado in sides_in_goal}

        for idx, name in enumerate(joint_names):
            if name not in self.JOINT_MAP:
                continue
            lado, rws_idx = self.JOINT_MAP[name]
            if lado not in tmp:
                continue
            tmp[lado][rws_idx] = math.degrees(point.positions[idx])

        out: Dict[str, List[float]] = {}
        for lado in sides_in_goal:
            missing = [i for i, v in enumerate(tmp[lado]) if v is None]
            if missing:
                self.get_logger().error(
                    f"Goal inválido: faltam juntas para lado {lado}. "
                    f"Índices RWS missing={missing} (esperado 0..6). "
                    f"Isto costuma acontecer em both_arms se o SRDF/MoveIt group não tiver as 7 juntas."
                )
                return None
            out[lado] = [float(v) for v in tmp[lado]]
        return out

    # ================================================================
    # Normalização do eixo externo (eax_a) para evitar wrap em both_arms
    # ================================================================

    def _normalize_ext_axis_waypoints_inplace(self, waypoints: List[List[float]], cur_ext: float):
        """
        Ajusta wp[6] (eax_a) em todos os waypoints para ficar "perto" do valor atual (menor delta).
        Faz isto de forma incremental: usa o último valor já normalizado como referência.
        """
        ref = cur_ext
        for wp in waypoints:
            v = wp[6]
            while (v - ref) > 180.0:
                v -= 360.0
            while (v - ref) < -180.0:
                v += 360.0
            wp[6] = v
            ref = v

    def _sanity_check_all_steps(self, lado: str, waypoints: List[List[float]], current: List[float]):
        """
        Verifica saltos excessivos entre o estado atual e todos os waypoints consecutivos.
        """
        if not waypoints:
            return

        prev = current[:]
        for k, wp in enumerate(waypoints):
            for i in range(7):
                dj = abs(wp[i] - prev[i])
                if dj > self.max_jump_deg:
                    self.get_logger().error(
                        f"Sanity check falhou ({lado}): salto demasiado grande "
                        f"no wp {k}, junta idx={i}: {dj:.1f} deg (> {self.max_jump_deg:.1f})"
                    )
                    raise RuntimeError("Refuse unsafe jump")
            prev = wp[:]

    # ================================================================
    # RWS communication
    # ================================================================

    def _read_joints(self, lado: str) -> Optional[List[float]]:
        """
        Lê 7 "juntas" do RWS em graus, de forma robusta:
        rax_1..rax_6 e eax_a.

        Retorna [rax1, rax2, rax3, rax4, rax5, rax6, eax_a]
        """
        mechunit = "ROB_L" if lado == "L" else "ROB_R"
        try:
            r = self.session.get(f"{self.base_motion}/{mechunit}/jointtarget", timeout=2)
            if r.status_code != 200:
                return None

            html = r.text
            pattern = r'class="(?P<key>rax_[1-6]|eax_a)">(?P<val>[^<]+)<'
            matches = re.findall(pattern, html)

            data = {}
            for k, v in matches:
                try:
                    data[k] = float(v)
                except Exception:
                    pass

            required = [f"rax_{i}" for i in range(1, 7)] + ["eax_a"]
            if any(k not in data for k in required):
                return None

            return [
                data["rax_1"], data["rax_2"], data["rax_3"],
                data["rax_4"], data["rax_5"], data["rax_6"], data["eax_a"]
            ]
        except Exception:
            return None

    def _set_rapid_var(self, task: str, name: str, value: str) -> bool:
        encoded_name = quote(name, safe="")
        url = f"{self.base_rapid}/{task}/{self.MODULE}/{encoded_name}?action=set"
        try:
            r = self.session.post(url, data={"value": str(value)}, timeout=3)
            if r.status_code != 204:
                self.get_logger().error(
                    f"RWS set falhou: task={task} name={name} "
                    f"status={r.status_code} body={r.text[:500]}"
                )
                return False
            return True
        except Exception as e:
            self.get_logger().error(f"RWS exception: task={task} name={name} err={e}")
            return False

    def _pulse_signal(self, signal_name="RUN_RAPID_ROUTINE"):
        try:
            self.session.post(
                f"{self.base_io}/{signal_name}?action=set",
                data={"lvalue": "1"},
                timeout=2,
            )
            time.sleep(SIGNAL_PULSE_DELAY)
            self.session.post(
                f"{self.base_io}/{signal_name}?action=set",
                data={"lvalue": "0"},
                timeout=2,
            )
        except Exception as e:
            self.get_logger().error(f"Erro ao pulsar {signal_name}: {e}")

    def _joints_to_jointtarget(self, j: List[float]) -> str:
        """
        Conforme o teu guia: 6 robax + extax (eax_a).
        """
        return (
            f"[[{j[0]},{j[1]},{j[2]},{j[3]},{j[4]},{j[5]}],"
            f"[{j[6]},9E9,9E9,9E9,9E9,9E9]]"
        )

    def _write_multi_waypoints(self, lado: str, waypoints: List[List[float]]) -> bool:
        task = self.TASK[lado]
        n = len(waypoints)
        if n <= 0:
            return False
        if n > self.max_wp:
            waypoints = waypoints[: self.max_wp]
            n = len(waypoints)

        ok_all = True

        try:
            for i, wp in enumerate(waypoints):
                jt = self._joints_to_jointtarget(wp)
                ok = self._set_rapid_var(task, f"wp_array{{{i+1}}}", jt)
                if not ok:
                    ok_all = False
                    self.get_logger().error(f"Falha a escrever wp_array{{{i+1}}} ({lado}) = {jt}")

            ok_count = self._set_rapid_var(task, "wp_count", str(n))
            ok_speed = self._set_rapid_var(task, "wp_speed", DEFAULT_SPEED)
            ok_routine = self._set_rapid_var(task, "routine_name_input", '"runMoveAbsJMulti"')

            ok_all = ok_all and ok_count and ok_speed and ok_routine

            if ok_all:
                self.get_logger().info(f"MoveAbsJMulti {lado}: {n} waypoints escritos")
            else:
                self.get_logger().error(f"MoveAbsJMulti {lado}: escrita incompleta")

            return ok_all

        except Exception as e:
            self.get_logger().error(f"Erro RWS write {lado}: {e}")
            return False

    def _wait_position_reached(self, rws_targets: Dict[str, List[float]], poll_hz: float = 5.0) -> bool:
        t0 = time.time()
        while (time.time() - t0) < self.settle_timeout:
            all_ok = True
            for lado, target in rws_targets.items():
                current = self._read_joints(lado)
                if current is None:
                    all_ok = False
                    break
                for i in range(7):
                    if abs(current[i] - target[i]) > self.pos_tol:
                        all_ok = False
                        break
                if not all_ok:
                    break

            if all_ok:
                dt = time.time() - t0
                self.get_logger().info(f"Posição atingida em {dt:.1f}s")
                return True

            time.sleep(1.0 / poll_hz)

        return False


def main(args=None):
    rclpy.init(args=args)
    node = RWSTrajectoryController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        node.get_logger().error(f"Abortado por segurança: {e}")
    finally:
        node.session.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()