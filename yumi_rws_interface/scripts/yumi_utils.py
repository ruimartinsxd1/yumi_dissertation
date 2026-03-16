#!/usr/bin/env python3
"""
yumi_utils.py — Funções RWS partilhadas para scripts YuMi.

Centraliza as operações HTTP comuns (leitura de juntas, movimento, gripper,
espera de posição) usadas pelos scripts de teach & replay, evitando duplicação.

Part of the yumi_rws_interface package.
Project: Collaborative Robotics for RCD Pre-Assembly
Author: Rui Martins (up202108756@edu.fe.up.pt)
Supervisors: Prof. Luís F. Rocha, Prof. [co-orientador]
FEUP/INESCTEC, 2026
"""

import re
import time

import requests
from requests.auth import HTTPDigestAuth

# ---------------------------------------------------------------------------
# Configuração padrão do robô
# ---------------------------------------------------------------------------

ROBOT_IP: str = "192.168.125.1"
DEFAULT_USER: str = "Default User"
DEFAULT_PASSWORD: str = "robotics"
MODULE: str = "TRobRAPID"

# Valores operacionais
SIGNAL_PULSE_DELAY: float = 0.1   # seconds — tempo para o controlador detetar o pulso
DEFAULT_SPEED: str = "[1000,500,1000,500]"  # v1000 TCP, z500 zone
SLOW_SPEED: str = "[100,50,100,50]"         # v100 TCP, z50 zone
POLL_INTERVAL: float = 0.2         # seconds entre verificações de posição
WAIT_TIMEOUT: float = 15.0         # seconds — timeout máximo de espera
POSITION_TOLERANCE: float = 2.0    # graus — tolerância para considerar posição atingida


# ---------------------------------------------------------------------------
# Sessão HTTP
# ---------------------------------------------------------------------------

def get_session(
    robot_ip: str = ROBOT_IP,
    user: str = DEFAULT_USER,
    password: str = DEFAULT_PASSWORD,
) -> requests.Session:
    """Cria e devolve uma sessão HTTP com autenticação Digest para o RWS.

    Args:
        robot_ip: Endereço IP do controlador ABB.
        user: Nome de utilizador RWS.
        password: Password RWS.

    Returns:
        Sessão requests configurada com autenticação Digest.
    """
    s = requests.Session()
    s.auth = HTTPDigestAuth(user, password)
    return s


def get_base_url(robot_ip: str = ROBOT_IP) -> str:
    """Devolve a URL base do RWS para o IP fornecido.

    Args:
        robot_ip: Endereço IP do controlador ABB.

    Returns:
        URL base no formato ``http://<robot_ip>``.
    """
    return f"http://{robot_ip}"


# ---------------------------------------------------------------------------
# Leitura de estado
# ---------------------------------------------------------------------------

def read_joints(session: requests.Session, base_url: str, side: str) -> list[float]:
    """Lê as 7 posições de junta de um braço via RWS (em graus).

    Ordem de retorno: [rax_1, rax_2, rax_3, rax_4, rax_5, rax_6, eax_a].

    Args:
        session: Sessão HTTP autenticada.
        base_url: URL base do RWS (ex: ``http://192.168.125.1``).
        side: Braço — ``"R"`` (direito) ou ``"L"`` (esquerdo).

    Returns:
        Lista de 7 ângulos em graus.
    """
    mechunit = f"ROB_{side}"
    r = session.get(
        f"{base_url}/rw/motionsystem/mechunits/{mechunit}/jointtarget",
        timeout=2,
    )
    vals = re.findall(r'class="(?:rax_\d|eax_a)">([^<]+)<', r.text)
    return [float(v) for v in vals[:7]]


def read_cartesian(session: requests.Session, base_url: str, side: str) -> list[float]:
    """Lê a posição cartesiana [x, y, z] em mm de um braço via RWS.

    Args:
        session: Sessão HTTP autenticada.
        base_url: URL base do RWS.
        side: Braço — ``"R"`` ou ``"L"``.

    Returns:
        Lista ``[x, y, z]`` em milímetros.
    """
    mechunit = f"ROB_{side}"
    r = session.get(
        f"{base_url}/rw/motionsystem/mechunits/{mechunit}/robtarget",
        timeout=2,
    )
    x = float(re.search(r'class="x">([^<]+)', r.text).group(1))
    y = float(re.search(r'class="y">([^<]+)', r.text).group(1))
    z = float(re.search(r'class="z">([^<]+)', r.text).group(1))
    return [x, y, z]


# ---------------------------------------------------------------------------
# Controlo de movimento
# ---------------------------------------------------------------------------

def move_to(
    session: requests.Session,
    base_url: str,
    side: str,
    joints: list[float],
    speed: str = DEFAULT_SPEED,
) -> None:
    """Envia um comando MoveAbsJ para o braço especificado via RWS.

    Escreve o jointtarget, a velocidade e o nome da rotina RAPID, depois
    pulsa o sinal digital RUN_RAPID_ROUTINE para desencadear a execução.

    Args:
        session: Sessão HTTP autenticada.
        base_url: URL base do RWS.
        side: Braço — ``"R"`` ou ``"L"``.
        joints: 7 ângulos em graus [rax_1..rax_6, eax_a].
        speed: String RAPID de velocidade, ex: ``"[1000,500,1000,500]"``.
    """
    task = f"T_ROB_{side}"
    j = joints
    jt = (
        f"[[{j[0]},{j[1]},{j[2]},{j[3]},{j[4]},{j[5]}],"
        f"[{j[6]},9E9,9E9,9E9,9E9,9E9]]"
    )
    base_path = f"{base_url}/rw/rapid/symbol/data/RAPID/{task}/{MODULE}"
    session.post(
        f"{base_path}/move_jointtarget_input?action=set",
        data={"value": jt},
        timeout=3,
    )
    session.post(
        f"{base_path}/move_speed_input?action=set",
        data={"value": speed},
        timeout=3,
    )
    session.post(
        f"{base_path}/routine_name_input?action=set",
        data={"value": '"runMoveAbsJ"'},
        timeout=3,
    )
    # Pulsar sinal para desencadear a rotina RAPID
    session.post(
        f"{base_url}/rw/iosystem/signals/RUN_RAPID_ROUTINE?action=set",
        data={"lvalue": "1"},
        timeout=2,
    )
    time.sleep(SIGNAL_PULSE_DELAY)
    session.post(
        f"{base_url}/rw/iosystem/signals/RUN_RAPID_ROUTINE?action=set",
        data={"lvalue": "0"},
        timeout=2,
    )


def gripper_cmd(
    session: requests.Session,
    base_url: str,
    side: str,
    command: int,
) -> None:
    """Envia um comando ao SmartGripper via RWS.

    Escreve o comando na variável RAPID ``command_input`` do módulo TRobSG
    e pulsa o sinal RUN_SG_ROUTINE.

    Comandos TRobSG: 1=init, 2=calibrate, 3=moveTo, 4=close, 5=open.

    Args:
        session: Sessão HTTP autenticada.
        base_url: URL base do RWS.
        side: Braço — ``"R"`` ou ``"L"``.
        command: Código de comando TRobSG (ex: 4=fechar, 5=abrir).
    """
    task = f"T_ROB_{side}"
    session.post(
        f"{base_url}/rw/rapid/symbol/data/RAPID/{task}/TRobSG/command_input?action=set",
        data={"value": str(command)},
        timeout=3,
    )
    session.post(
        f"{base_url}/rw/iosystem/signals/RUN_SG_ROUTINE?action=set",
        data={"lvalue": "1"},
        timeout=2,
    )
    time.sleep(SIGNAL_PULSE_DELAY)
    session.post(
        f"{base_url}/rw/iosystem/signals/RUN_SG_ROUTINE?action=set",
        data={"lvalue": "0"},
        timeout=2,
    )


# ---------------------------------------------------------------------------
# Espera de posição
# ---------------------------------------------------------------------------

def wait_position(
    session: requests.Session,
    base_url: str,
    side: str,
    target: list[float],
    tol: float = POSITION_TOLERANCE,
    timeout: float = WAIT_TIMEOUT,
) -> bool:
    """Aguarda que o braço atinja a posição alvo (polling via RWS).

    Compara cada junta com a tolerância ``tol`` em graus. Retorna quando
    todas as juntas estiverem dentro da tolerância ou quando o timeout
    for atingido.

    Args:
        session: Sessão HTTP autenticada.
        base_url: URL base do RWS.
        side: Braço — ``"R"`` ou ``"L"``.
        target: 7 ângulos alvo em graus.
        tol: Tolerância por junta em graus (padrão: 2.0°).
        timeout: Tempo máximo de espera em segundos (padrão: 15.0 s).

    Returns:
        ``True`` se a posição foi atingida, ``False`` se timeout.
    """
    t0 = time.time()
    while time.time() - t0 < timeout:
        cur = read_joints(session, base_url, side)
        if all(abs(c - t) < tol for c, t in zip(cur, target)):
            return True
        time.sleep(POLL_INTERVAL)
    return False
