"""
yumi_rws_interface — Pacote ROS 2 para controlo do ABB YuMi IRB 14000 via RWS.

Fornece nós ROS 2 para publicação de joint states, controlo de trajetórias
MoveIt2, controlo de grippers SmartGripper e serviços de estado do controlador.

Part of the yumi_rws_interface package.
Project: Collaborative Robotics for RCD Pre-Assembly
Author: Rui Martins (up202108756@edu.fe.up.pt)
Supervisors: Prof. Luís F. Rocha, Prof. [co-orientador]
FEUP/INESCTEC, 2026
"""

from .rws_client import YuMiRWSClient
from .joint_state_publisher import YuMiJointStatePublisher
from .rws_commander import YuMiRWSCommander

__all__ = [
    'YuMiRWSClient',
    'YuMiJointStatePublisher', 
    'YuMiRWSCommander',
]

__version__ = '0.1.0'
__author__ = 'Rui Martins'
