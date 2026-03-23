"""
yumi_egm_interface — ROS 2 package for ABB YuMi IRB 14000 real-time control via EGM.

Provides EGM UDP communication manager and 250 Hz trajectory controller for MoveIt2.

Part of the yumi_dissertation project.
Project: Collaborative Robotics for RCD Pre-Assembly
Author: Rui Martins (up202108756@edu.fe.up.pt)
Supervisors: Prof. Luís F. Rocha, Prof. [co-orientador]
FEUP/INESCTEC, 2026
"""

from .egm_manager import EGMManager, EGMConfig, EGMArmChannel

__all__ = [
    'EGMManager',
    'EGMConfig',
    'EGMArmChannel',
]

__version__ = '0.1.0'
__author__ = 'Rui Martins'
