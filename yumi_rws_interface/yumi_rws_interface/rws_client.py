#!/usr/bin/env python3
"""
rws_client.py — Cliente HTTP para a API Robot Web Services (RWS) do ABB YuMi.

Encapsula toda a comunicação com o controlador IRC5: leitura de estado,
leitura de posições de juntas, controlo de RAPID, variáveis RAPID e sinais I/O.
Utilizado pelos nós ROS 2 do pacote como camada de acesso ao robô.

Part of the yumi_rws_interface package.
Project: Collaborative Robotics for RCD Pre-Assembly
Author: Rui Martins (up202108756@edu.fe.up.pt)
Supervisors: Prof. Luís F. Rocha, Prof. [co-orientador]
FEUP/INESCTEC, 2026
"""

import re
from typing import Dict, List, Optional

import requests
from requests.auth import HTTPDigestAuth



class YuMiRWSClient:
    """Cliente RWS para comunicação com ABB YuMi."""
    
    def __init__(
        self, 
        ip: str = "192.168.125.1",
        username: str = "Default User",
        password: str = "robotics"
    ):
        self.base_url = f"http://{ip}"
        self.auth = HTTPDigestAuth(username, password)
        self.session = requests.Session()
        self.session.headers.update({
            'Accept': 'application/xhtml+xml;v=2.0',
            'Content-Type': 'application/x-www-form-urlencoded'
        })
        self._mastership = False
    
    # =========================================================================
    # Métodos de diagnóstico
    # =========================================================================
    
    def discover_endpoints(self) -> Dict[str, dict]:
        """Testa vários endpoints para descobrir quais funcionam."""
        endpoints = [
            "/", "/rw", "/rw/system", "/rw/system/robotware",
            "/rw/panel", "/rw/panel/ctrlstate", "/rw/panel/opmode",
            "/rw/rapid", "/rw/rapid/tasks", "/rw/rapid/execution",
            "/rw/motionsystem", "/rw/motionsystem/mechunits",
            "/rw/iosystem", "/rw/iosystem/signals",
            "/ctrl", "/ctrl/identity",
        ]
        
        results = {}
        for endpoint in endpoints:
            try:
                r = self.session.get(
                    f"{self.base_url}{endpoint}",
                    auth=self.auth,
                    timeout=3
                )
                results[endpoint] = {
                    'status': r.status_code,
                    'ok': r.status_code == 200,
                    'content_type': r.headers.get('Content-Type', 'unknown'),
                    'sample': r.text[:200] if r.status_code == 200 else r.text[:100]
                }
            except Exception as e:
                results[endpoint] = {
                    'status': 'error',
                    'ok': False,
                    'error': str(e)
                }
        
        return results
    
    def print_endpoint_discovery(self):
        """Imprime resultados da descoberta de endpoints."""
        print("\n" + "="*60)
        print("DESCOBERTA DE ENDPOINTS RWS")
        print("="*60)
        
        results = self.discover_endpoints()
        
        for endpoint, info in results.items():
            if info['ok']:
                print(f"✅ {endpoint}")
                print(f"   Content-Type: {info.get('content_type', 'N/A')}")
            else:
                print(f"❌ {endpoint} - Status: {info.get('status', 'error')}")
        
        print("\n" + "="*60)
    
    # =========================================================================
    # Métodos de conexão e estado
    # =========================================================================
    
    def is_connected(self) -> bool:
        """Verifica se consegue comunicar com o controlador."""
        try:
            for endpoint in ["/rw", "/rw/system", "/rw/panel"]:
                r = self.session.get(
                    f"{self.base_url}{endpoint}",
                    auth=self.auth,
                    timeout=2
                )
                if r.status_code == 200:
                    return True
            return False
        except:
            return False
    
    def get_controller_state(self) -> Optional[str]:
        """Obtém estado do controlador."""
        try:
            r = self.session.get(
                f"{self.base_url}/rw/panel/ctrlstate",
                auth=self.auth,
                timeout=5
            )
            if r.status_code == 200:
                # Procurar estado no XML
                match = re.search(r'ctrlstate["\s>]+([^<"]+)', r.text, re.IGNORECASE)
                if match:
                    return match.group(1).strip()
                
                # Procurar palavras-chave conhecidas
                text_lower = r.text.lower()
                for state in ['motoron', 'motoroff', 'init', 'guardstop', 'emergencystop', 'sysfail']:
                    if state in text_lower:
                        return state
            return None
        except Exception as e:
            print(f"Erro ao obter controller state: {e}")
            return None
    
    def get_operation_mode(self) -> Optional[str]:
        """Obtém modo de operação (MANR, MANF, AUTO)."""
        try:
            r = self.session.get(
                f"{self.base_url}/rw/panel/opmode",
                auth=self.auth,
                timeout=5
            )
            if r.status_code == 200:
                text_upper = r.text.upper()
                for mode in ['AUTO', 'MANR', 'MANF', 'MANUAL']:
                    if mode in text_upper:
                        return mode
                
                match = re.search(r'opmode["\s>:]+([^<",\s]+)', r.text, re.IGNORECASE)
                if match:
                    return match.group(1).strip().upper()
            return None
        except Exception as e:
            print(f"Erro ao obter operation mode: {e}")
            return None
    
    def get_rapid_execution_state(self) -> Optional[str]:
        """Obtém estado de execução RAPID."""
        try:
            r = self.session.get(
                f"{self.base_url}/rw/rapid/execution",
                auth=self.auth,
                timeout=5
            )
            if r.status_code == 200:
                text_lower = r.text.lower()
                if 'running' in text_lower:
                    return 'running'
                elif 'stopped' in text_lower:
                    return 'stopped'
            return None
        except Exception as e:
            print(f"Erro ao obter RAPID execution state: {e}")
            return None
    
    # =========================================================================
    # Métodos de leitura de posição
    # =========================================================================
    
    def get_mechanical_units(self) -> Optional[List[str]]:
        """Obtém lista de unidades mecânicas disponíveis."""
        try:
            r = self.session.get(
                f"{self.base_url}/rw/motionsystem/mechunits",
                auth=self.auth,
                timeout=5
            )
            if r.status_code == 200:
                matches = re.findall(r'(ROB_[LR1-9]|rob_[lr1-9])', r.text, re.IGNORECASE)
                if matches:
                    return list(set([m.upper() for m in matches]))
                
                matches = re.findall(r'mechunits/([^/"<>\s]+)', r.text)
                if matches:
                    return list(set(matches))
            return None
        except Exception as e:
            print(f"Erro ao obter mechanical units: {e}")
            return None
    
    def get_joint_positions(self, mechunit: str = "ROB_L") -> Optional[List[float]]:
        """
        Obtém posições dos joints de uma unidade mecânica.
        
        Args:
            mechunit: 'ROB_L' para braço esquerdo, 'ROB_R' para direito
            
        Returns:
            Lista com 7 valores de joint (graus) ou None
        """
        try:
            r = self.session.get(
                f"{self.base_url}/rw/motionsystem/mechunits/{mechunit}/jointtarget",
                auth=self.auth,
                timeout=5
            )
            if r.status_code == 200:
                joints = []
                
                # YuMi format: rax_1 to rax_6, then eax_a for joint 7
                # Pattern: <span class="rax_1">value</span>
                for i in range(1, 7):
                    match = re.search(rf'class="rax_{i}">([^<]+)<', r.text)
                    if match:
                        joints.append(float(match.group(1)))
                
                # Joint 7 is eax_a
                match = re.search(r'class="eax_a">([^<]+)<', r.text)
                if match:
                    val = float(match.group(1))
                    # Check if it's a valid value (not 8999999488)
                    if val < 1000000:
                        joints.append(val)
                
                if len(joints) >= 6:
                    return joints
                    
            return None
        except Exception as e:
            print(f"Erro ao obter joint positions para {mechunit}: {e}")
            return None
    
    def get_cartesian_position(self, mechunit: str = "ROB_L") -> Optional[Dict]:
        """Obtém posição cartesiana do TCP."""
        try:
            r = self.session.get(
                f"{self.base_url}/rw/motionsystem/mechunits/{mechunit}/robtarget",
                auth=self.auth,
                timeout=5
            )
            if r.status_code == 200:
                result = {}
                
                for coord in ['x', 'y', 'z']:
                    match = re.search(rf'class="{coord}">([^<]+)<', r.text, re.IGNORECASE)
                    if match:
                        result[coord] = float(match.group(1))
                
                for q in ['q1', 'q2', 'q3', 'q4']:
                    match = re.search(rf'class="{q}">([^<]+)<', r.text, re.IGNORECASE)
                    if match:
                        result[q] = float(match.group(1))
                
                if len(result) >= 3:
                    return result
            return None
        except Exception as e:
            print(f"Erro ao obter cartesian position: {e}")
            return None
    
    # =========================================================================
    # Métodos de controlo (requerem mastership)
    # =========================================================================
    
    def request_mastership(self, domain: str = "rapid") -> bool:
        """Solicita mastership para controlar o robot."""
        try:
            r = self.session.post(
                f"{self.base_url}/rw/{domain}/mastership/request",
                auth=self.auth,
                timeout=5
            )
            if r.status_code in [200, 204]:
                self._mastership = True
                return True
            
            r = self.session.post(
                f"{self.base_url}/rw/{domain}/mastership?action=request",
                auth=self.auth,
                timeout=5
            )
            self._mastership = r.status_code in [200, 204]
            return self._mastership
        except Exception as e:
            print(f"Erro ao solicitar mastership: {e}")
            return False
    
    def release_mastership(self, domain: str = "rapid") -> bool:
        """Liberta mastership."""
        try:
            r = self.session.post(
                f"{self.base_url}/rw/{domain}/mastership/release",
                auth=self.auth,
                timeout=5
            )
            if r.status_code in [200, 204]:
                self._mastership = False
                return True
            
            r = self.session.post(
                f"{self.base_url}/rw/{domain}/mastership?action=release",
                auth=self.auth,
                timeout=5
            )
            if r.status_code in [200, 204]:
                self._mastership = False
                return True
            return False
        except Exception as e:
            print(f"Erro ao libertar mastership: {e}")
            return False
    
    def motors_on(self) -> bool:
        """Liga os motores (requer modo Auto e mastership)."""
        try:
            r = self.session.post(
                f"{self.base_url}/rw/panel/ctrlstate?action=setctrlstate",
                auth=self.auth,
                data={"ctrl-state": "motoron"},
                timeout=5
            )
            return r.status_code in [200, 204]
        except Exception as e:
            print(f"Erro ao ligar motores: {e}")
            return False
    
    def motors_off(self) -> bool:
        """Desliga os motores."""
        try:
            r = self.session.post(
                f"{self.base_url}/rw/panel/ctrlstate?action=setctrlstate",
                auth=self.auth,
                data={"ctrl-state": "motoroff"},
                timeout=5
            )
            return r.status_code in [200, 204]
        except Exception as e:
            print(f"Erro ao desligar motores: {e}")
            return False
    
    # =========================================================================
    # Métodos RAPID
    # =========================================================================
    
    def get_rapid_tasks(self) -> Optional[List[str]]:
        """Obtém lista de tasks RAPID."""
        try:
            r = self.session.get(
                f"{self.base_url}/rw/rapid/tasks",
                auth=self.auth,
                timeout=5
            )
            if r.status_code == 200:
                matches = re.findall(r'(T_ROB_[LR1-9]|t_rob_[lr1-9])', r.text, re.IGNORECASE)
                if matches:
                    return list(set([m.upper() for m in matches]))
                
                matches = re.findall(r'tasks/([^/"<>\s]+)', r.text)
                if matches:
                    return list(set(matches))
            return None
        except Exception as e:
            print(f"Erro ao obter RAPID tasks: {e}")
            return None
    
    def start_rapid(self) -> bool:
        """Inicia execução do programa RAPID."""
        try:
            r = self.session.post(
                f"{self.base_url}/rw/rapid/execution?action=start",
                auth=self.auth,
                data={
                    "regain": "continue",
                    "execmode": "continue",
                    "cycle": "once",
                    "condition": "none",
                    "stopatbp": "disabled",
                    "alltaskbytsp": "false"
                },
                timeout=5
            )
            return r.status_code in [200, 204]
        except Exception as e:
            print(f"Erro ao iniciar RAPID: {e}")
            return False
    
    def stop_rapid(self) -> bool:
        """Para execução do programa RAPID."""
        try:
            r = self.session.post(
                f"{self.base_url}/rw/rapid/execution?action=stop",
                auth=self.auth,
                data={"stopmode": "stop"},
                timeout=5
            )
            return r.status_code in [200, 204]
        except Exception as e:
            print(f"Erro ao parar RAPID: {e}")
            return False
    
    def reset_program_pointer(self) -> bool:
        """Reset do program pointer para main."""
        try:
            r = self.session.post(
                f"{self.base_url}/rw/rapid/execution?action=resetpp",
                auth=self.auth,
                timeout=5
            )
            return r.status_code in [200, 204]
        except Exception as e:
            print(f"Erro ao resetar PP: {e}")
            return False
    
    # =========================================================================
    # Métodos para variáveis RAPID
    # =========================================================================
    
    def get_rapid_variable(self, task: str, module: str, variable: str) -> Optional[str]:
        """Lê valor de uma variável RAPID."""
        try:
            r = self.session.get(
                f"{self.base_url}/rw/rapid/symbol/data/RAPID/{task}/{module}/{variable}",
                auth=self.auth,
                timeout=5
            )
            if r.status_code == 200:
                match = re.search(r'value["\s>:]+([^<"]+)', r.text, re.IGNORECASE)
                if match:
                    return match.group(1).strip()
            return None
        except Exception as e:
            print(f"Erro ao ler variável RAPID: {e}")
            return None
    
    def set_rapid_variable(self, task: str, module: str, variable: str, value: str) -> bool:
        """Escreve valor numa variável RAPID."""
        try:
            r = self.session.post(
                f"{self.base_url}/rw/rapid/symbol/data/RAPID/{task}/{module}/{variable}?action=set",
                auth=self.auth,
                data={"value": value},
                timeout=5
            )
            return r.status_code in [200, 204]
        except Exception as e:
            print(f"Erro ao escrever variável RAPID: {e}")
            return False
    
    def set_robtarget(
        self, task: str, module: str, variable: str,
        x: float, y: float, z: float,
        q1: float = 0.0, q2: float = 1.0, q3: float = 0.0, q4: float = 0.0,
        cf1: int = 0, cf4: int = 0, cf6: int = 0, cfx: int = 0
    ) -> bool:
        """Define um robtarget em RAPID."""
        value = f"[[{x},{y},{z}],[{q1},{q2},{q3},{q4}],[{cf1},{cf4},{cf6},{cfx}],[9E9,9E9,9E9,9E9,9E9,9E9]]"
        return self.set_rapid_variable(task, module, variable, value)
    
    # =========================================================================
    # Métodos de I/O
    # =========================================================================
    
    def get_io_signal(self, signal_name: str) -> Optional[int]:
        """Lê valor de um sinal I/O."""
        try:
            r = self.session.get(
                f"{self.base_url}/rw/iosystem/signals/{signal_name}",
                auth=self.auth,
                timeout=5
            )
            if r.status_code == 200:
                match = re.search(r'lvalue["\s>:]+(\d+)', r.text, re.IGNORECASE)
                if match:
                    return int(match.group(1))
            return None
        except Exception as e:
            print(f"Erro ao ler I/O: {e}")
            return None
    
    def set_io_signal(self, signal_name: str, value: int) -> bool:
        """Define valor de um sinal I/O."""
        try:
            r = self.session.post(
                f"{self.base_url}/rw/iosystem/signals/{signal_name}?action=set",
                auth=self.auth,
                data={"lvalue": str(value)},
                timeout=5
            )
            return r.status_code in [200, 204]
        except Exception as e:
            print(f"Erro ao escrever I/O: {e}")
            return False


# =============================================================================
# Função de teste standalone
# =============================================================================

def main():
    """Função de teste - pode ser executada diretamente."""
    import sys
    
    ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.125.1"
    
    print(f"\n{'='*60}")
    print("TESTE DO CLIENTE RWS - YuMi")
    print(f"IP: {ip}")
    print(f"{'='*60}\n")
    
    client = YuMiRWSClient(ip=ip)
    
    # Descobrir endpoints
    print("1. Descobrindo endpoints disponíveis...")
    client.print_endpoint_discovery()
    
    # Testar conexão
    print("\n2. Testando conexão...")
    if client.is_connected():
        print("✅ Conectado ao YuMi!")
    else:
        print("❌ Não foi possível conectar")
        return
    
    # Obter informações
    print("\n3. Obtendo informações do controlador...")
    state = client.get_controller_state()
    print(f"   Controller state: {state}")
    
    mode = client.get_operation_mode()
    print(f"   Operation mode: {mode}")
    
    rapid_state = client.get_rapid_execution_state()
    print(f"   RAPID execution: {rapid_state}")
    
    # Mechanical units
    print("\n4. Obtendo unidades mecânicas...")
    mechunits = client.get_mechanical_units()
    print(f"   Mechanical units: {mechunits}")
    
    # RAPID tasks
    print("\n5. Obtendo tasks RAPID...")
    tasks = client.get_rapid_tasks()
    print(f"   RAPID tasks: {tasks}")
    
    # Joint positions
    print("\n6. Obtendo posições dos joints...")
    for mu in ['ROB_L', 'ROB_R']:
        joints = client.get_joint_positions(mu)
        print(f"   {mu} joints: {joints}")
    
    print(f"\n{'='*60}")
    print("TESTE COMPLETO")
    print(f"{'='*60}\n")


if __name__ == "__main__":
    main()
