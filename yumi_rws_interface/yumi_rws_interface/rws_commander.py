#!/usr/bin/env python3
"""
rws_commander.py — Nó ROS 2 que expõe serviços para comandar o YuMi via RWS.

Publica o estado do controlador e fornece serviços para controlo de alto nível:
arranque/paragem de RAPID, reset do program pointer, motores ON/OFF e diagnóstico.

Serviços disponíveis:
  /yumi/start_rapid            — Inicia execução RAPID
  /yumi/stop_rapid             — Para execução RAPID
  /yumi/reset_program_pointer  — Reset PP to main
  /yumi/set_motors             — Liga/desliga motores (requer modo AUTO)
  /yumi/get_state              — Obtém estado completo do controlador
  /yumi/test_connection        — Testa conectividade com o robô

Part of the yumi_rws_interface package.
Project: Collaborative Robotics for RCD Pre-Assembly
Author: Rui Martins (up202108756@edu.fe.up.pt)
Supervisors: Prof. Luís F. Rocha, Prof. [co-orientador]
FEUP/INESCTEC, 2026
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String

try:
    from .rws_client import YuMiRWSClient
except ImportError:
    from rws_client import YuMiRWSClient


class YuMiRWSCommander(Node):
    """Nó que expõe serviços ROS 2 para controlar o YuMi via RWS."""
    
    def __init__(self):
        super().__init__('yumi_rws_commander')
        
        # Parâmetros
        self.declare_parameter('robot_ip', '192.168.125.1')
        self.declare_parameter('state_publish_rate', 1.0)  # Hz
        
        robot_ip = self.get_parameter('robot_ip').value
        state_rate = self.get_parameter('state_publish_rate').value
        
        # Cliente RWS
        self.rws_client = YuMiRWSClient(ip=robot_ip)
        
        # === Serviços ===
        
        # RAPID control
        self.srv_start_rapid = self.create_service(
            Trigger, 'yumi/start_rapid', self.start_rapid_callback
        )
        self.srv_stop_rapid = self.create_service(
            Trigger, 'yumi/stop_rapid', self.stop_rapid_callback
        )
        self.srv_reset_pp = self.create_service(
            Trigger, 'yumi/reset_program_pointer', self.reset_pp_callback
        )
        
        # Motor control
        self.srv_motors = self.create_service(
            SetBool, 'yumi/set_motors', self.set_motors_callback
        )
        
        # State query
        self.srv_get_state = self.create_service(
            Trigger, 'yumi/get_state', self.get_state_callback
        )
        
        # Connection test
        self.srv_test_connection = self.create_service(
            Trigger, 'yumi/test_connection', self.test_connection_callback
        )
        
        # === Publishers ===
        self.state_pub = self.create_publisher(String, 'yumi/controller_state', 10)
        
        # Timer para publicar estado periodicamente
        if state_rate > 0:
            self.create_timer(1.0 / state_rate, self.publish_state)
        
        # Log startup
        self.get_logger().info('='*50)
        self.get_logger().info('YuMi RWS Commander')
        self.get_logger().info(f'  Robot IP: {robot_ip}')
        self.get_logger().info('  Services:')
        self.get_logger().info('    - /yumi/start_rapid')
        self.get_logger().info('    - /yumi/stop_rapid')
        self.get_logger().info('    - /yumi/reset_program_pointer')
        self.get_logger().info('    - /yumi/set_motors')
        self.get_logger().info('    - /yumi/get_state')
        self.get_logger().info('    - /yumi/test_connection')
        self.get_logger().info('='*50)
    
    # =========================================================================
    # Publisher callback
    # =========================================================================
    
    def publish_state(self):
        """Publica estado do controlador periodicamente."""
        try:
            state = self.rws_client.get_controller_state() or 'unknown'
            mode = self.rws_client.get_operation_mode() or 'unknown'
            rapid = self.rws_client.get_rapid_execution_state() or 'unknown'
            
            msg = String()
            msg.data = f"ctrl:{state}|mode:{mode}|rapid:{rapid}"
            self.state_pub.publish(msg)
        except Exception as e:
            self.get_logger().debug(f'Error publishing state: {e}')
    
    # =========================================================================
    # Service callbacks
    # =========================================================================
    
    def test_connection_callback(self, request, response):
        """Serviço para testar conexão ao YuMi."""
        self.get_logger().info('Testing connection to YuMi...')
        
        if self.rws_client.is_connected():
            state = self.rws_client.get_controller_state()
            mode = self.rws_client.get_operation_mode()
            response.success = True
            response.message = f"Connected! State: {state}, Mode: {mode}"
            self.get_logger().info(f'✅ {response.message}')
        else:
            response.success = False
            response.message = "Failed to connect to YuMi controller"
            self.get_logger().error(f'❌ {response.message}')
        
        return response
    
    def start_rapid_callback(self, request, response):
        """Serviço para iniciar execução RAPID."""
        self.get_logger().info('Requesting RAPID start...')
        
        # Obter mastership primeiro
        if not self.rws_client.request_mastership():
            response.success = False
            response.message = "Failed to get mastership. Check if mode is AUTO and no one else has control."
            self.get_logger().error(response.message)
            return response
        
        try:
            if self.rws_client.start_rapid():
                response.success = True
                response.message = "RAPID execution started"
                self.get_logger().info(f'✅ {response.message}')
            else:
                response.success = False
                response.message = "Failed to start RAPID execution"
                self.get_logger().error(response.message)
        finally:
            self.rws_client.release_mastership()
        
        return response
    
    def stop_rapid_callback(self, request, response):
        """Serviço para parar execução RAPID."""
        self.get_logger().info('Requesting RAPID stop...')
        
        if not self.rws_client.request_mastership():
            response.success = False
            response.message = "Failed to get mastership"
            self.get_logger().error(response.message)
            return response
        
        try:
            if self.rws_client.stop_rapid():
                response.success = True
                response.message = "RAPID execution stopped"
                self.get_logger().info(f'✅ {response.message}')
            else:
                response.success = False
                response.message = "Failed to stop RAPID execution"
                self.get_logger().error(response.message)
        finally:
            self.rws_client.release_mastership()
        
        return response
    
    def reset_pp_callback(self, request, response):
        """Serviço para reset do program pointer."""
        self.get_logger().info('Requesting PP reset to main...')
        
        if not self.rws_client.request_mastership():
            response.success = False
            response.message = "Failed to get mastership"
            self.get_logger().error(response.message)
            return response
        
        try:
            if self.rws_client.reset_program_pointer():
                response.success = True
                response.message = "Program pointer reset to main"
                self.get_logger().info(f'✅ {response.message}')
            else:
                response.success = False
                response.message = "Failed to reset program pointer"
                self.get_logger().error(response.message)
        finally:
            self.rws_client.release_mastership()
        
        return response
    
    def set_motors_callback(self, request, response):
        """Serviço para ligar/desligar motores."""
        action = "ON" if request.data else "OFF"
        self.get_logger().info(f'Requesting motors {action}...')
        
        # Verificar modo de operação
        mode = self.rws_client.get_operation_mode()
        if request.data and mode != 'AUTO':
            response.success = False
            response.message = f"Cannot turn motors ON in {mode} mode. Switch to AUTO mode first."
            self.get_logger().warn(response.message)
            return response
        
        if not self.rws_client.request_mastership():
            response.success = False
            response.message = "Failed to get mastership"
            self.get_logger().error(response.message)
            return response
        
        try:
            if request.data:
                success = self.rws_client.motors_on()
            else:
                success = self.rws_client.motors_off()
            
            if success:
                response.success = True
                response.message = f"Motors turned {action}"
                self.get_logger().info(f'✅ {response.message}')
            else:
                response.success = False
                response.message = f"Failed to turn motors {action}"
                self.get_logger().error(response.message)
        finally:
            self.rws_client.release_mastership()
        
        return response
    
    def get_state_callback(self, request, response):
        """Serviço para obter estado completo do controlador."""
        self.get_logger().info('Getting controller state...')
        
        try:
            state = self.rws_client.get_controller_state()
            mode = self.rws_client.get_operation_mode()
            rapid = self.rws_client.get_rapid_execution_state()
            mechunits = self.rws_client.get_mechanical_units()
            tasks = self.rws_client.get_rapid_tasks()
            
            response.success = True
            response.message = (
                f"Controller: {state}\n"
                f"Mode: {mode}\n"
                f"RAPID: {rapid}\n"
                f"Mechunits: {mechunits}\n"
                f"Tasks: {tasks}"
            )
            self.get_logger().info(f'State retrieved:\n{response.message}')
        except Exception as e:
            response.success = False
            response.message = f"Error getting state: {e}"
            self.get_logger().error(response.message)
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    node = YuMiRWSCommander()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
