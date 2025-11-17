#!/usr/bin/env python3
"""
TF Virtual Camera Node
Subscribes to /tf topics and outputs link positions to virtual camera
Supports 2 robots with 30 links each (60 total)
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np
import csv
import os

try:
    import pyvirtualcam
    VIRTUAL_CAM_AVAILABLE = True
except ImportError:
    VIRTUAL_CAM_AVAILABLE = False

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

import socket
import struct

from .shadermotion_encoder import ShaderMotionEncoder


class TfVirtualCameraNode(Node):
    """Node to convert TF data to virtual camera output"""

    def __init__(self):
        super().__init__('tf_virtual_camera_node')

        # Parameters
        self.declare_parameter('link_csv_file', '')
        self.declare_parameter('robot1_name', 'KHR3_1')
        self.declare_parameter('robot2_name', 'KHR3_2')
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('update_rate', 30.0)
        self.declare_parameter('grid_width', 80)
        self.declare_parameter('grid_height', 45)
        self.declare_parameter('enable_virtual_cam', True)
        self.declare_parameter('enable_network_stream', True)
        self.declare_parameter('stream_host', 'auto')  # 'auto' for Windows host detection
        self.declare_parameter('stream_port', 5000)
        self.declare_parameter('show_preview', False)  # Show cv2 preview window

        # Get parameters
        csv_file = self.get_parameter('link_csv_file').get_parameter_value().string_value
        self.robot1_name = self.get_parameter('robot1_name').get_parameter_value().string_value
        self.robot2_name = self.get_parameter('robot2_name').get_parameter_value().string_value
        self.reference_frame = self.get_parameter('reference_frame').get_parameter_value().string_value
        update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        grid_width = self.get_parameter('grid_width').get_parameter_value().integer_value
        grid_height = self.get_parameter('grid_height').get_parameter_value().integer_value
        enable_cam = self.get_parameter('enable_virtual_cam').get_parameter_value().bool_value
        enable_stream = self.get_parameter('enable_network_stream').get_parameter_value().bool_value
        stream_host = self.get_parameter('stream_host').get_parameter_value().string_value
        stream_port = self.get_parameter('stream_port').get_parameter_value().integer_value
        self.show_preview = self.get_parameter('show_preview').get_parameter_value().bool_value

        # Load link names from CSV
        self.robot1_links, self.robot2_links = self._load_links_from_csv(csv_file)

        self.get_logger().info(f'Robot 1 ({self.robot1_name}): {len(self.robot1_links)} links')
        self.get_logger().info(f'Robot 2 ({self.robot2_name}): {len(self.robot2_links)} links')

        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Encoder setup
        self.encoder = ShaderMotionEncoder(grid_width=grid_width, grid_height=grid_height)
        self.seq = 0
        self.width = grid_width * 8
        self.height = grid_height * 8

        # Virtual camera setup
        self.cam = None
        if VIRTUAL_CAM_AVAILABLE and enable_cam:
            try:
                fps = int(update_rate)
                self.cam = pyvirtualcam.Camera(width=self.width, height=self.height, fps=fps, 
                                               fmt=pyvirtualcam.PixelFormat.RGB)
                self.get_logger().info(f'Virtual camera started: {self.cam.device}')
            except Exception as e:
                self.get_logger().warn(f'Failed to start virtual camera: {e}')
        elif not VIRTUAL_CAM_AVAILABLE:
            self.get_logger().warn('pyvirtualcam not installed. Install with: pip install pyvirtualcam')

        # Network stream setup (UDP for low latency)
        self.stream_socket = None
        if enable_stream:
            try:
                # Detect Windows host IP in WSL2 environment
                if stream_host == 'auto':
                    stream_host = self._detect_windows_host()
                    self.get_logger().info(f'Auto-detected Windows host: {stream_host}')
                
                self.stream_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.stream_host = stream_host
                self.stream_port = stream_port
                self.get_logger().info(f'Network stream enabled: {stream_host}:{stream_port} (UDP)')
            except Exception as e:
                self.get_logger().error(f'Failed to create UDP socket: {e}')
                self.stream_socket = None

        # Timer for periodic updates
        timer_period = 1.0 / update_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Preview window flag
        if self.show_preview and not CV2_AVAILABLE:
            self.get_logger().warn('opencv-python not installed. Preview disabled.')
            self.get_logger().warn('Install with: pip install opencv-python')
            self.show_preview = False

        self.get_logger().info('TF Virtual Camera Node started')

    def _load_links_from_csv(self, csv_file):
        """
        Load link names from CSV file
        
        CSV format:
        robot1_link1,robot2_link1
        robot1_link2,robot2_link2
        ...
        
        Returns:
            (robot1_links, robot2_links) - lists of link names (max 30 each)
        """
        robot1_links = []
        robot2_links = []

        if not csv_file or not os.path.exists(csv_file):
            self.get_logger().warn(f'CSV file not found: {csv_file}. Using default links.')
            # Default: base_footprint only
            robot1_links = [f'{self.robot1_name}_base_footprint']
            robot2_links = [f'{self.robot2_name}_base_footprint']
        else:
            try:
                with open(csv_file, 'r') as f:
                    reader = csv.reader(f)
                    for row in reader:
                        # Skip empty rows and comment lines
                        if not row or (len(row) > 0 and row[0].strip().startswith('#')):
                            continue
                        
                        if len(row) >= 2:
                            robot1_links.append(row[0].strip())
                            robot2_links.append(row[1].strip())
                        elif len(row) == 1:
                            robot1_links.append(row[0].strip())
                
                self.get_logger().info(f'Loaded {len(robot1_links)} links from {csv_file}')
            except Exception as e:
                self.get_logger().error(f'Failed to read CSV file: {e}')
                robot1_links = [f'{self.robot1_name}_base_footprint']
                robot2_links = [f'{self.robot2_name}_base_footprint']

        # Limit to 30 links per robot
        robot1_links = robot1_links[:30]
        robot2_links = robot2_links[:30]

        return robot1_links, robot2_links

    def _get_transform(self, target_frame, source_frame):
        """
        Get transform from source_frame to target_frame
        
        Returns:
            (position, quaternion) or None if failed
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            trans = transform.transform.translation
            rot = transform.transform.rotation
            
            position = np.array([trans.x, trans.y, trans.z])
            quaternion = np.array([rot.x, rot.y, rot.z, rot.w])
            
            return position, quaternion
            
        except TransformException as e:
            # self.get_logger().debug(f'TF lookup failed for {source_frame}: {e}')
            return None

    def timer_callback(self):
        """Periodic callback to update virtual camera"""
        
        # Collect transforms for 60 links (30 per robot × 2 robots)
        positions = np.zeros((60, 3))
        rotations = np.zeros((60, 4))
        
        # Robot 1 (links 0-29)
        for i, link_name in enumerate(self.robot1_links):
            result = self._get_transform(self.reference_frame, link_name)
            if result:
                positions[i], rotations[i] = result
                #self.get_logger().info(f'{i}: Link {link_name} position: {positions[i]}, rotation: {rotations[i]}')
            else:
                # Padding with (-4.5, -4.5, -4.5) and identity quaternion
                positions[i] = np.array([-4.5, -4.5, -4.5])
                rotations[i] = np.array([0.0, 0.0, 0.0, 1.0])
        
        # Pad remaining slots for robot 1 (if less than 30 links)
        for i in range(len(self.robot1_links), 30):
            positions[i] = np.array([-4.5, -4.5, -4.5])
            rotations[i] = np.array([0.0, 0.0, 0.0, 1.0])
        
        # Robot 2 (links 30-59)
        for i, link_name in enumerate(self.robot2_links):
            idx = 30 + i
            result = self._get_transform(self.reference_frame, link_name)
            if result:
                positions[idx], rotations[idx] = result
                #self.get_logger().info(f'{i}: Link {link_name} position: {positions[idx]}, rotation: {rotations[idx]}')
            else:
                positions[idx] = np.array([-4.5, -4.5, -4.5])
                rotations[idx] = np.array([0.0, 0.0, 0.0, 1.0])
        
        # Pad remaining slots for robot 2
        for i in range(len(self.robot2_links), 30):
            idx = 30 + i
            positions[idx] = np.array([-4.5, -4.5, -4.5])
            rotations[idx] = np.array([0.0, 0.0, 0.0, 1.0])
        
        # Encode to ShaderMotion format
        grid = self.encoder.encode_frame(positions, rotations, seq=self.seq)
        grid_uint8 = (grid * 255).astype(np.uint8)
        
        # Show preview window
        if self.show_preview and CV2_AVAILABLE:
            # Convert RGB to BGR for OpenCV
            grid_bgr = cv2.cvtColor(grid_uint8, cv2.COLOR_RGB2BGR)
            cv2.imshow('TF Virtual Camera Preview', grid_bgr)
            cv2.waitKey(1)
        
        # Send to virtual camera
        if self.cam is not None:
            try:
                self.cam.send(grid_uint8)
            except Exception as e:
                self.get_logger().error(f'Failed to send frame to virtual camera: {e}')
        
        # Send to network stream
        if self.stream_socket is not None:
            try:
                self._send_frame_udp(grid_uint8)
            except Exception as e:
                self.get_logger().error(f'Failed to send network stream: {e}')
        
        self.seq = (self.seq + 1) & 0xFFFF
        
        # Log status periodically
        if self.seq % 300 == 0:  # Every 10 seconds at 30 FPS
            self.get_logger().info(f'Frame {self.seq}: Encoding {len(self.robot1_links)}+{len(self.robot2_links)} links')

    def _send_frame_udp(self, frame: np.ndarray):
        """
        Send frame via UDP in chunks
        Frame format: [header][chunk_data]
        Header: seq(2) + chunk_id(2) + total_chunks(2) + width(2) + height(2) = 10 bytes
        """
        # Maximum UDP payload size (safe value to avoid fragmentation)
        MAX_CHUNK_SIZE = 60000  # ~60KB per chunk
        
        # Frame data
        frame_data = frame.tobytes()
        total_size = len(frame_data)
        total_chunks = (total_size + MAX_CHUNK_SIZE - 1) // MAX_CHUNK_SIZE
        
        # Send chunks
        for chunk_id in range(total_chunks):
            start_idx = chunk_id * MAX_CHUNK_SIZE
            end_idx = min(start_idx + MAX_CHUNK_SIZE, total_size)
            chunk_data = frame_data[start_idx:end_idx]
            
            # Create header
            header = struct.pack('!HHHHH', 
                                self.seq & 0xFFFF,
                                chunk_id,
                                total_chunks,
                                self.width,
                                self.height)
            
            # Send packet
            packet = header + chunk_data
            self.stream_socket.sendto(packet, (self.stream_host, self.stream_port))

    def _detect_windows_host(self) -> str:
        """
        Detect Windows host IP address in WSL2 environment
        Tries multiple methods:
        1. Default gateway (most reliable for WSL2)
        2. /etc/resolv.conf nameserver
        """
        # Method 1: Get default gateway (Windows host IP in WSL2)
        try:
            import subprocess
            result = subprocess.run(
                ['ip', 'route', 'show', 'default'],
                capture_output=True,
                text=True,
                timeout=1
            )
            if result.returncode == 0:
                # Output format: "default via 172.20.128.1 dev eth0 ..."
                parts = result.stdout.split()
                if len(parts) >= 3 and parts[0] == 'default' and parts[1] == 'via':
                    gateway_ip = parts[2]
                    self.get_logger().info(f'Detected Windows host via gateway: {gateway_ip}')
                    return gateway_ip
        except Exception as e:
            self.get_logger().debug(f'Failed to detect gateway: {e}')
        
        # Method 2: /etc/resolv.conf nameserver (fallback)
        try:
            with open('/etc/resolv.conf', 'r') as f:
                for line in f:
                    if line.startswith('nameserver'):
                        ip = line.split()[1]
                        self.get_logger().info(f'Detected Windows host via resolv.conf: {ip}')
                        return ip
        except Exception as e:
            self.get_logger().warn(f'Failed to detect Windows host IP: {e}')
        
        # Fallback to localhost
        self.get_logger().warn('Could not auto-detect Windows host. Using 127.0.0.1')
        return '127.0.0.1'

    def destroy_node(self):
        """Cleanup on shutdown"""
        if self.cam is not None:
            self.cam.close()
            self.get_logger().info('Virtual camera closed')
        if self.stream_socket is not None:
            self.stream_socket.close()
            self.get_logger().info('Network stream closed')
        if self.show_preview and CV2_AVAILABLE:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TfVirtualCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
