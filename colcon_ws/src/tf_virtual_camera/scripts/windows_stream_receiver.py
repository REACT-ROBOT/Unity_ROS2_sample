"""
Windows UDP Stream Receiver for OBS Virtual Camera
Receives frames from WSL2 ROS node and outputs to OBS Virtual Camera

Requirements:
    pip install numpy pyvirtualcam opencv-python

Usage:
    python windows_stream_receiver.py --port 5000
"""

import socket
import struct
import numpy as np
import argparse
import time
from collections import defaultdict

try:
    import pyvirtualcam
    VIRTUAL_CAM_AVAILABLE = True
except ImportError:
    VIRTUAL_CAM_AVAILABLE = False
    print("Error: pyvirtualcam not installed")
    print("Install with: pip install pyvirtualcam")
    exit(1)

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    print("Warning: opencv-python not installed. Preview disabled.")
    print("Install with: pip install opencv-python")


class UDPFrameReceiver:
    """Receives frames via UDP and outputs to virtual camera"""
    
    def __init__(self, port=5000, width=640, height=360, fps=30):
        self.port = port
        self.width = width
        self.height = height
        self.fps = fps
        
        # Frame assembly buffer
        self.frame_buffer = defaultdict(dict)
        self.last_complete_seq = -1
        
        # Statistics
        self.frames_received = 0
        self.frames_dropped = 0
        self.last_stats_time = time.time()
        
    def start(self, preview=False):
        """Start receiving and outputting frames"""
        
        # Setup UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('0.0.0.0', self.port))
        sock.settimeout(0.01)  # 10ms timeout for faster loop
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1048576)  # 1MB receive buffer
        
        print(f"Listening on UDP port {self.port}")
        print(f"Output: {self.width}x{self.height} @ {self.fps} FPS")
        
        # Setup virtual camera
        cam = None
        if VIRTUAL_CAM_AVAILABLE:
            try:
                cam = pyvirtualcam.Camera(width=self.width, height=self.height, 
                                         fps=self.fps, fmt=pyvirtualcam.PixelFormat.RGB)
                print(f"Virtual camera started: {cam.device}")
            except Exception as e:
                print(f"Failed to start virtual camera: {e}")
                print("Make sure OBS Virtual Camera is installed and not in use")
                return
        
        print("Receiving frames... Press Ctrl+C to stop")
        
        last_frame = None
        frames_since_last_log = 0
        
        try:
            while True:
                packets_received = 0
                
                # Receive all available packets in burst
                while packets_received < 100:  # Max 100 packets per iteration
                    try:
                        # Receive packet
                        data, addr = sock.recvfrom(65536)
                        packets_received += 1
                        
                        # Parse header
                        if len(data) < 10:
                            continue
                        
                        header = struct.unpack('!HHHHH', data[:10])
                        seq, chunk_id, total_chunks, width, height = header
                        chunk_data = data[10:]
                        
                        # Update dimensions if changed
                        if width != self.width or height != self.height:
                            self.width = width
                            self.height = height
                            print(f"Frame size updated: {width}x{height}")
                        
                        # Store chunk
                        self.frame_buffer[seq][chunk_id] = chunk_data
                        
                        # Check if frame is complete
                        if len(self.frame_buffer[seq]) == total_chunks:
                            frame = self._assemble_frame(seq, total_chunks)
                            
                            if frame is not None:
                                last_frame = frame
                                frames_since_last_log += 1
                                
                                # Clean up old buffers
                                self._cleanup_buffers(seq)
                                self.last_complete_seq = seq
                    
                    except socket.timeout:
                        break  # No more packets available
                    except Exception as e:
                        # Skip individual packet errors
                        pass
                
                # Send last complete frame to virtual camera
                if last_frame is not None and cam is not None:
                    try:
                        cam.send(last_frame)
                        self.frames_received += 1
                    except Exception as e:
                        print(f"Failed to send to virtual camera: {e}")
                    
                    # Preview
                    if preview and CV2_AVAILABLE:
                        cv2.imshow('Stream Preview', cv2.cvtColor(last_frame, cv2.COLOR_RGB2BGR))
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break
                
                # Print statistics every 5 seconds
                current_time = time.time()
                if current_time - self.last_stats_time >= 5.0:
                    elapsed = current_time - self.last_stats_time
                    fps_actual = self.frames_received / elapsed
                    print(f"FPS: {fps_actual:.1f}, Frames: {self.frames_received}, Dropped: {self.frames_dropped}, Buffered: {len(self.frame_buffer)}")
                    self.frames_received = 0
                    self.frames_dropped = 0
                    self.last_stats_time = current_time
                    
        except KeyboardInterrupt:
            print("\nStopping...")
        
        finally:
            if cam is not None:
                cam.close()
            sock.close()
            if preview and CV2_AVAILABLE:
                cv2.destroyAllWindows()
            print("Receiver stopped")
    
    def _assemble_frame(self, seq, total_chunks):
        """Assemble frame from chunks"""
        try:
            # Concatenate chunks in order
            frame_data = b''.join([self.frame_buffer[seq][i] for i in range(total_chunks)])
            
            # Reshape to image
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = frame.reshape((self.height, self.width, 3))
            
            return frame
            
        except Exception as e:
            print(f"Failed to assemble frame {seq}: {e}")
            self.frames_dropped += 1
            return None
    
    def _cleanup_buffers(self, current_seq):
        """Remove old frame buffers"""
        # Keep only recent 10 frames
        old_seqs = [seq for seq in self.frame_buffer.keys() if abs(seq - current_seq) > 10]
        for seq in old_seqs:
            del self.frame_buffer[seq]


def main():
    parser = argparse.ArgumentParser(description='Windows UDP Stream Receiver for OBS')
    parser.add_argument('--port', type=int, default=5000, help='UDP port to listen on')
    parser.add_argument('--width', type=int, default=640, help='Frame width')
    parser.add_argument('--height', type=int, default=360, help='Frame height')
    parser.add_argument('--fps', type=int, default=30, help='Target FPS')
    parser.add_argument('--preview', action='store_true', help='Show preview window')
    args = parser.parse_args()
    
    receiver = UDPFrameReceiver(port=args.port, width=args.width, height=args.height, fps=args.fps)
    receiver.start(preview=args.preview)


if __name__ == '__main__':
    main()
