#!/usr/bin/env python3
"""
Interactive CLI tool for controlling the real ABB IRB 120 robot.

This provides a simple command-line interface for:
- Connecting to the robot
- Reading joint positions
- Executing simple movements
- Testing the socket communication

Usage:
    python3 irb120_control.py
    python3 irb120_control.py --ip 192.168.125.1 --port 5000
"""

import socket
import time
import argparse
import readline  # Enables command history with arrow keys


class IRB120Control:
    """Interactive controller for ABB IRB 120 robot."""

    def __init__(self, ip='192.168.125.1', port=5000):
        self.ip = ip
        self.port = port
        self.socket = None
        self.connected = False

    def connect(self) -> bool:
        """Connect to robot socket server."""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(10.0)
            self.socket.connect((self.ip, self.port))
            self.connected = True
            print(f'Connected to {self.ip}:{self.port}')

            # Verify with PING
            response = self.send('PING')
            if 'PONG' in response or 'POS:' in response:
                print('Connection verified')
                return True
            else:
                print(f'Warning: Unexpected response: {response}')
                return True
        except Exception as e:
            print(f'Connection failed: {e}')
            self.connected = False
            return False

    def disconnect(self):
        """Disconnect from robot."""
        if self.socket:
            try:
                self.send('QUIT')
            except:
                pass
            try:
                self.socket.close()
            except:
                pass
        self.socket = None
        self.connected = False
        print('Disconnected')

    def send(self, cmd: str, timeout: float = 60.0) -> str:
        """Send command and receive response."""
        if not self.connected:
            return 'ERROR: Not connected'

        try:
            self.socket.settimeout(timeout)
            self.socket.send((cmd + '\n').encode())
            time.sleep(0.1)
            response = self.socket.recv(1024).decode().strip()
            return response
        except socket.timeout:
            return 'ERROR: Timeout'
        except Exception as e:
            return f'ERROR: {e}'

    def get_position(self):
        """Get and display current joint positions."""
        response = self.send('GETPOS')
        if response.startswith('POS:'):
            joints = response[4:].split()
            print('\nCurrent Joint Positions (degrees):')
            print('-' * 35)
            names = ['Joint 1 (base)', 'Joint 2 (shoulder)', 'Joint 3 (elbow)',
                     'Joint 4 (wrist 1)', 'Joint 5 (wrist 2)', 'Joint 6 (wrist 3)']
            for name, val in zip(names, joints):
                print(f'  {name:20s}: {float(val):8.2f}°')
            print()
        else:
            print(f'Response: {response}')

    def move(self, joints: list):
        """Move to joint positions (degrees)."""
        if len(joints) != 6:
            print('Error: Need exactly 6 joint values')
            return

        cmd = 'MOVEJ ' + ' '.join([f'{j:.2f}' for j in joints])
        print(f'Sending: {cmd}')
        response = self.send(cmd)
        print(f'Response: {response}')

    def home(self):
        """Move to home position (all zeros)."""
        print('Moving to home position...')
        self.move([0, 0, 0, 0, 0, 0])

    def set_speed(self, speed: int):
        """Set movement speed (1-500 mm/s)."""
        if not 1 <= speed <= 500:
            print('Speed must be between 1 and 500')
            return
        response = self.send(f'SPEED {speed}')
        print(f'Response: {response}')

    def stop(self):
        """Stop robot movement."""
        response = self.send('STOP')
        print(f'Response: {response}')

    def ping(self):
        """Test connection with PING."""
        response = self.send('PING')
        print(f'Response: {response}')

    def run_interactive(self):
        """Run interactive command loop."""
        print()
        print('=' * 50)
        print('  ABB IRB 120 Interactive Control')
        print('=' * 50)
        print()
        print('Commands:')
        print('  connect              - Connect to robot')
        print('  disconnect           - Disconnect from robot')
        print('  ping                 - Test connection')
        print('  status / pos         - Show current joint positions')
        print('  home                 - Move to home position (all zeros)')
        print('  move J1 J2 J3 J4 J5 J6  - Move to joint positions (degrees)')
        print('  speed N              - Set speed (1-500 mm/s)')
        print('  stop                 - Stop movement')
        print('  raw <command>        - Send raw command to robot')
        print('  help                 - Show this help')
        print('  quit / exit          - Exit program')
        print()

        while True:
            try:
                # Show connection status in prompt
                status = '●' if self.connected else '○'
                cmd = input(f'[{status}] irb120> ').strip()

                if not cmd:
                    continue

                parts = cmd.split()
                command = parts[0].lower()

                if command in ['quit', 'exit', 'q']:
                    if self.connected:
                        self.disconnect()
                    break

                elif command == 'connect':
                    if self.connected:
                        print('Already connected')
                    else:
                        self.connect()

                elif command == 'disconnect':
                    if self.connected:
                        self.disconnect()
                    else:
                        print('Not connected')

                elif command == 'ping':
                    if self.connected:
                        self.ping()
                    else:
                        print('Not connected')

                elif command in ['status', 'pos', 'position', 'getpos']:
                    if self.connected:
                        self.get_position()
                    else:
                        print('Not connected')

                elif command == 'home':
                    if self.connected:
                        self.home()
                    else:
                        print('Not connected')

                elif command == 'move':
                    if not self.connected:
                        print('Not connected')
                    elif len(parts) != 7:
                        print('Usage: move J1 J2 J3 J4 J5 J6')
                        print('Example: move 10 0 0 0 0 0')
                    else:
                        try:
                            joints = [float(x) for x in parts[1:7]]
                            self.move(joints)
                        except ValueError:
                            print('Invalid joint values. Use numbers.')

                elif command == 'speed':
                    if not self.connected:
                        print('Not connected')
                    elif len(parts) != 2:
                        print('Usage: speed N (1-500)')
                    else:
                        try:
                            speed = int(parts[1])
                            self.set_speed(speed)
                        except ValueError:
                            print('Invalid speed value')

                elif command == 'stop':
                    if self.connected:
                        self.stop()
                    else:
                        print('Not connected')

                elif command == 'raw':
                    if not self.connected:
                        print('Not connected')
                    elif len(parts) < 2:
                        print('Usage: raw <command>')
                    else:
                        raw_cmd = ' '.join(parts[1:])
                        response = self.send(raw_cmd)
                        print(f'Response: {response}')

                elif command == 'help':
                    print()
                    print('Commands:')
                    print('  connect    - Connect to robot')
                    print('  disconnect - Disconnect')
                    print('  ping       - Test connection')
                    print('  status     - Show joint positions')
                    print('  home       - Move to home')
                    print('  move J1-J6 - Move to position')
                    print('  speed N    - Set speed')
                    print('  stop       - Stop movement')
                    print('  raw <cmd>  - Send raw command')
                    print('  quit       - Exit')
                    print()

                else:
                    print(f'Unknown command: {command}')
                    print('Type "help" for available commands')

            except KeyboardInterrupt:
                print('\nUse "quit" to exit')
            except EOFError:
                break

        print('Goodbye!')


def main():
    parser = argparse.ArgumentParser(
        description='Interactive control for ABB IRB 120 robot'
    )
    parser.add_argument('--ip', default='192.168.125.1',
                        help='Robot IP address (default: 192.168.125.1)')
    parser.add_argument('--port', type=int, default=5000,
                        help='Socket port (default: 5000)')
    parser.add_argument('--auto-connect', '-c', action='store_true',
                        help='Automatically connect on startup')

    args = parser.parse_args()

    controller = IRB120Control(ip=args.ip, port=args.port)

    if args.auto_connect:
        controller.connect()

    controller.run_interactive()


if __name__ == '__main__':
    main()
