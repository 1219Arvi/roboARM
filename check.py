import serial
import struct
import time

# ========== CONFIG ========== #
PORT = '/dev/ttyACM0'         # Change this to match your serial port
BAUDRATE = 115200
TOTAL_SERVOS = 7              # Adjust this to match your setup
STRUCT_HEADER = b'STRT'
CMD_MOVE = b'M'
CMD_FEEDBACK = b'F'

# ========== SERVO STRUCT ========== #
# Format: 7 int16_t positions + 7 int16_t velocities = 14 int16_t = 28 bytes
servo_struct_format = f"<{TOTAL_SERVOS * 2}h"
servo_struct_size = struct.calcsize(servo_struct_format)

def calculate_checksum(command, packet_size, payload_size, payload_bytes):
    checksum = 0
    checksum += command[0]
    checksum += packet_size & 0xFF
    checksum += (packet_size >> 8) & 0xFF
    checksum += payload_size & 0xFF
    checksum += (payload_size >> 8) & 0xFF
    checksum += sum(payload_bytes)
    return checksum & 0xFF

def send_command(ser, command, positions, velocities):
    # Pack the payload (positions + velocities)
    payload = struct.pack(servo_struct_format, *(positions + velocities))
    payload_size = len(payload)
    packet_size = 1 + 2 + payload_size + 1 + 1  # command + payload_size (2) + payload + checksum + EOC
    checksum = calculate_checksum(command, packet_size, payload_size, payload)

    # Build and send the packet
    packet = bytearray()
    packet.extend(STRUCT_HEADER)
    packet.extend(struct.pack('<H', packet_size))     # packet size (2 bytes)
    packet.extend(command)                            # command (1 byte)
    packet.extend(struct.pack('<H', payload_size))    # payload size (2 bytes)
    packet.extend(payload)                            # payload
    packet.append(checksum)                           # checksum (1 byte)
    packet.append(ord('\n'))                          # end of command

    ser.write(packet)

def request_feedback(ser: serial.Serial):
    # No payload needed for feedback
    payload_size = 0
    packet_size = 1 + 2 + payload_size + 1 + 1
    checksum = calculate_checksum(CMD_FEEDBACK, packet_size, payload_size, b'')

    packet = bytearray()
    packet.extend(STRUCT_HEADER)
    packet.extend(struct.pack('<H', packet_size))
    packet.extend(CMD_FEEDBACK)
    packet.extend(struct.pack('<H', payload_size))
    packet.append(checksum)
    packet.append(ord('\n'))

    ser.write(packet)

    # Wait and read feedback packet
    header = ser.read(len(STRUCT_HEADER))
    if header != STRUCT_HEADER:
        print("Invalid header", header)
        return None

    packet_size = struct.unpack('<H', ser.read(2))[0]
    command = ser.read(1)
    if command != CMD_FEEDBACK:
        print("Unexpected command in feedback")
        return None

    payload_size = struct.unpack('<H', ser.read(2))[0]
    payload = ser.read(payload_size)
    checksum = ser.read(1)[0]
    eoc = ser.read(1)
    if eoc != b'\n':
        print("Invalid EOC")
        return None

    if checksum != calculate_checksum(command, packet_size, payload_size, payload):
        print("Invalid checksum")
        return None

    values = struct.unpack(servo_struct_format, payload)
    positions = values[:TOTAL_SERVOS]
    velocities = values[TOTAL_SERVOS:]
    return positions, velocities

def synchronize_serial(ser, target="STRT"):
    buffer = ""
    print("Waiting for synchronization string 'STRT' from Arduino...")
    while True:
        byte = ser.read(1).decode('utf-8', errors='ignore')
        if byte:
            buffer += byte
            buffer = buffer[-4:]  # Keep only the last 4 characters
            if buffer == target:
                print("Synchronized with Arduino!")
                break

    ser.read(35)

# ========== MAIN ========== #
if __name__ == "__main__":
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    time.sleep(3)  # Give time for Arduino reset

    # Example: move all servos to 90 degrees with zero velocity
    positions = [0, 0, 0, 0, 0, 0, 90]  # Example positions for servos
    # positions = [45, 45, 45, 0, 0, 0, 0]  # Example positions for servos
    # positions = [0] * TOTAL_SERVOS  # Example positions for servos
    velocities = [10, 10, 10, 10, 50, 10, -50]  # Example velocities for servos
    # synchronize_serial(ser)
    while(True):
        send_command(ser, CMD_MOVE, positions, velocities)
        print("Command sent!")
        time.sleep(0.01)
        feedback = request_feedback(ser)
        if feedback:
            pos, vel = feedback
            print("Feedback Positions:", pos)
            print("Feedback Velocities:", vel)