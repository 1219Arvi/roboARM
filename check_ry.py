import serial
import struct
import time

# ========== CONFIG ========== #
PORT = '/dev/ttyACM0'         # Update to match your serial port
BAUDRATE = 115200             # Match your Arduino's baud rate
TOTAL_SERVOS = 7
STRUCT_HEADER = b'S'
CMD_MOVE = b'M'
CMD_FEEDBACK = b'F'

# ========== SERVO STRUCT ========== #
# Format: 7 float positions + 7 float velocities = 14 floats = 56 bytes
servo_struct_format = "<" + "f" * TOTAL_SERVOS * 2
servo_struct_size = struct.calcsize(servo_struct_format)
print(f"Servo struct size: {servo_struct_size} bytes")  # Should be 56 bytes for 7 servos

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
    packet.append(ord('\n')) 
    # print(len(packet))                         # end of command
    # CHUNK_SIZE = 45
    # ser.reset_output_buffer() 
    # for i in range(0, len(packet), CHUNK_SIZE):
        
    #     ser.write(packet[i:i+CHUNK_SIZE])
        # ser.flush()  # Ensure the chunk is sent

    ser.write(packet)
    # ser.flush()  # Ensure the command is sent immediately

def wait_for_data(ser, num_bytes, timeout=0.1):
    start_time = time.time()
    while ser.in_waiting < num_bytes:
        if time.time() - start_time > timeout:
            print("Timeout waiting for data")
            return False
        # time.sleep(0.1)  # Small delay to avoid busy waiting
    return True

def request_feedback(ser):
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
    ser.reset_output_buffer() 
    ser.write(packet)
    # ser.flush()

    # Wait for header (with timeout check)
    if not wait_for_data(ser, len(STRUCT_HEADER)):
        print("Header not received in time")
        return None
    header = ser.read(len(STRUCT_HEADER))
    if header != STRUCT_HEADER:
        print("Invalid header", header)
        return None

    # Read packet size
    if not wait_for_data(ser, 2):
        return None
    packet_size = struct.unpack('<H', ser.read(2))[0]

    # Read command and validate
    if not wait_for_data(ser, 1):
        return None
    command = ser.read(1)
    if command != CMD_FEEDBACK:
        print("Unexpected command in feedback")
        return None

    # Read payload size
    if not wait_for_data(ser, 2):
        return None
    payload_size = struct.unpack('<H', ser.read(2))[0]

    # Read the payload
    if not wait_for_data(ser, payload_size):
        return None
    payload = ser.read(payload_size)

    # Read checksum and EOC
    if not wait_for_data(ser, 2):
        return None
    checksum = ser.read(1)[0]
    eoc = ser.read(1)
    if eoc != b'\n':
        print(f"Invalid EOC: {eoc}")
        return None

    # Validate checksum
    if checksum != calculate_checksum(command, packet_size, payload_size, payload):
        print("Invalid checksum")
        return None

    # Unpack values from the payload
    values = struct.unpack(servo_struct_format, payload)
    positions = values[:TOTAL_SERVOS]
    velocities = values[TOTAL_SERVOS:]

    return positions, velocities


# ========== MAIN ========== #
if __name__ == "__main__":
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    time.sleep(2)  # Give time for Arduino reset

    # Example: move all servos to 90 degrees with zero velocity
    positions = [0, 0, 0, 0, 0, 0,10]  # Example positions for servos
    # positions = [0, 0, 25, 45, 45, 50, 10]  # Example positions for servos
    # positions = [0.0] * TOTAL_SERVOS  # Example positions for servos
    # positions = [10, 0, 10, -25, 45, 30, 60]  # Example velocities for servos
    # positions = [45, 40, 40, 0, 0, 0, 0] 
    velocities = [20.0] * TOTAL_SERVOS
    # send_command(ser, CMD_MOVE, positions, velocities)
    # print(ser.read(1))
    # ser.flush()
    # request_feedback(ser)
    # ser.reset_input_buffer()
    # ser.reset_output_buffer()
    # time.sleep(2)  # wait for Arduino reset
    while(True):
        send_command(ser, CMD_MOVE, positions, velocities)
    
        time.sleep(1)
        feedback = request_feedback(ser)
        # print("Command sent!")
        if feedback:
            pos, vel = feedback
            print("Feedback Positions:", pos)
            print("Feedback Velocities:", vel)