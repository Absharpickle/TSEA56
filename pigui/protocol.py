import struct

def build_command_packet(state, target, action_val):
    """
    Bygger ett 8-byte 0x05 command packet.
    target: 0x00 för Hjul ("wheel"), 0x01 för Arm ("arm") eller direkt hex-värde.
    action_val: Kan vara en sträng (t.ex. 'f' för hjul) eller ett heltal (0-255 för arm).
    """
    if isinstance(target, str):
        target_byte = 0x00 if target == "wheel" else 0x01
    else:
        target_byte = target

    # Hantera både karaktärer (för hjul) och rena bytes (för armen)
    if isinstance(action_val, str):
        action_byte = ord(action_val[0])
    else:
        action_byte = action_val & 0xFF
        
    return struct.pack('BBBBBBBB',
                       0x05, state, target_byte, action_byte,
                       0x00, 0x00, 0x00, 0xFF)

def encode_arm_action(joint_index, movement_type):
    """
    Packar vilken joint och vilken rörelse till ETT byte (8-bitar) för styrmodulen.
    """
    move_bits = 0b00
    if movement_type == 'inc':
        move_bits = 0b10
    elif movement_type == 'dec':
        move_bits = 0b01
    
    joint_mask = 0
    if joint_index == 6:  
        joint_mask = 0b111111 
    else:
        joint_mask = (1 << joint_index)
        
    return (joint_mask << 2) | move_bits

def build_item_list_packet(item_edges):
    if not item_edges:
        return None
    
    n = len(item_edges)
    fmt = 'B' * (3 + 2 * n)
    values = [0x07, n]
    for u, v in item_edges:
        values.extend([u, v])
    values.append(0xFF)
    
    return struct.pack(fmt, *values)

def parse_telemetry(data):
    if len(data) != 14 or data[0] != 0x06 or data[13] != 0xFF:
        return None
    
    unpacked = struct.unpack('14B', data)
    return {
        'phase': unpacked[1],
        'action': chr(unpacked[2]),
        'next_action': chr(unpacked[3]),
        'line_var_f': unpacked[4],
        'gyro1': unpacked[5],
        'gyro2': unpacked[6],
        'flags': unpacked[7],
        'node': unpacked[8],
        'item_idx': unpacked[9],
        'item_count': unpacked[10],
        'direction': chr(unpacked[11]),
        'action_done': unpacked[12]
    }