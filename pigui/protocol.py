import struct


def build_command_packet(state, target, action_char):
    """Build an 8-byte 0x05 command packet."""
    target_byte = 0x00 if target == "wheel" else 0x01
    action_byte = ord(str(action_char)[0])
    
    return struct.pack('BBBBBBBB',
                       0x05, state, target_byte, action_byte,
                       0x00, 0x00, 0x00, 0xFF)


def build_item_list_packet(item_edges):
    """Build a variable-length 0x07 item-list packet.
    
    Format: [0x07, N, u1, v1, u2, v2, ..., 0xFF]
    """
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
    """Parse an 18-byte telemetry packet into a dict. Returns None if invalid."""
    if len(data) != 18 or data[0] != 0x06 or data[17] != 0xFF:
        return None
    
    unpacked = struct.unpack('15BbbB', data)
    return {
        'phase': unpacked[0+1],
        'action': chr(unpacked[1+1]),
        'next_action': chr(unpacked[2+1]),
        'line_var': unpacked[3+1],
        'gyro1': unpacked[4+1],
        'gyro2': unpacked[5+1],
        'flags': unpacked[6+1],
        'current_node': unpacked[7+1],
        'current_item': unpacked[8+1],
        'item_count': unpacked[9+1],
        'direction': chr(unpacked[10+1]),
        'action_done': unpacked[11+1],
        'gas_right': unpacked[12+1],
        'gas_left': unpacked[13+1],
        'claw_pos_r': unpacked[14+1],
        'claw_pos_z': unpacked[15+1],
    }
