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
    """Parse a 14-byte telemetry packet into a dict. Returns None if invalid."""
    if len(data) != 14 or data[0] != 0x06 or data[13] != 0xFF:
        return None
    
    unpacked = struct.unpack('14B', data)
    return {
        'phase': unpacked[1],
        'action': chr(unpacked[2]),
        'next_action': chr(unpacked[3]),
        'line_var': unpacked[4],
        'gyro1': unpacked[5],
        'gyro2': unpacked[6],
        'flags': unpacked[7],
        'current_node': unpacked[8],
        'current_item': unpacked[9],
        'item_count': unpacked[10],
        'direction': chr(unpacked[11]),
        'action_done': unpacked[12]
    }
