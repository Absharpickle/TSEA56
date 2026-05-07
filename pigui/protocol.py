import struct

def build_command_packet(state, target, action_char):
    """Bygger ett 8-byte 0x05 kommandopaket."""
    target_byte = 0x00 if target == "wheel" else 0x01
    action_byte = ord(str(action_char)[0])
    return struct.pack('BBBBBBBB', 0x05, state, target_byte, action_byte, 0, 0, 0, 0xFF)

def build_item_list_packet(item_edges):
    """Bygger ett 0x07 varulistepaket."""
    if not item_edges: return None
    n = len(item_edges)
    values = [0x07, n]
    for u, v in item_edges: values.extend([u, v])
    values.append(0xFF)
    return struct.pack('B' * len(values), *values)

def parse_telemetry(data):
    """Parsar 15-byte telemetri. VIKTIGT: Måste vara exakt 15 bytes."""
    if len(data) != 15 or data[0] != 0x06 or data[14] != 0xFF:
        return None
    
    unpacked = struct.unpack('15B', data)
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
        'action_done': unpacked[12],
        'ir_distance': unpacked[13] # <--- Här hämtas IR-värdet
    }
