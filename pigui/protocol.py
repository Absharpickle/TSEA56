# ------------------------------------------------------
# Markus Hellers, Joel Eberhardsson - 28 maj 2026 - V1.0
# ------------------------------------------------------

## Protokollfunktioner för att bygga och parsa paket som skickas mellan GUI:t och roboten ##

import struct

# Bygg ett motorpaket (0x05) för hjul eller armkontroller
def build_command_packet(state, target, action_char):
    target_byte = 0x00 if target == "wheel" else 0x01   # 0x00 = hjul, 0x01 = arm
    action_byte = ord(str(action_char)[0])              # Konvertera action-char till dess ASCII-värde
    
    return struct.pack('BBBBBBBB',
                       0x05, state, target_byte, action_byte,
                       0x00, 0x00, 0x00, 0xFF)          # 8-byte paket: [0x05, state, target, action, 0, 0, 0, 0xFF]


# Bygg ett motorpaket (0x05) specifikt för armens manuella läge
def build_arm_command_packet(state, joint, direction):
    """
    joint: 1-6 (vilken led i armen som ska styras)
    direction: 0=stopp, 1=öka, 2=minska
    Action byte: bits 0-5 = one-hot joint, bits 6-7 = direction
    """
    action_byte = (1 << (joint - 1)) | (direction << 6)
    return struct.pack('BBBBBBBB',
                       0x05, state, 0x01, action_byte,
                       0x00, 0x00, 0x00, 0xFF) # 8-byte paket: [0x05, state, 0x01 (arm), action_byte, 0, 0, 0, 0xFF]


# Bygg ett 0x09 reset-paket för att nollställa robotens tillstånd
def build_reset_packet():
    return struct.pack('BBBBBBBB',
                       0x09, 0x00, 0x00, 0x00,
                       0x00, 0x00, 0x00, 0xFF) # 8-byte paket: [0x09, 0, 0, 0, 0, 0, 0, 0xFF]


# Bygg ett 0x07 item-list paket för att skicka positioner av varor
def build_item_list_packet(item_edges):
    if not item_edges:  # Om listan är tom...
        return None     # ...returnera None (inget paket att skicka)
    
    n = len(item_edges)         # Antal varor i listan
    fmt = 'B' * (3 + 2 * n)
    values = [0x07, n]          # Pakettyp 0x07, följt av antal varor
    for u, v in item_edges:     # Lägg till varje varas position (u, v) i paketet
        values.extend([u, v])   # Lägg till u och v som två bytes i paketet
    values.append(0xFF)         # Avslutande byte
    
    return struct.pack(fmt, *values) # Bygg paketet enligt dynamiskt format

# Parsa ett 0x06 telemetripaket till en dictionary. Returnerar None om paketet är ogiltigt.
def parse_telemetry(data):
    if len(data) != 18 or data[0] != 0x06 or data[17] != 0xFF: # Kontrollera att paketet har rätt läng
        return None
    
    unpacked = struct.unpack('15BbbB', data) # Parsa data
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
