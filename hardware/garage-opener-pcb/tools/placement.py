"""Rev 1 placement, in board mm: origin = board top-left corner, x right, y down (KiCad view = lid view)."""
BOARD_W, BOARD_H = 95.0, 70.0
ORIGIN = (100.0, 100.0)          # where the board's top-left sits on the KiCad page
HOLE_INSET = 3.5

# U1: pin 1 (3V3) at the module end, pin 19 (5V) at the USB end. The USB end sits 3.18 mm past pin 19,
# so pin 19 at x=3.18 puts the devkit's USB edge on the board edge.
U1_PIN1 = (48.90, 14.00)


def _front(pin_to_front):
    """Pin-row y for a horizontal header whose front (courtyard minus 0.5 mm) is pin_to_front below its pins."""
    return BOARD_H - pin_to_front


PLACE = {
    # ref: (x, y, rotation_deg, locked)
    "H1": (HOLE_INSET, HOLE_INSET, 0, True),
    "H2": (BOARD_W - HOLE_INSET, HOLE_INSET, 0, True),
    "H3": (HOLE_INSET, BOARD_H - HOLE_INSET, 0, True),
    "H4": (BOARD_W - HOLE_INSET, BOARD_H - HOLE_INSET, 0, True),
    "U1": (U1_PIN1[0], U1_PIN1[1], -90, True),
    "J1": (22.00, _front(8.04), 0, True),     # MC 3.81
    "J3": (40.00, _front(10.04), 0, True),    # MSTBA 5.08
    "J2": (58.00, _front(10.04), 0, True),
    "U2": (84.00, 42.00, -90, True),          # pins run left from pin 1; DHT body lies below the row
    "D4": (6.00, 46.00, 0, True),             # RUN  (pad 1 = K)
    "D5": (12.50, 46.00, 0, True),            # DOOR
    "K1": (66.00, 18.00, -90, False),         # COM top, coil (top corners), NO/NC bottom
    # power, near J1
    "F1": (21.00, 45.00, 0, False),
    "D1": (21.00, 51.00, 0, False),
    "C1": (29.00, 46.00, 0, False),
    "C2": (29.00, 52.00, 0, False),
    # reed, near J3
    "R4": (38.00, 46.00, 0, False),
    "R3": (42.50, 46.00, 0, False),
    "C3": (47.00, 46.00, 0, False),
    "D3": (40.00, 51.50, 0, False),
    # LED resistors
    "R6": (7.00, 52.00, 0, False),
    "R7": (13.50, 52.00, 0, False),
    # relay driver, above the relay coil pins
    "Q1": (60.00, 8.00, 0, False),
    "R1": (55.50, 6.50, 0, False),
    "R2": (55.50, 10.00, 0, False),
    "D2": (66.00, 11.00, 0, False),
    # DHT support
    "R5": (79.00, 38.00, 0, False),
    "C4": (84.00, 38.00, 0, False),
    # board ID straps, above U1 pins 7/8 (x 33.66 / 31.12)
    "R8": (28.00, 7.00, 90, False),
    "R9": (31.00, 7.00, 90, False),
    "R10": (35.00, 7.00, 90, False),
    "R11": (38.00, 7.00, 90, False),
}
# Flat-mounted DHT22 body (15.1 x 25.1 mm + 0.1 margin), centred on U2's pin row, extending downward.
_U2_CX = 84.00 - 1.5 * 2.54
DHT_KEEPOUT = (_U2_CX - 7.6, 44.0, _U2_CX + 7.6, 69.2)
LOCKED = {r for r, v in PLACE.items() if v[3]}
