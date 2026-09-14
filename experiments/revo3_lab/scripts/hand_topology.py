"""MANUS landmark names shared by adapters and legacy solver, without a runtime."""
FINGERS = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
NAMES = ['Hand'] + ['Thumb_' + j for j in ['CMC', 'MCP', 'DIP', 'TIP']] + [
    f + '_' + j for f in FINGERS[1:] for j in ['CMC', 'MCP', 'PIP', 'DIP', 'TIP']]
