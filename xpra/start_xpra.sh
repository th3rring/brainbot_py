#!/bin/bash

export XPRA_PORT=10000

# Start session w/ terminal in new tmux session
# tmux new-session -d -s "xpra" xpra start --bind-tcp=0.0.0.0:$XPRA_PORT --html=on --start-child=gnome-terminal --exit-with-children=no --daemon=no --pulseaudio=no --notifications=no --bell=no

# Start in this session, attach terminal
# Env var increases the threshold for alerting the user about network instabilities
xpra start --bind-tcp=localhost:$XPRA_PORT --html=on --start-child=gnome-terminal --exit-with-children=no --daemon=no --pulseaudio=no --notifications=no --bell=no --env=XPRA_ACK_TOLERANCE=250


# Start in this session, no terminal
# xpra start --bind-tcp=localhost:$XPRA_PORT --html=on --exit-with-children=no --daemon=no --pulseaudio=no --notifications=no --bell=no
