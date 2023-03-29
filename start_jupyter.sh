#! /bin/bash
# Author: Thomas Herring
# License: MIT

"""
This script starts a jupyter notebook server in a tmux session. Useful for running persistent but managed processes.
"""

# tmux new-session -d -s "jupyter" jupyter lab --ip=0.0.0.0 --port=8888 --allow-root --no-browser --NotebookApp.token='robobio'
# jupyter lab --ip=0.0.0.0 --port=8888 --allow-root --no-browser --NotebookApp.token='robobio'
# jupyter notebook --ip=0.0.0.0 --port=8888 --allow-root --no-browser --NotebookApp.token='robobio'

$CONDA_PREFIX/bin/jupyter lab --ip=0.0.0.0 --port=8888 --allow-root --no-browser --NotebookApp.token='robobio' --NotebookApp.allow_origin='*'
