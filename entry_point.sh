#!/bin/bash

spacenavd &
BASHRC_LINE='omni_python /pkgs/curobo/examples/automatica2025/init_spacenav.py'
grep -Fxq "$BASHRC_LINE" ~/.bashrc || echo "$BASHRC_LINE" >> ~/.bashrc

exec bash