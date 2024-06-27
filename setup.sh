#!/bin/bash
CURRENT_PATH=$(pwd)
ISAAC_PATH=$HOME/.local/share/ov/pkg/isaac_sim-2023.1.1

echo "Changing paths in .vscode/launch.json"
sed -i "s|isaac_install_path|$ISAAC_PATH|g" .vscode/launch.json
echo "Done."

echo "Changing paths in .vscode/settings.json"
sed -i "s|isaac_install_path|$ISAAC_PATH|g" .vscode/settings.json
echo "Done."

echo "Changing paths in .vscode/tasks.json"
sed -i "s|isaac_install_path|$ISAAC_PATH|g" .vscode/tasks.json
echo "Done."

echo "Changing paths in assets/turtlebot3_waffle_pi/urdf/turtlebot3_waffle_pi.urdf/"
sed -i "s|current_path|$CURRENT_PATH|g" assets/turtlebot3_waffle_pi/urdf/turtlebot3_waffle_pi.urdf
echo "Done."
