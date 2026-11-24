#!/usr/bin/env bash
set -euo pipefail

pi_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
for file in bridge.py desktop.py ../control_panel/argos_control_panel.py ../control_panel/argos_exploration.py \
    ../control_panel/argos_odometry.py ../control_panel/argos_motion_calibration.json; do
    test -r "$pi_dir/$file" || { printf 'Missing file: %s\n' "$file" >&2; exit 1; }
done
if [[ "$EUID" == 0 ]]; then
    echo "Run as the Pi desktop user. The installer uses sudo for system changes."
    exit 1
fi

sudo apt-get update
sudo apt-get install -y python3-serial python3-tk xdg-user-dirs
sudo usermod -aG dialout "$USER"
sudo loginctl enable-linger "$USER"

mkdir -p "$HOME/.config/systemd/user" "$HOME/.local/share/applications"
cat > "$HOME/.config/systemd/user/argos-bridge.service" <<EOF
[Unit]
Description=ARGOS Raspberry Pi UART bridge

[Service]
ExecStart=/usr/bin/python3 "$pi_dir/bridge.py"
Restart=on-failure
RestartSec=2

[Install]
WantedBy=default.target
EOF

desktop_dir="$(xdg-user-dir DESKTOP)"
if [[ -z "$desktop_dir" || "$desktop_dir" == "$HOME" ]]; then
    desktop_dir="$HOME/Desktop"
fi
mkdir -p "$desktop_dir"
launcher="$HOME/.local/share/applications/argos.desktop"
cat > "$launcher" <<EOF
[Desktop Entry]
Type=Application
Name=ARGOS
Exec=/usr/bin/python3 "$pi_dir/desktop.py"
Icon=applications-engineering
Terminal=false
Categories=Utility;
EOF
install -m 755 "$launcher" "$desktop_dir/ARGOS.desktop"
systemctl --user daemon-reload
systemctl --user enable argos-bridge.service
echo "ARGOS installed. Reboot to apply UART group membership and start the bridge."
