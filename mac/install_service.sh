#!/bin/sh
# Keep the dashboard running as a login service (launchd): it starts when you log in, restarts if it
# stops, and serves http://localhost:8010/ (this Mac only).
#   mac/install_service.sh          install (or reinstall) and start it
#   mac/install_service.sh remove   stop it and take it out
# Log: mac/dashboard.log. Extra dashboard arguments go in ROVER_DASHBOARD_ARGS (e.g. --host <camera>).
set -e
LABEL=local.home-rover.dashboard
PLIST="$HOME/Library/LaunchAgents/$LABEL.plist"
REPO=$(cd "$(dirname "$0")/.." && pwd)
PYTHON=${PYTHON:-$(command -v python3)}

launchctl bootout "gui/$(id -u)/$LABEL" 2>/dev/null || true
if [ "$1" = remove ]; then
  rm -f "$PLIST"
  echo "Dashboard service removed."
  exit 0
fi

ARGS=""
for a in $ROVER_DASHBOARD_ARGS; do ARGS="$ARGS<string>$a</string>"; done
mkdir -p "$HOME/Library/LaunchAgents"
cat > "$PLIST" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0"><dict>
  <key>Label</key><string>$LABEL</string>
  <key>ProgramArguments</key><array><string>$PYTHON</string><string>-u</string><string>$REPO/mac/dashboard.py</string>$ARGS</array>
  <key>WorkingDirectory</key><string>$REPO</string>
  <key>RunAtLoad</key><true/>
  <key>KeepAlive</key><true/>
  <key>ThrottleInterval</key><integer>10</integer>
  <key>ProcessType</key><string>Interactive</string>
  <key>StandardOutPath</key><string>$REPO/mac/dashboard.log</string>
  <key>StandardErrorPath</key><string>$REPO/mac/dashboard.log</string>
</dict></plist>
PLIST
launchctl bootstrap "gui/$(id -u)" "$PLIST"
echo "Dashboard service installed: http://localhost:8010/ (log mac/dashboard.log)"
