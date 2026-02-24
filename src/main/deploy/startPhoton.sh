#!/bin/sh
# startPhoton.sh
# Helper script to start PhotonVision server on the RoboRIO if the server jar
# has been deployed to /home/lvuser.

PHOTON_JAR="/home/lvuser/photonvision-server.jar"
LOGFILE="/home/lvuser/photon.log"

if [ ! -f "$PHOTON_JAR" ]; then
  echo "Photon server jar not found at $PHOTON_JAR" >&2
  exit 1
fi

echo "Starting PhotonVision server..."
nohup java -jar "$PHOTON_JAR" > "$LOGFILE" 2>&1 &
echo "Photon started (logs: $LOGFILE)"
