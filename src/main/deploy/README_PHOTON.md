PhotonVision deploy helper
=========================

This small helper explains how to deploy and start PhotonVision server on the RoboRIO.

1) Place the Photon server JAR in the deploy folder (or copy it to the RoboRIO directly):

   scp photonvision-server.jar admin@roborio-TEAM.local:/home/lvuser/

2) Copy the start script to the RoboRIO deploy folder (optional — GradleRIO will copy files
   in `src/main/deploy` automatically when you deploy):

   scp src/main/deploy/startPhoton.sh admin@roborio-TEAM.local:/home/lvuser/

3) SSH to the RoboRIO and run the start script (or run it locally on the RoboRIO):

   ssh admin@roborio-TEAM.local
   chmod +x /home/lvuser/startPhoton.sh
   /home/lvuser/startPhoton.sh

4) Verify Photon logs:

   tail -n 200 /home/lvuser/photon.log

Notes
-----
- Running Photon on the RoboRIO may be CPU intensive. If you observe scheduling or performance issues
  move Photon to a coprocessor (RPi/Jetson) and use the PhotonVision coprocessor workflow.
- Make sure the camera name in the Photon UI matches the name you use in your robot code
  (e.g., new PhotonCamera("photonCam")).
