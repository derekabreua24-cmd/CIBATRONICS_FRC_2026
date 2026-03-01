Camera calibration and AprilTag setup

This project expects a simple properties file at deploy path:

  src/main/deploy/camera/camera_calib.properties

Keys (defaults shown):
- cameraName (string) - usbCam0
- deviceIndex (int) - 0
- fx, fy, cx, cy (double) - focal lengths and principal point in pixels
- tagSizeMeters (double) - AprilTag side length in meters (2026 REBUILT: 6.5 in = 0.1651)
- cameraTx/cameraTy/cameraTz (double, optional) - translation of camera relative to robot frame in meters
- cameraRotDegX/cameraRotDegY/cameraRotDegZ (double, optional) - rotation (degrees) of camera relative to robot frame

Behavior
- On robot startup, the calibration file is loaded and the Vision subsystem and USB AprilTag processor are started.
- The system is intentionally fail-fast: missing or malformed calibration or AprilTag layout JSON will cause startup to throw (so you'll see the problem early).
- To stop the USB processor when disabled, `Robot.disabledInit()` will call the container shutdown hook.

Files
* For 2026 REBUILT, the detector uses the **tag36h11** family and tag size 6.5 in (0.1651 m). A field layout can be loaded from `deploy/apriltagfield_2026.json` (or fallback to WPILib default).
- `src/main/deploy/camera/camera_calib.properties` – example calibration properties (edit before deploy).

If you prefer graceful startup instead of fail-fast, I can change the behavior to log and continue without vision.
