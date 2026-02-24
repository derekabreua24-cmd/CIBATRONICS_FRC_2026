Camera calibration and AprilTag setup

This project expects a simple properties file at deploy path:

  src/main/deploy/camera/camera_calib.properties

Keys (defaults shown):
- cameraName (string) - usbCam0
- deviceIndex (int) - 0
- fx, fy, cx, cy (double) - focal lengths and principal point in pixels
- tagSizeMeters (double) - AprilTag side length in meters (e.g. 0.1524)
- cameraTx/cameraTy/cameraTz (double, optional) - translation of camera relative to robot frame in meters
- cameraRotDegX/cameraRotDegY/cameraRotDegZ (double, optional) - rotation (degrees) of camera relative to robot frame

Behavior
- On robot startup, the calibration file is loaded and the Vision subsystem and USB AprilTag processor are started.
- The system is intentionally fail-fast: missing or malformed calibration or AprilTag layout JSON will cause startup to throw (so you'll see the problem early).
- To stop the USB processor when disabled, `Robot.disabledInit()` will call the container shutdown hook.

Files
* The project now uses WPILib's internal AprilTag fields API directly (AprilTagFields) and no longer includes any deployed AprilTag JSON.
- `src/main/deploy/camera/camera_calib.properties` – example calibration properties (edit before deploy).

If you prefer graceful startup instead of fail-fast, I can change the behavior to log and continue without vision.
