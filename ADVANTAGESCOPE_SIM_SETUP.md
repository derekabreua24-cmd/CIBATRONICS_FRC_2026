AdvantageScope & WPILib Simulator — Quick Setup

This short guide explains how to run the WPILib simulator for this project and verify AdvantageKit / AdvantageScope is receiving telemetry (NT4).

Prerequisites
- Java 17 (installed on your machine)
- WPILib / GradleRIO development environment (installed via WPILib installer)
- AdvantageScope app (Mechanical Advantage) installed on your PC
- Firewall rules allow local TCP/UDP between simulator and AdvantageScope (localhost should be allowed)

Run the simulator
1. Build the project:

```powershell
./gradlew.bat build
```

2. Start the simulator (desktop run):

```powershell
./gradlew.bat simulateRobot
```

What to expect
- The simulator will launch the WPILib GUI. The robot code will call `simulationInit()` which:
  - Starts the selected autonomous command (useful for smoke tests).
  - Publishes a small AdvantageKit "smoke" signal (Logger.recordOutput entries "Smoke/AdvantageKit/Alive" and "Smoke/AdvantageKit/Message").
- In the simulator console you should see log lines indicating `simulationInit`.

Verify AdvantageScope
1. Open AdvantageScope on the same machine.
2. It should auto-detect the robot's NT4 stream. If not, connect to localhost or the simulator address.
3. Look for data under the logger/junction frames or a key named similar to "Smoke/AdvantageKit/Alive".
4. You should see the message "simInit" appear under "Smoke/AdvantageKit/Message".

Troubleshooting
- No data in AdvantageScope:
  - Make sure both the simulator and AdvantageScope are on the same machine.
  - Temporarily disable Windows Firewall or add a rule to allow the simulator's Java process and AdvantageScope to communicate over the local network.
  - Confirm `Robot.robotInit()` called `Logger.start()` (this project does this by default).
- JNI or native load errors:
  - Check the simulation console for UnsatisfiedLinkError messages; vendordeps contains platform-native artifacts for AdvantageKit - ensure `windowsx86-64` is allowed.
- Shuffleboard entries missing:
  - Open Shuffleboard and check the "Autonomous" tab. The PathPlanner chooser is displayed as "PathPlanner Autos".

Next steps
- If you want automated verification, I can add a short unit/integration test that runs in desktop mode and asserts Logger produced a frame; this requires wiring a test NT4 consumer and is optional.
