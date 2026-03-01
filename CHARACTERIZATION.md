# Drive characterization (SysId) and feedforward

This project uses WPILib’s System Identification (SysId) to characterize the drivetrain and get feedforward gains (kS, kV, kA) for accurate path following and closed-loop drive.

## 2026 KitBot specs (in code)

Constants are set for the **2026 KitBot** (AM14U6 chassis, 6" HiGrip wheels, ToughBox Mini S 10.71:1):

- **Wheel diameter:** 6 in → `0.1524` m  
- **Gear ratio:** `10.71`  
- **Track width:** ~22 in → `0.5588` m (measure yours if you changed frame config)  
- **Sim mass / J:** 25 kg, 2.1 kg⋅m² (typical with battery/bumpers)

If your build differs (e.g. different ratio or wheel size), update `Constants.DriveConstants` and the sim in `DriveSubsystem` accordingly.

## Running characterization

1. **Space:** At least 10 ft (ideally ~20 ft) of clear, flat space for the robot to drive straight.

2. **Deploy and enable:** Deploy this project to the robot and enable (teleop or test).

3. **Run SysId tests (hold buttons):**
   - **Quasistatic forward:** Hold **Left Bumper + A**
   - **Quasistatic reverse:** Hold **Left Bumper + B**
   - **Dynamic forward:** Hold **Left Bumper + X**
   - **Dynamic reverse:** Hold **Left Bumper + Y**

   Run each test until it finishes or you hit the timeout. It’s best to run forward then reverse for the same type (e.g. quasistatic fwd, then quasistatic rev) so the robot ends roughly where it started.

4. **Save the log:** In Driver Station (or your logging flow), save the log file that contains the SysId data (e.g. `.wpilog` from AdvantageKit or WPILog).

5. **Analyze in SysId tool:**  
   - Open the [WPILib SysId](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html) tool.  
   - Load your saved log.  
   - In the analysis pane, use the **Drive** mechanism and the **Linear** fit.  
   - Ensure **wheel diameter** and **units** match (e.g. meters).  
   - The tool will report **kS**, **kV**, and **kA** (and optionally feedback gains).

6. **Paste gains into the project:**
   - In **Constants.java**, set:
     - `DriveConstants.kDriveKS` = SysId **kS** (volts)
     - `DriveConstants.kDriveKV` = SysId **kV** (volts per m/s)
     - `DriveConstants.kDriveKA` = SysId **kA** (volts per m/s²)
   - Optionally copy the same values into the **Tuning** tab (Drive KS, KV, KA) so you can tweak at runtime without recompiling.

7. **Rebuild and redeploy** so path following and any feedforward-based commands use the new gains.

## Units (for SysId tool)

- Use **meters** for distance and **seconds** for time so the reported gains are in:
  - **kS:** V  
  - **kV:** V⋅s/m  
  - **kA:** V⋅s²/m  

These match the units used in `Constants.DriveConstants` and in `DriveSubsystem` / PathPlanner.

## References

- [WPILib: Characterizing your robot drive](https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/characterizing-drive.html)  
- [WPILib: System Identification](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html)  
- [Creating a SysId routine](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/creating-routine.html)
