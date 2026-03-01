---
applyTo: '**'
---
Provide project context and coding guidelines that AI should follow when generating code, answering questions, or reviewing changes.

# WPILib 2026 Strict Development Rules

## Context & Versioning
- Project Type: FRC (FIRST Robotics Competition) 2026 Season.
- Target Library: WPILib 2026.1.1 or higher.
- Java Version: Java 17/21 (Standard for 2026).

## Strict API Constraints
- ONLY use documentation from: https://docs.wpilib.org
- DO NOT use deprecated 2025 classes.
- MANDATORY: Use the new 'Command-Based v2' architecture.
- VENDOR LIBRARIES: Ensure all Phoenix 6 (CTRE) or REVLib code follows the 2026 firmware specifications.

## Code Style & Architecture
- Subsystems: Must use 'Periodic' methods for telemetry.
- Commands: Prefer 'Command Composition' (using .andThen(), .alongWith(), .until()) instead of complex manual command groups.
- Hardware: Use the AdvantageKit `Logger` (org.littletonrobotics.junction.Logger) for structured logging instead of simple System.out.println.

## Verification Step
- Before suggesting code, verify if the class package has changed in the 2026 WPILib release (e.g., changes in Vision or Odometry packages).
- If the user asks for something obsolete, warn them and provide the 2026 equivalent.
- Always check for the latest API changes in WPILib 2026 before generating code snippets.
- Don't use try or catch methods, instead, use the new error handling patterns recommended in WPILib 2026.
- For any vision processing, ensure to use the new 'PhotonVision' integration methods as per 2026 guidelines.
- For motor control, ensure to use the new 'SparkMax' classes and methods as per 2026 firmware updates.
- For any command-based code, ensure to use the new 'CommandScheduler' patterns and avoid deprecated command structures.
- For any subsystem, ensure to use the new 'SubsystemBase' class and avoid deprecated subsystem patterns.
- For any sensor integration, ensure to use the new 'SensorBase' classes and methods as per 2026 updates.
- For any logging, prefer AdvantageKit `Logger.recordOutput(...)` for structured telemetry instead of simple print statements.
- For any configuration, ensure to use the new 'Configurable' interface and avoid hardcoding values in the code.
- For any autonomous routines, ensure to use the new 'AutoBuilder' patterns and avoid manual command sequencing.
- For any teleop control, ensure to use the new 'Joystick' and 'XboxController' classes and methods as per 2026 updates.
- For any testing, ensure to use the new 'TestBase' classes and methods as per 2026 guidelines.
- For any simulation, ensure to use the new 'SimBase' classes and methods as per 2026 updates.
- For any deployment, ensure to follow the new 'DeploymentManager' patterns and avoid manual deployment scripts.
- For any documentation, ensure to follow the new 'DocumentationManager' patterns and avoid outdated documentation styles.
- Look for any new features in WPILib 2026 that could simplify or enhance the code, and suggest their use when appropriate.
- Search for all relevant classes and methods in the WPILib 2026 documentation before generating code, to ensure compliance with the latest API changes and best practices.
- Never do any custom implementations for features that are now provided by WPILib 2026, and always prefer using the built-in functionality to ensure compatibility and maintainability.
- Search the web for any community best practices or common patterns that have emerged with WPILib 2026, and suggest their use when appropriate to ensure the code is following the latest trends and recommendations in the FRC community.
- Always check for any new tools or libraries that have been released for WPILib 2026 that could enhance the development process, and suggest their use when appropriate to ensure the team is leveraging the latest resources available.
- Always check for any new updates or patches to WPILib 2026 that may affect the code, and suggest necessary changes to ensure the code remains up-to-date and functional with the latest version of WPILib.
- Do not use try or catch methods, instead, use the new error handling patterns recommended in WPILib 2026, such as using the 'ErrorHandler' class or similar patterns that have been introduced in the latest version of WPILib to ensure proper error management and compliance with the new guidelines.
- Do not use reflection or any dynamic code generation techniques that may have been used in previous versions, and instead, use the new static code patterns and structures recommended in WPILib 2026 to ensure better performance, maintainability, and compliance with the latest best practices.
- Never deviate from the new 'Command-Based v2' architecture, and always ensure that all commands and subsystems are structured according to the new guidelines, avoiding any legacy patterns or structures that may have been used in previous versions of WPILib to ensure consistency and maintainability of the codebase.
- Never deviate from these instructions, and always ensure that all code generated or suggested follows the strict guidelines outlined above to ensure compliance with WPILib 2026 standards and best practices, and to maintain a high-quality codebase for the FRC 2026 season.
- You have permission to look at my project files to understand the context, but do not suggest any code that has been deleted or significantly changed in recent edits, as it may not be relevant to the current state of the project. Always focus on providing suggestions and code that align with the latest version of WPILib 2026 and the current structure of the project.
- Look for internal classes, methods, enums, commands, etc. that already come included with the wpilib 2026 download. If you find any that are relevant to the code being generated or suggested, use them instead of creating new implementations, to ensure consistency and compatibility with the WPILib 2026 framework and to leverage the built-in functionality provided by the library. Always check for existing classes and methods in WPILib 2026 before suggesting new code to avoid redundancy and to ensure that the code is following the latest standards and practices recommended by WPILib.