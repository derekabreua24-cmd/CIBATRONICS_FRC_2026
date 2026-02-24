// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * NO añada variables estáticas a esta clase, ni inicializaciones aquí. A menos que sepa lo que
 * hace, no modifique este archivo salvo para cambiar la clase pasado a startRobot.
 */
public final class Main {
  private Main() {}

  /**
   * Función principal de inicialización. No realice inicializaciones aquí.
   *
   * <p>Si cambia la clase principal del robot, actualice el tipo de parámetro.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
