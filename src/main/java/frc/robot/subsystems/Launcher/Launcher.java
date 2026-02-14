// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Launcher;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Launcher extends SubsystemBase {


  private final Flywheel flywheel = new Flywheel();
  private final Hood hood = new Hood();
  private final Turret turret = new Turret();

  /** Creates a new Launcher. */
  public Launcher() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
