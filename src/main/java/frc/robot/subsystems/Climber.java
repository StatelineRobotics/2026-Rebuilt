// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class Climber extends SubsystemBase {

  private SparkFlex climberMotor = new SparkFlex(Constants.climberId, MotorType.kBrushless);

  /** Creates a new Climber. */
  @SuppressWarnings("removal")
  public Climber() {

 SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kCoast)
          .smartCurrentLimit(40);
          config.closedLoop.pid(0, 0, 0);
          config.closedLoop.maxMotion.cruiseVelocity(0).maxAcceleration(0);

 climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }

  @Override
  public void periodic() {
  }

   public Command idleCommand() {
    return startRun(() -> {
        climberMotor.stopMotor();
         }, () -> {});
  }

  public Command runClimbCommand(double setPoint) {
    return run(() -> climberMotor.getClosedLoopController().setSetpoint(setPoint, ControlType.kMAXMotionPositionControl));

  }
}
