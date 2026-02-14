// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Launcher;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

class Flywheel extends SubsystemBase {

  private TalonFX leftFlywheelMotor = new TalonFX(Constants.leftFlyweelId, CANBus.roboRIO());
  private TalonFX rightFlywheelMotor = new TalonFX(Constants.rightFlywheelId, CANBus.roboRIO());

  private double motorToFlywheelRatio = 15/18;

  private ControlRequest velocityControlRequest = new VelocityTorqueCurrentFOC(motorToFlywheelRatio);


  /** Creates a new Flywheel. */
  public Flywheel() {

    leftFlywheelMotor.getConfigurator().apply(motorConfig());
    rightFlywheelMotor.getConfigurator().apply(motorConfig());
    rightFlywheelMotor.setControl(new Follower(Constants.leftFlyweelId, MotorAlignmentValue.Opposed));
  }

  private TalonFXConfiguration motorConfig() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(120)
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(60);
    
    motorConfig.MotorOutput
      .withNeutralMode(NeutralModeValue.Coast)
      .withInverted(InvertedValue.Clockwise_Positive);

    motorConfig.ClosedLoopGeneral
      .withContinuousWrap(false);
    
    motorConfig.Feedback.withSensorToMechanismRatio(motorToFlywheelRatio);

    return motorConfig;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
