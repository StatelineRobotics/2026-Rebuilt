// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class Intake extends SubsystemBase {
  
  private SparkFlex pivotMotor = new SparkFlex(Constants.pivotId, MotorType.kBrushless);
  private SparkAbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();
  private SparkClosedLoopController pivotController = pivotMotor.getClosedLoopController();
  private TalonFX rollerMotor = new TalonFX(Constants.rollerId, CANBus.roboRIO());

  private static final double storePosition = 0.25;
  private static final double deployPosition = 0.0;


  /** Creates a new Intake. */
  public Intake() {
    SparkFlexConfig pivotConfig = new SparkFlexConfig();
    pivotConfig
      .idleMode(IdleMode.kCoast)
      .inverted(false)
      .smartCurrentLimit(20);
    pivotConfig.absoluteEncoder
      .inverted(false)
      .positionConversionFactor(0.5)
      .velocityConversionFactor(0.5)
      .zeroCentered(false);
    pivotConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .p(0.0)
      .d(0.0)
      .feedForward
        .kA(0.0)
        .kV(0.0)
        .kS(0)
        .kG(0.0);
    pivotConfig.closedLoop.maxMotion
      .cruiseVelocity(10)
      .maxAcceleration(20);
    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput
      .withInverted(InvertedValue.Clockwise_Positive)
      .withNeutralMode(NeutralModeValue.Coast);
    rollerConfig.CurrentLimits
      .withSupplyCurrentLimitEnable(true)
      .withStatorCurrentLimitEnable(true)
      .withSupplyCurrentLimit(20)
      .withStatorCurrentLimit(40);
    
    rollerMotor.getConfigurator().apply(rollerConfig);
    


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public Command storeCommand() {
    return startRun(
      () -> {
        pivotController.setSetpoint(storePosition, ControlType.kMAXMotionPositionControl);
        rollerMotor.setControl(new NeutralOut());
      }, 
      () -> {});
  }

  public Command deployCommand() {
    return startRun(
      () -> {
        pivotController.setSetpoint(deployPosition, ControlType.kMAXMotionPositionControl);
        rollerMotor.setControl(new NeutralOut());
      }, 
      () -> {});
  }

  public Command IntakeCommand() {
    return startRun(
      () -> {
        pivotController.setSetpoint(deployPosition, ControlType.kMAXMotionPositionControl);
        rollerMotor.setControl(new VoltageOut(4.0));
      }, 
      () -> {});
  }
}
