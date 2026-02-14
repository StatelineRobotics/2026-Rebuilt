// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Launcher;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants;

@Logged
class Hood extends SubsystemBase {
  /** Creates a new Hood. */

  private TalonFX hoodMotor = new TalonFX(Constants.hoodId);
  private TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

  private ControlRequest positionRequest = new PositionVoltage(0);
  private ControlRequest voltageRequest = new VoltageOut(0);

  private StatusSignal<AngularVelocity> velocitySignal = hoodMotor.getVelocity();
  private StatusSignal<Current> statorCurrentSignal = hoodMotor.getStatorCurrent();

  private static final double motorToHoodRatio = 1.0/1.0;
  private static final double hoodPositionOffset = 0.0;

  Alert zeroedState = new Alert("Hood Failed To Zero", AlertType.kWarning);

  public Hood() {
    
    hoodConfig.MotorOutput
      .withInverted(InvertedValue.Clockwise_Positive)
      .withNeutralMode(NeutralModeValue.Coast);
    hoodConfig.Feedback
      .withSensorToMechanismRatio(motorToHoodRatio);
    hoodConfig.CurrentLimits
      .withStatorCurrentLimitEnable(true)
      .withSupplyCurrentLimitEnable(true)
      .withStatorCurrentLimit(60)
      .withSupplyCurrentLimit(30);
    hoodConfig.Slot0
      .withKP(0.0)
      .withKD(0.0)
      .withKS(.0)
      .withKV(0.0)
      .withKG(0.0);

    hoodMotor.getConfigurator().apply(hoodConfig);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private boolean isStalled() {
    return velocitySignal.getValueAsDouble() < 1.0 && statorCurrentSignal.getValueAsDouble() > 20.0;
  }

  
  protected Command zeroHoodCommand() {
    return new ParallelRaceGroup(
      startEnd(
          () -> hoodMotor.setControl(new VoltageOut(-2.0)), 
          () -> {
            hoodMotor.setControl(new NeutralOut());
            hoodMotor.setPosition(hoodPositionOffset);
          })
        .until(this::isStalled)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming),
      new WaitCommand(5.0)
        .finallyDo((interupted) -> {zeroedState.set(!interupted);})
      
    );
  
  }
}
