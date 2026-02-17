// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Launcher;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

@Logged
class Turret extends SubsystemBase {

  private TalonFX turretMotor = new TalonFX(Constants.turretId, TunerConstants.kCANBus);

  private PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

  private static final Angle minRotation = Rotations.of(-0.5);
  private static final Angle maxRotation = Rotations.of(0.5);
  private static final double gearRatio = 5.0 * (100.0 / 10.0);


  /** Creates a new Turret. */
  public Turret() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput
      .withInverted(InvertedValue.Clockwise_Positive)
      .withNeutralMode(NeutralModeValue.Coast);
    config.CurrentLimits
      .withStatorCurrentLimitEnable(true)
      .withSupplyCurrentLimitEnable(true)
      .withStatorCurrentLimit(120)
      .withSupplyCurrentLimit(40);
    config.ClosedLoopGeneral.withContinuousWrap(false);
    config.Feedback.withSensorToMechanismRatio(gearRatio);
    config.SoftwareLimitSwitch
      .withForwardSoftLimitEnable(true)
      .withReverseSoftLimitEnable(true)
      .withForwardSoftLimitThreshold(maxRotation)
      .withReverseSoftLimitThreshold(minRotation);
    config.Slot0
      .withKP(0.0)
      .withKD(0.0)
      .withKS(0.0)
      .withKV(0.0);

    turretMotor.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
