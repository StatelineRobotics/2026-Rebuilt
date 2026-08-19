// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

@Logged
class Turret extends SubsystemBase {

  private TalonFX turretMotor = new TalonFX(Constants.turretId, TunerConstants.kCANBus);
  private CANcoder smallEncoder = new CANcoder(Constants.smallTurretEncoderId, TunerConstants.kCANBus);
  private CANcoder largeEncoder = new CANcoder(Constants.largeTurretEncoderId, TunerConstants.kCANBus);

  private PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

  private static final double turretZeroOffset = -0.946289;
  private static final Angle minRotation = Rotations.of(turretZeroOffset);
  private static final Angle maxRotation = Rotations.of(0.85);
  private static final double gearRatio = 3.0 * (100.0 / 10.0);
  private static final double largeEncoderTeeth = 14.0;
  private static final double smallEncoderTeeth = 13.0;
  private static final double turretTeeth = 100.0;

  private Alert zeroedAlert = new Alert("Turret Failed To Zero", AlertType.kError);

  private NetworkTableEntry turretEntry = NetworkTableInstance.getDefault().getEntry("/tuning/turretTarget");
  private NetworkTableEntry turretOffset = NetworkTableInstance.getDefault().getEntry("/adjustments/turretOffset");

  public AngularVelocity requestTargetVelocity = positionRequest.getVelocityMeasure();
  public Angle requestTargetangle = positionRequest.getPositionMeasure();

  public BooleanSupplier safeSpin = () -> true;

  /** Creates a new Turret. */
  public Turret() {
    turretEntry.getTopic().genericPublish("double");

    turretEntry.getTopic().setPersistent(true);
    turretOffset.getTopic().genericPublish("double");
    turretOffset.getTopic().setPersistent(true);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Coast);
    config.CurrentLimits.withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(120)
        .withSupplyCurrentLimit(40);
    config.ClosedLoopGeneral.withContinuousWrap(false);
    config.Feedback.withSensorToMechanismRatio(gearRatio);
    config.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(maxRotation)
        .withReverseSoftLimitThreshold(minRotation);
    config.Slot0.withKP(170.0).withKD(5.0).withKS(0.65).withKV(3.5);

    turretMotor.getConfigurator().apply(config);

    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1.0);
    smallEncoder
        .getConfigurator()
        .apply(encoderConfig
            .MagnetSensor
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withMagnetOffset(-0.821044921875));
    largeEncoder
        .getConfigurator()
        .apply(encoderConfig
            .MagnetSensor
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            .withMagnetOffset(-0.0537109375));

    if (Robot.isSimulation()) {
      smallEncoder.setPosition(Math.abs(turretZeroOffset) * (turretTeeth / smallEncoderTeeth) % 1.0);
      largeEncoder.setPosition(Math.abs(turretZeroOffset) * (turretTeeth / largeEncoderTeeth) % 1.0);
    }

    turretMotor.setPosition(calculateCRTangle());
    // turretMotor.setPosition(0.0);
    // SmartDashboard.putData("TargetTurret", targetDashboardAngle());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    requestTargetVelocity = positionRequest.getVelocityMeasure();
    requestTargetangle = positionRequest.getPositionMeasure();
  }

  public Angle getRotation() {
    return turretMotor.getPosition(true).getValue();
  }

  protected Command targetAngle(Supplier<Angle> targetAngle) {
    return Commands.waitUntil(safeSpin)
        .andThen(run(() -> turretMotor.setControl(positionRequest.withPosition(
            wrapTargetAngle(targetAngle.get().plus(Degrees.of(turretOffset.getDouble(0.0))))))));
  }

  protected Command targetAngleWithVelocity(Supplier<Angle> targetAngle, Supplier<AngularVelocity> targetVelocity) {
    return run(() -> turretMotor.setControl(positionRequest
        .withPosition(wrapTargetAngle(targetAngle.get().plus(Degrees.of(turretOffset.getDouble(0.0)))))
        .withVelocity(targetVelocity.get())));
  }

  public boolean atTarget() {
    return positionRequest
        .getPositionMeasure()
        .isNear(turretMotor.getPosition().getValue(), Degrees.of(2.0));
  }

  private Angle calculateCRTangle() {
    double e1Angle = smallEncoder.getAbsolutePosition(true).getValueAsDouble();
    double e2Angle = largeEncoder.getAbsolutePosition(true).getValueAsDouble();

    double[] possibleE1Angles = new double[(int) (largeEncoderTeeth)];
    for (int i = 0; i < largeEncoderTeeth; i++) {
      double value = (i + e1Angle) * (smallEncoderTeeth / turretTeeth);
      possibleE1Angles[i] = value;
    }

    for (int i = 0; i < smallEncoderTeeth; i++) {
      double value = (i + e2Angle) * (largeEncoderTeeth / turretTeeth);
      for (double e1Value : possibleE1Angles) {
        if (Math.abs(value - e1Value) <= 0.003) {
          zeroedAlert.set(false);
          return Rotations.of(value + turretZeroOffset);
        }
      }
    }

    zeroedAlert.set(true);
    return Rotations.of(0);
  }

  // Assume target angle is within a single rotation [-0.5 , 0.5]
  private Angle wrapTargetAngle(Angle targetAngle) {
    double target = targetAngle.in(Rotations);
    double min = minRotation.in(Rotations);
    double max = maxRotation.in(Rotations);
    double largerTarget = target + 1.0;
    double smallerTarget = target - 1.0;
    double current = getRotation().in(Rotations);

    if (target < 0.0) {
      if (largerTarget < max && Math.abs(largerTarget - current) < Math.abs(target - current)) {
        return Rotations.of(largerTarget);
      } else {
        return Rotations.of(target);
      }
    } else {
      if (smallerTarget > min && Math.abs(smallerTarget - current) < Math.abs(target - current)) {
        return Rotations.of(smallerTarget);
      } else {
        return Rotations.of(target);
      }
    }
  }

  public Command targetDashboardAngle() {
    return targetAngle(() -> Degrees.of(turretEntry.getDouble(0)));
  }
}
