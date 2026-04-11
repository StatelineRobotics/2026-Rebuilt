// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Launcher;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import java.util.function.Supplier;

@Logged
class Hood extends SubsystemBase {
  /** Creates a new Hood. */
  private TalonFX hoodMotor = new TalonFX(Constants.hoodId, TunerConstants.kCANBus);

  private TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

  private PositionVoltage positionRequest = new PositionVoltage(0);
  private VoltageOut voltageRequest = new VoltageOut(0);
  private NeutralOut neutralOut = new NeutralOut();

  private StatusSignal<AngularVelocity> velocitySignal = hoodMotor.getVelocity();
  private StatusSignal<Current> statorCurrentSignal = hoodMotor.getStatorCurrent();
  private StatusSignal<Angle> angleSignal = hoodMotor.getPosition();
  private StatusSignalCollection signalList =
      new StatusSignalCollection(velocitySignal, statorCurrentSignal, angleSignal);

  private static final double motorToHoodRatio = (46.0 / 16.0) * (162.0 / 20.0); // 2.875 * 8.1 = 23.2875
  private static final Angle hoodMin = Degree.of(15.0);
  private static final Angle hoodMax = Degree.of(48.0);

  private NetworkTableEntry hoodEntry = NetworkTableInstance.getDefault().getEntry("/tuning/hoodTarget");
  private NetworkTableEntry hoodOffset = NetworkTableInstance.getDefault().getEntry("/adjustments/hoodOffset");

  Alert zeroedState = new Alert("Hood Failed To Zero", AlertType.kWarning);

  Trigger stalled = new Trigger(this::isStalled);

  public Hood() {
    hoodEntry.getTopic().genericPublish("double");
    hoodEntry.getTopic().setPersistent(true);
    hoodOffset.getTopic().genericPublish("double");
    hoodOffset.getTopic().setPersistent(true);

    hoodConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Coast);
    hoodConfig.Feedback.withSensorToMechanismRatio(motorToHoodRatio);
    hoodConfig
        .CurrentLimits
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(60)
        .withSupplyCurrentLimit(20);
    hoodConfig
        .SoftwareLimitSwitch
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(hoodMax)
        .withReverseSoftLimitThreshold(hoodMin);
    hoodConfig.Slot0.withKP(120.0).withKD(0.0).withKS(0.38).withKV(0.0).withKG(0.0);

    hoodMotor.getConfigurator().apply(hoodConfig);
    hoodMotor.setPosition(Degrees.of(15));

    signalList.setUpdateFrequencyForAll(50);
    hoodMotor.optimizeBusUtilization();

    // SmartDashboard.putData("HoodCommand", targetDashboardAngle());
    SmartDashboard.putData("ZeroHoodCommand", zeroHood());

    setDefaultCommand(targetAngle(() -> Degrees.of(15.0)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    signalList.refreshAll();
  }

  private boolean isStalled() {
    return velocitySignal.getValueAsDouble() < 1.0 && statorCurrentSignal.getValueAsDouble() > 20.0;
  }

  public boolean atTarget() {
    return positionRequest.getPositionMeasure().isNear(angleSignal.getValue(), Degree.of(3.0));
  }

  protected Command zeroHood() {
    return new ParallelRaceGroup(
        startEnd(
                () -> hoodMotor.setControl(
                    voltageRequest.withOutput(-1.0).withIgnoreSoftwareLimits(true)),
                () -> {
                  hoodMotor.setControl(neutralOut);
                  hoodMotor.setPosition(hoodMin);
                })
            .until(stalled.debounce(0.25))
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming),
        new WaitCommand(5.0).finallyDo((interupted) -> {
          zeroedState.set(!interupted);
        }));
  }

  protected Command targetAngle(Supplier<Angle> targetAngle) {
    return run(() -> hoodMotor.setControl(
        positionRequest.withPosition(targetAngle.get().plus(Degrees.of(hoodOffset.getDouble(0.0))))));
  }

  protected Command targetDashboardAngle() {
    return run(() -> hoodMotor.setControl(positionRequest.withPosition(Degrees.of(hoodEntry.getDouble(15)))));
  }
}
