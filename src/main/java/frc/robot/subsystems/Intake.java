// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import java.util.function.BooleanSupplier;

@Logged
public class Intake extends SubsystemBase {

  private SparkFlex pivotMotor = new SparkFlex(Constants.pivotId, MotorType.kBrushless);
  private SparkAbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();
  private SparkClosedLoopController pivotController = pivotMotor.getClosedLoopController();
  private TalonFX rollerMotor = new TalonFX(Constants.rollerId, TunerConstants.kCANBus);
  private TalonFX otherRollerMotor = new TalonFX(27, TunerConstants.kCANBus);
  private TalonFX lowerRollerMotor = new TalonFX(Constants.lowerRollerID, TunerConstants.kCANBus);

  private NeutralOut neutralRequest = new NeutralOut();
  private VoltageOut voltageRequest = new VoltageOut(0);

  private static final double pivotRatio = 25.0 * (60.0 / 24.0) * 0.5; // planetary * gears * chain

  private static final double storePosition = 0.2;
  private static final double intakePosition = -0.18;
  private static final double deployPosition = 0.0;

  private final Alert absoluteEncoderAlert = new Alert("Intake Not Start In Expected Position", AlertType.kWarning);
  private boolean lastUseValue = false;

  private SparkFlexConfig otfConfig = new SparkFlexConfig();

  /** Creates a new Intake. */
  public Intake() {
    SparkFlexConfig pivotConfig = new SparkFlexConfig();
    pivotConfig.idleMode(IdleMode.kCoast).inverted(false).smartCurrentLimit(20);
    pivotConfig.encoder.positionConversionFactor(pivotRatio).velocityConversionFactor(pivotRatio);
    pivotConfig
        .absoluteEncoder
        .inverted(true)
        .positionConversionFactor(0.5)
        .velocityConversionFactor(0.5)
        .zeroCentered(true);
    pivotConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(40.0)
        .d(0.2)
        .positionWrappingEnabled(false);
    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig
        .MotorOutput
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast);
    rollerConfig
        .CurrentLimits
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(30)
        .withStatorCurrentLimit(80);

    rollerMotor.getConfigurator().apply(rollerConfig);
    otherRollerMotor.getConfigurator().apply(rollerConfig);
    lowerRollerMotor
        .getConfigurator()
        .apply(rollerConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive));

    pivotMotor.getEncoder().setPosition(storePosition);

    if (Math.abs(storePosition - pivotMotor.getAbsoluteEncoder().getPosition()) > 0.05) {
      absoluteEncoderAlert.set(true);
    }

    otherRollerMotor.setControl(new Follower(Constants.rollerId, MotorAlignmentValue.Opposed));

    SmartDashboard.putBoolean("UsePivotInternal", false);
  }

  @Override
  public void periodic() {
    var newUseValue = SmartDashboard.getBoolean("UsePivotInternal", false);
    if (lastUseValue != newUseValue) {
      lastUseValue = newUseValue;
      otfConfig.closedLoop.feedbackSensor(
          lastUseValue ? FeedbackSensor.kPrimaryEncoder : FeedbackSensor.kAbsoluteEncoder);

      pivotMotor.configureAsync(otfConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
  }

  public Command storeCommand() {
    return startRun(
        () -> {
          pivotController.setSetpoint(storePosition, ControlType.kPosition);
          rollerMotor.setControl(neutralRequest);
          lowerRollerMotor.setControl(neutralRequest);
        },
        () -> {});
  }

  public Command deployCommand() {
    return startRun(
        () -> {
          pivotController.setSetpoint(intakePosition, ControlType.kPosition);
          rollerMotor.setControl(neutralRequest);
          lowerRollerMotor.setControl(neutralRequest);
        },
        () -> {});
  }

  public Command agitate() {
    return startRun(
            () -> {
              pivotController.setSetpoint(deployPosition, ControlType.kPosition);
              rollerMotor.setControl(voltageRequest.withOutput(2.0));
              lowerRollerMotor.setControl(neutralRequest);
            },
            () -> {})
        .withName("agitate");
  }

  public Command idleDeployed() {
    return startRun(
        () -> {
          pivotController.setSetpoint(intakePosition, ControlType.kPosition);
          rollerMotor.setControl(neutralRequest);
          lowerRollerMotor.setControl(neutralRequest);
        },
        () -> {});
  }

  public Command autoAgitate() {
    return Commands.waitSeconds(2.0).andThen(agitate());
  }

  public Command intakeCommand() {
    return startRun(
        () -> {
          pivotController.setSetpoint(intakePosition, ControlType.kPosition);
          rollerMotor.setControl(voltageRequest.withOutput(6.0));
          lowerRollerMotor.setControl(voltageRequest.withOutput(7.0));
        },
        () -> {});
  }

  public Command reverseIntake() {
    return startRun(
        () -> {
          pivotController.setSetpoint(intakePosition, ControlType.kPosition);
          rollerMotor.setControl(voltageRequest.withOutput(-8.0));
          lowerRollerMotor.setControl(voltageRequest.withOutput(-9.0));
        },
        () -> {});
  }

  public Command reverseRunIntake(BooleanSupplier condition) {
    return reverseIntake().until(condition).andThen(intakeCommand());
  }
}
