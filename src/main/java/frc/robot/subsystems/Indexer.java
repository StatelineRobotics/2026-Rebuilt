// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;

@Logged
public class Indexer extends SubsystemBase {

  private SparkFlex ovalMotor = new SparkFlex(Constants.ovalId, MotorType.kBrushless);
  private SparkFlexSim ovalSim = new SparkFlexSim(ovalMotor, DCMotor.getNeoVortex(1));
  private SparkFlex kickerMotor = new SparkFlex(Constants.kickerId, MotorType.kBrushless);
  private Trigger isStalled = new Trigger(this::likelyStalled);

  /** Creates a new Indexer. */
  public Indexer() {
    ovalMotor.configure(
        motorConfig().inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kickerMotor.configure(motorConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setDefaultCommand(idleCommand());
  }

  private SparkFlexConfig motorConfig() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kCoast).smartCurrentLimit(80);
    return config;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    ovalSim.iterate(0, 12, 0.02);
  }

  private boolean likelyStalled() {
    return ovalMotor.getOutputCurrent() > 75.0
        && Math.abs(ovalMotor.getEncoder().getVelocity()) < 10.0;
  }

  public Command idleCommand() {
    return startRun(
        () -> {
          ovalMotor.stopMotor();
          kickerMotor.stopMotor();
        },
        () -> {});
  }

  public Command runIndexer() {
    return startRun(
        () -> {
          ovalMotor.setVoltage(9.0);
          kickerMotor.setVoltage(9.0);
        },
        () -> {});
  }

  public Command reverseIndexer() {
    return startRun(
        () -> {
          ovalMotor.setVoltage(-9.0);
          kickerMotor.setVoltage(-9.0);
        },
        () -> {});
  }

  public Command reverseRunIndexer(BooleanSupplier condition) {
    return reverseIndexer().until(isStalled).andThen(runIndexer());
  }

  public Command antiJamRun() {
    return (runIndexer().until(isStalled).andThen(reverseIndexer().withTimeout(0.5))).repeatedly();
  }
}
