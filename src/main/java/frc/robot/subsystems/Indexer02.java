// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.PrivateKey;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer02 extends SubsystemBase {

  private SparkFlex ovalMotor = new SparkFlex(Constants.ovalId, MotorType.kBrushless);
  private SparkFlex kickerMotor = new SparkFlex(Constants.kickerId, MotorType.kBrushless);

  /** Creates a new Indexer02. */
  public Indexer02() {
    ovalMotor.configure(motorConfig().inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kickerMotor.configure(motorConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   }

   private SparkFlexConfig motorConfig() {
      SparkFlexConfig config = new SparkFlexConfig();
      config.idleMode(IdleMode.kCoast)
      .smartCurrentLimit(40);
      return config;
    }

    public Command idleCommand () {
        return startRun(() -> {
          ovalMotor.stopMotor();
          kickerMotor.stopMotor();
        }, () -> {});
    }

    public Command runIndexer() {
      return startRun (() -> {
        ovalMotor.setVoltage(12.0);
        kickerMotor.setVoltage(12.0);
      }, () -> {});
    }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
