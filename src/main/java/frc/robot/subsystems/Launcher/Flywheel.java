// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Launcher;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import java.util.function.DoubleSupplier;

@Logged
public class Flywheel extends SubsystemBase {

  private static final double motorToFlywheelRatio = 18.0 / 15.0;

  private TalonFX leftFlywheelMotor = new TalonFX(Constants.leftFlyweelId, TunerConstants.kCANBus);
  private TalonFXSimState simState = leftFlywheelMotor.getSimState();
  private TalonFX rightFlywheelMotor = new TalonFX(Constants.rightFlywheelId, TunerConstants.kCANBus);

  private DCMotorSim dcSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(2), 0.0023377, motorToFlywheelRatio),
      DCMotor.getKrakenX60Foc(2));

  private VelocityVoltage velocityControlRequest = new VelocityVoltage(motorToFlywheelRatio).withSlot(0);
  private NeutralOut neutralRequest = new NeutralOut();
  private VoltageOut voltageRequest = new VoltageOut(0.0);

  private NetworkTableEntry speedEntry = NetworkTableInstance.getDefault().getEntry("/tuning/flywheelTarget");
  private NetworkTableEntry speedOffset = NetworkTableInstance.getDefault().getEntry("/adjustments/flywheelOffset");

  /** Creates a new Flywheel. */
  public Flywheel() {

    speedEntry.getTopic().genericPublish("double");
    speedEntry.getTopic().setPersistent(true);
    speedOffset.getTopic().genericPublish("double");
    speedOffset.getTopic().setPersistent(true);

    leftFlywheelMotor.getConfigurator().apply(motorConfig());
    rightFlywheelMotor.getConfigurator().apply(motorConfig());
    rightFlywheelMotor.setControl(new Follower(Constants.leftFlyweelId, MotorAlignmentValue.Opposed));

    simState.Orientation = ChassisReference.CounterClockwise_Positive;
    simState.setSupplyVoltage(Volts.of(12));
    simState.setMotorType(MotorType.KrakenX60);

    // SmartDashboard.putData("tuning/flywheelRun", runAtDashboardVelocity());
    setDefaultCommand(idleCommand());
  }

  private TalonFXConfiguration motorConfig() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig
        .CurrentLimits
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(80)
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(40);

    motorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast);

    motorConfig.Feedback.withSensorToMechanismRatio(motorToFlywheelRatio);

    motorConfig.Slot0.withKP(2.0).withKD(0.0).withKS(0.31).withKV(0.145);

    return motorConfig;
  }

  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, // Use default ramp rate (1 V/s)
          Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
          null, // Use default timeout (10 s)
          // Log state with Phoenix SignalLogger class
          (state) -> SignalLogger.writeString("state", state.toString())),
      new SysIdRoutine.Mechanism(
          (volts) -> leftFlywheelMotor.setControl(voltageRequest.withOutput(volts.in(Volts))), null, this));

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void simulationPeriodic() {
    return;
    // get the motor voltage of the TalonFX
    // var motorVoltage = simState.getMotorVoltageMeasure();

    // // use the motor voltage to calculate new position and velocity
    // // using WPILib's DCMotorSim class for physics simulation
    // dcSim.setInputVoltage(motorVoltage.in(Volts));
    // dcSim.update(0.020); // assume 20 ms loop time

    // // apply the new rotor position and velocity to the TalonFX;
    // // note that this is rotor position/velocity (before gear ratio), but
    // // DCMotorSim returns mechanism position/velocity (after gear ratio)
    // simState.setRotorVelocity(dcSim.getAngularVelocity().times(motorToFlywheelRatio));
    // simState.setRawRotorPosition(dcSim.getAngularPosition().times(motorToFlywheelRatio));
  }

  public boolean atTarget() {
    return velocityControlRequest
        .getVelocityMeasure()
        .isNear(leftFlywheelMotor.getVelocity().getValue(), RotationsPerSecond.of(4));
  }

  protected Command idleCommand() {
    return startRun(() -> leftFlywheelMotor.setControl(neutralRequest), () -> {});
  }

  protected Command runAtVelocity(DoubleSupplier desiredVelocity) {
    return run(() -> leftFlywheelMotor.setControl(
        velocityControlRequest.withVelocity(desiredVelocity.getAsDouble() + speedOffset.getDouble(0.0))));
  }

  protected Command runAtDashboardVelocity() {
    return run(() -> leftFlywheelMotor.setControl(velocityControlRequest.withVelocity(speedEntry.getDouble(0))));
  }
}
