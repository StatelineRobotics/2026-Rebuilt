package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher.Launcher;

public class AutoCommands {
  Launcher launcher;
  Intake intake;
  Indexer indexer;
  CommandSwerveDrivetrain drivetrain;

  public AutoCommands(CommandSwerveDrivetrain drivetrain, Launcher launcher, Intake intake, Indexer indexer) {
    this.launcher = launcher;
    this.drivetrain = drivetrain;
    this.intake = intake;
    this.indexer = indexer;
  }

  public Command shoot() {
    return launcher.targetHub()
        .alongWith(Commands.waitUntil(launcher.launcherReady).andThen(indexer.antiJamRun()))
        .asProxy();
  }

  public Command prepShooter() {
    return launcher.targetHub().asProxy();
  }

  public Command stopShooting() {
    return indexer.idleCommand().alongWith(launcher.runToZero()).asProxy();
  }

  public Command intake() {
    return intake.intakeCommand().asProxy();
  }
}
