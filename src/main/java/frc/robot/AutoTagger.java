// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.BooleanSupplier;

/** Add your docs here. */
public class AutoTagger {
  private PathPlannerPath depotPath;
  private PathPlannerPath humanPath;
  private Alert depotAlert = new Alert("Depot Path NOT Found", AlertType.kWarning);
  private Alert humanAlert = new Alert("Depot Path NOT Found", AlertType.kWarning);
  private Command shootCommand;
  private Command agitateCommand;

  private Pose2d targetPose = new Pose2d();

  private SendableChooser<Command> tagChooser = new SendableChooser<>();

  public PathConstraints constraints =
      new PathConstraints(1.5, 1.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
  public PathConstraints sotmConstraints =
      new PathConstraints(1.0, 0.5, Units.degreesToRadians(0), Units.degreesToRadians(0));

  public PathPlannerPath leftBump;
  public PathPlannerPath leftStart;
  private Alert leftBumpAlert = new Alert("Left Bump NOT Found", AlertType.kWarning);

  private BooleanSupplier shouldMirror;

  public AutoTagger(CommandSwerveDrivetrain drivetrain, BooleanSupplier mirror, Command shoot, Command agitate) {
    PathPlannerLogging.setLogTargetPoseCallback(this::setTarget);
    shootCommand = shoot;
    shouldMirror = mirror;
    agitateCommand = agitate;
    var idleRequest = new SwerveRequest.Idle();
    tagChooser.setDefaultOption("None", drivetrain.applyRequest(() -> idleRequest));
    tagChooser.addOption("ShootToend", shoot().alongWith(agitateCommand.asProxy()));

    try {
      depotPath = PathPlannerPath.fromPathFile("Depot");
      tagChooser.addOption("Depot", getDepot());
    } catch (Exception e) {
      depotAlert.set(true);
    }
    try {
      humanPath = PathPlannerPath.fromPathFile("Human Player");
      tagChooser.addOption("Human Player", getHumanPlayer());
    } catch (Exception e) {
      humanAlert.set(true);
    }

    try {
      leftStart = PathPlannerPath.fromPathFile("SecondSweep");
    } catch (Exception e) {
      humanAlert.set(true);
    }

    try {
      leftBump = PathPlannerPath.fromPathFile("Left Bump");
    } catch (Exception e) {
      leftBumpAlert.set(true);
    }
  }

  private void setTarget(Pose2d target) {
    targetPose = target;
  }

  public SendableChooser<Command> getChosser() {
    return tagChooser;
  }

  private Command getDepot() {
    return AutoBuilder.pathfindThenFollowPath(depotPath, constraints)
        .andThen(shootCommand.asProxy())
        .withName("DepotTag");
  }

  private Command getHumanPlayer() {
    return AutoBuilder.pathfindThenFollowPath(humanPath, constraints)
        .andThen(shootCommand.asProxy())
        .withName("HumanTag");
  }

  private Command flippingPathFollowingCommand(PathPlannerPath path) {
    return new ConditionalCommand(
        AutoBuilder.pathfindThenFollowPath(humanPath.mirrorPath(), constraints),
        AutoBuilder.pathfindThenFollowPath(humanPath, constraints),
        shouldMirror);
  }

  private Command getLeftZone() {
    return AutoBuilder.pathfindToPose(leftStart.getStartingHolonomicPose().get(), constraints);
  }

  private Command getRightZone() {
    return AutoBuilder.pathfindToPose(
        leftStart.mirrorPath().getStartingHolonomicPose().get(), constraints);
  }

  public Command navigateToTrenchShot() {
    return new ConditionalCommand(getRightZone(), getLeftZone(), shouldMirror);
  }

  private Command shoot() {
    return shootCommand.asProxy().withName("ShootToEnd");
  }
}
