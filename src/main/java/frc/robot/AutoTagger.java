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
import frc.robot.subsystems.CommandSwerveDrivetrain;

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
      new PathConstraints(3.5, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
  public PathConstraints sotmConstraints =
      new PathConstraints(1.0, 0.5, Units.degreesToRadians(0), Units.degreesToRadians(0));

  public PathPlannerPath leftBump;
  public PathPlannerPath leftStart;
  private Alert leftBumpAlert = new Alert("Left Bump NOT Found", AlertType.kWarning);

  public AutoTagger(CommandSwerveDrivetrain drivetrain, Command shoot, Command agitate) {
    PathPlannerLogging.setLogTargetPoseCallback(this::setTarget);
    shootCommand = shoot;
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
      leftStart = PathPlannerPath.fromPathFile("Left Trench Shot to 2nd Sweep");
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

  public Command getLeftBump() {
    return replaningPathfinding(leftBump, constraints);
  }

  public Command getLeftZone() {
    return AutoBuilder.pathfindToPose(leftStart.getStartingHolonomicPose().get(), constraints);
  }

  private Command shoot() {
    return shootCommand.asProxy().withName("ShootToEnd");
  }

  private double distanceToTarget() {
    return AutoBuilder.getCurrentPose().getTranslation().getDistance(targetPose.getTranslation());
  }

  private double distanceToStart(PathPlannerPath path) {
    return AutoBuilder.getCurrentPose()
        .getTranslation()
        .getDistance(path.getStartingHolonomicPose().get().getTranslation());
  }

  private Command replaningPathfinding(PathPlannerPath path, PathConstraints constraints) {
    return (AutoBuilder.pathfindThenFollowPath(path, constraints)
            .until(() -> distanceToTarget() > 0.1)
            .repeatedly())
        .until(() -> distanceToStart(path) < 0.1)
        .andThen(AutoBuilder.pathfindThenFollowPath(path, constraints));
  }

  // class ReplanningPath {
  //   Alert alert;
  //   PathPlannerPath path;

  //   public ReplanningPath(String pathname) {
  //     alert = new Alert(pathname + "NOT found", AlertType.kWarning);
  //     try {
  //       path = PathPlannerPath.fromPathFile(pathname);
  //       alert.set(false);
  //     } catch (Exception e) {
  //       alert.set(true);
  //     }
  //   }

  //   public Command getCommand() {
  //     if (path == null) {
  //       return Commands.idle().withTimeout(1.0);
  //     }
  //     return replaningPathfinding(path, constraints).withName(path.name);
  //   }
  // }

  // class Tag {
  //   Alert alert;
  //   PathPlannerPath path;
  //   boolean shoot;

  //   public Tag(String pathName, SendableChooser<Command> selector, boolean shootAtEnd) {
  //     alert = new Alert(pathName + "NOT found", AlertType.kWarning);
  //     shoot = shootAtEnd;
  //     try {
  //       path = PathPlannerPath.fromPathFile(pathName);
  //       selector.addOption(pathName, getCommand());
  //       alert.set(false);
  //     } catch (Exception e) {
  //       alert.set(true);
  //       e.printStackTrace();
  //     }
  //   }

  //   private Command getCommand() {
  //     Command command = AutoBuilder.pathfindThenFollowPath(humanPath, constraints);
  //     if (shoot) {
  //       command = command.andThen(shootCommand.asProxy());
  //     }
  //     return command.withName(path.name);
  //   }
  // }
}
