package frc.robot.Util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.lib.BLine.FollowPath.Builder;
import frc.robot.lib.BLine.Path;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class LoadedPath {
  private Command pathCommand = Commands.none();
  private Alert pathAlert;
  private String pathName = "";
  public boolean pathAvalible = false;
  public Supplier<Pose2d> startingPose = () -> Pose2d.kZero;

  public LoadedPath(String name, boolean avalible, Command command, Supplier<Pose2d> startPose) {
    pathName = name;
    pathAvalible = avalible;
    pathCommand = command;
    startingPose = startPose;
    pathAlert = new Alert(pathName + " FAILED to load", AlertType.kWarning);
    pathAlert.set(!avalible);
  }

  public LoadedPath(String name, boolean avalible) {
    pathName = name;
    pathAvalible = avalible;
    pathAlert = new Alert(pathName + " FAILED to load", AlertType.kWarning);
    pathAlert.set(!avalible);
  }

  public Command getPathCommand() {
    if (pathAvalible) {
      return pathCommand.withName(pathName);
    } else {
      return Commands.none();
    }
  }

  public static LoadedPath pathplannerPath(String name, BooleanSupplier shouldMirror) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(name);
      return new LoadedPath(
          name,
          true,
          new ConditionalCommand(
              AutoBuilder.followPath(path.mirrorPath()), AutoBuilder.followPath(path), shouldMirror),
          () -> {
            if (shouldMirror.getAsBoolean()) {
              return path.mirrorPath().getStartingHolonomicPose().get();
            } else {
              return path.getStartingHolonomicPose().get();
            }
          });
    } catch (Exception e) {
      return new LoadedPath(name, false);
    }
  }

  public static LoadedPath bLinePath(String name, Builder builder) {
    try {
      Path path = new Path(name);
      return new LoadedPath(name, true, builder.build(path), () -> path.getStartPose());
    } catch (Exception e) {
      return new LoadedPath(name, false);
    }
  }

  public static LoadedPath tryAll(String name, Builder builder, BooleanSupplier shouldMirror) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(name);
      return new LoadedPath(
          name,
          true,
          new ConditionalCommand(
              AutoBuilder.followPath(path.mirrorPath()), AutoBuilder.followPath(path), shouldMirror),
          () -> {
            if (shouldMirror.getAsBoolean()) {
              return path.mirrorPath().getStartingHolonomicPose().get();
            } else {
              return path.getStartingHolonomicPose().get();
            }
          });
    } catch (Exception e) {
      try {
        Path path = new Path(name);
        return new LoadedPath(name, true, builder.build(path), () -> path.getStartPose());
      } catch (Exception f) {
        return new LoadedPath(name, false);
      }
    }
  }
}
