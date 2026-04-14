package frc.robot.Util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FollowPath.Builder;
import frc.robot.lib.BLine.Path;

public class LoadedPath {
    private Command pathCommand = Commands.none();
    private Alert pathAlert;
    private String pathName = "";
    public boolean pathAvalible = false;
    
    public LoadedPath(String name, boolean avalible, Command command) {
        pathName = name;
        pathAvalible = avalible;
        pathCommand = command;
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

    public static LoadedPath pathplannerPath(String name) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(name);
            return new LoadedPath(name, true, AutoBuilder.followPath(path));
        } catch (Exception e) {
            return new LoadedPath(name, false);
        }
    }

    public static LoadedPath bLinePath(Builder builder, String name) {
        try {
            Path path = new Path(name);
            return new LoadedPath(name, true, builder.build(path));
        } catch (Exception e) {
            return new LoadedPath(name, true);
        }
    }
}
