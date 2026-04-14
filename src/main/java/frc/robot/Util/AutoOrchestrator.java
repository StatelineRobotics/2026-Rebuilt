package frc.robot.Util;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FollowPath.Builder;

@Logged
public class AutoOrchestrator {
    Builder bLineBuilder;

    private SendableChooser<Command> initialChooser = new SendableChooser<Command>();
    private String[] initialPathOptions = {
        "p_TrenchShort",
        "p_TrenchMid",
        "p_TrenchFar"
    };

    public AutoOrchestrator(Builder builder) {
        bLineBuilder = builder;
        buildPathChooser(initialChooser, "initialPath", initialPathOptions);
    }

    private void buildPathChooser(SendableChooser<Command> chooser, String chooserName, String[] names) {
        chooser.setDefaultOption("none", Commands.none());
        for (String name : names) {
            LoadedPath path;
            if (name.startsWith("p_")) {
                path = LoadedPath.pathplannerPath(name);
            } else if (name.startsWith("b_")) {
                path = LoadedPath.bLinePath(bLineBuilder, name);
            } else {
                System.out.println("WARNING Tried to load path but missing prefix");
                continue;
            }
            if (path.pathAvalible) {
                chooser.addOption(name, path.getPathCommand());
            }
            
        }
        SmartDashboard.putData(chooserName, chooser);
    }
}
