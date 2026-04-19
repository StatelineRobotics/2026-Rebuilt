package frc.robot.Util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.AutoCommands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.FollowPath.Builder;
import java.util.function.BooleanSupplier;

@Logged
public class AutoOrchestrator {
  private Builder bLineBuilder;
  private AutoCommands autoCommands;

  private BooleanSupplier shouldMirror;

  private SendableChooser<Command> startOptions;
  private SendableChooser<Command> initialPath;
  private SendableChooser<Command> secondStage;
  private SendableChooser<Command> thirdStage;
  private SendableChooser<Command> fourthStage;
  private SendableChooser<Command> fithStage;
  private SendableChooser<Command> sixthStage;
  // private SendableChooser<Command> tags;
  private String[] initialPathOptions = {
    "trenchShort", "trenchMid", "trenchFar", "stackedTrenchShort", "stackedTrenchMid", "stackedTrenchFar"
  };
  private String[] secondStageOptions = {"returnBump", "returnTrench", "frontDepot", "sideDepot"};
  private String[] thirdStageOptions = {"frontDepot", "sideDepot", "CurveToTrench", "trenchShort"};
  private String[] fourthStageOptions = {"secondSwipe"};
  private String[] fithStageOptions = {"returnBump", "returnTrench"};
  private String[] sixthStageOptions = {"frontDepot", "sideDepot", "CurveToTrench"};

  public AutoOrchestrator(Builder builder, AutoCommands commands, BooleanSupplier mirror) {
    shouldMirror = mirror;
    bLineBuilder = builder;
    autoCommands = commands;
    buildAutoCommands();
    startOptions = buildInitialOptions();
    initialPath = buildResetingPathChooser("initialPath", initialPathOptions);
    secondStage = buildPathChooser("Second Stage", secondStageOptions);
    thirdStage = buildPathChooser("Third Stage", thirdStageOptions);
    fourthStage = buildPathChooser("Fourth Stage", fourthStageOptions);
    fithStage = buildPathChooser("fith stage", fithStageOptions);
    sixthStage = buildPathChooser("sixth Stage", sixthStageOptions);
    // tags = buildTags();
    SmartDashboard.putNumber("Auto Delay", 0.0);
  }

  public void buildAutoCommands() {
    FollowPath.registerEventTrigger("shoot", autoCommands.shoot());
    FollowPath.registerEventTrigger("stopShoot", autoCommands.stopShooting());
    FollowPath.registerEventTrigger("prepShoot", autoCommands.prepShooter());
    FollowPath.registerEventTrigger("intake", autoCommands.intake());
    FollowPath.registerEventTrigger("liftIntake", autoCommands.intakeLift());
    new EventTrigger("runIntake").onTrue(autoCommands.intake());
    new EventTrigger("shoot").whileTrue(autoCommands.shoot());
  }

  private SendableChooser<Command> buildResetingPathChooser(String chooserName, String[] names) {
    SendableChooser<Command> chooser = new SendableChooser<Command>();
    chooser.setDefaultOption("none", Commands.none());
    for (String name : names) {
      LoadedPath path = LoadedPath.tryAll(name, bLineBuilder, shouldMirror);
      if (path.pathAvalible) {
        chooser.addOption(
            name,
            Commands.deferredProxy(() -> AutoBuilder.resetOdom(path.startingPose.get()))
                .andThen(path.getPathCommand()));
      }
    }
    SmartDashboard.putData(chooserName, chooser);
    return chooser;
  }

  private SendableChooser<Command> buildPathChooser(String chooserName, String[] names) {
    SendableChooser<Command> chooser = new SendableChooser<Command>();
    chooser.setDefaultOption("none", Commands.none());
    for (String name : names) {
      LoadedPath path = LoadedPath.tryAll(name, bLineBuilder, shouldMirror);
      if (path.pathAvalible) {
        chooser.addOption(name, path.getPathCommand());
      }
    }
    SmartDashboard.putData(chooserName, chooser);
    return chooser;
  }

  private SendableChooser<Command> buildInitialOptions() {
    SendableChooser<Command> chooser = new SendableChooser<Command>();
    chooser.setDefaultOption("None", Commands.none());
    chooser.addOption(
        "Wait", Commands.deferredProxy(() -> Commands.waitSeconds(SmartDashboard.getNumber("Auto Delay", 0))));
    chooser.addOption(
        "Shoot Then Wait",
        Commands.deferredProxy(() -> Commands.waitSeconds(SmartDashboard.getNumber("Auto Delay", 0)))
            .andThen(autoCommands
                .intake()
                .alongWith(Commands.waitSeconds(0.5)
                    .andThen(autoCommands.shoot().withTimeout(5.0)))));
    SmartDashboard.putData("Start Options", chooser);
    return chooser;
  }

  // public SendableChooser<Command> buildTags() {
  //   SendableChooser<Command> chooser = new SendableChooser<Command>();
  //   chooser.setDefaultOption("None", Commands.none());
  //   chooser.addOption("Shoot", autoCommands.shoot());
  //   SmartDashboard.putData("Tag", chooser);
  //   return chooser;
  // }

  public Command getAutocommand() {
    return Commands.sequence(
        startOptions.getSelected(),
        initialPath.getSelected(),
        secondStage.getSelected(),
        thirdStage.getSelected(),
        fourthStage.getSelected(),
        fithStage.getSelected(),
        sixthStage.getSelected());
  }

  private Command deferEverything(Command... commands) {
    var commandSequence = Commands.none();
    for (Command command : commands) {
      commandSequence = commandSequence.andThen(Commands.deferredProxy(() -> command));
    }
    return commandSequence;
  }
}
