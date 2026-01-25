package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoChooser {
  private final SendableChooser<Command> autoChooser;

  public AutoChooser(Command defaultAuto, Command... autoCommands) {
    autoChooser = new SendableChooser<>();

    // Set default autonomous command
    autoChooser.setDefaultOption("Default Auto", defaultAuto);
    autoChooser.addOption("Do Nothing", null);
    // Add additional autonomous commands
    for (Command auto : autoCommands) {
      autoChooser.addOption(auto.getName(), auto);
    }

    // Send to SmartDashboard
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  public Command getSelectedAuto() {
    return autoChooser.getSelected();
  }
}
