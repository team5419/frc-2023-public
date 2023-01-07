package frc.robot;
import frc.robot.auto.*;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

class RobotContainer {
  public RobotContainer(ShuffleboardTab tab) {
    setDefaults();
  }
  
  public void configureButtonBindings(XboxController driver, XboxController codriver) {
  }

  public Command getAutonomousCommand() {
    return new Baseline();
  }

  public void setDefaults() {
  }
}