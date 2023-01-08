package frc.robot;
import frc.robot.auto.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {
  private ExampleSubsystem example;
  private Intake intake;
  SendableChooser<SequentialCommandGroup> autoSelector;
  public RobotContainer(ShuffleboardTab tab) {
    intake = new Intake();

    example = new ExampleSubsystem();
    autoSelector = new SendableChooser<SequentialCommandGroup>();
    autoSelector.setDefaultOption("Baseline", new Baseline());
    autoSelector.addOption("Proto Routine", new ProtoRoutine(drivetrain));
    setDefaults();
  }
  
  public void configureButtonBindings(XboxController driver, XboxController codriver) {
    Trigger aButtonDriver = new Trigger(() -> { // make the A button
      return driver.getAButton();
    });
    aButtonDriver.whileTrue(new RunIntake(intake));
  }

  public Command getAutonomousCommand() {
    return autoSelector.getSelected();
  }

  public void setDefaults() {
    example.setDefaultCommand(new ExampleCommand()); // constantly run the example command
  }
}