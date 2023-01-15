package frc.robot;
import frc.robot.auto.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {
  private Intake intake;
  private IntakeDeploy deploy;
  private Vision vision;
  private SendableChooser<SequentialCommandGroup> autoSelector;
  //private Drivetrain drivetrain;
  private Swerve swerve;
  private XboxController driver;
  private Claw claw;
  private ShuffleboardTab tab;

  public RobotContainer(ShuffleboardTab _tab) {
    tab = _tab;
    driver = new XboxController(0);
    vision = new Vision(_tab);
    intake = new Intake();
    claw = new Claw();

    swerve = new Swerve(vision); /* CHOOSE ONE!!! */
    //drivetrain = new Drivetrain(); /* ^^^ */

    deploy = new IntakeDeploy();
    autoSelector = new SendableChooser<SequentialCommandGroup>();
    autoSelector.setDefaultOption("Baseline", new Baseline());
    //autoSelector.addOption("Proto Routine", new ProtoRoutine(drivetrain));
    autoSelector.addOption("Proto Routine", new SwerveRoutine(swerve));
    setDefaults();
  }
  
  public void configureButtonBindings(XboxController driver, XboxController codriver) {
    Trigger aButtonDriver = new Trigger(() -> driver.getAButton());
    Trigger bButtonDriver = new Trigger(() -> driver.getBButton());
    Trigger xButtonDriver = new Trigger(() -> driver.getXButton());
    Trigger yButtonDriver = new Trigger(() -> driver.getYButton());

    Trigger aButtonCodriver = new Trigger(() -> codriver.getAButton());
    Trigger bButtonCodriver = new Trigger(() -> codriver.getBButton());
    Trigger yButtonCodriver = new Trigger(()-> codriver.getYButton());
    
    yButtonCodriver.onTrue(new goToPosition(tab));
    aButtonDriver.whileTrue(new RunIntake(intake));
    bButtonDriver.onTrue(deploy.twoPhase());
    xButtonDriver.onTrue(claw.twoPhase());
    yButtonDriver.toggleOnTrue(new Balance(swerve, driver));
    aButtonCodriver.toggleOnTrue(Commands.runOnce(() -> swerve.brake()));
  }

  public Command getAutonomousCommand() {
    return autoSelector.getSelected();
  }

  public void setDefaults() {
    //drivetrain.setDefaultCommand(new DriveCommand(drivetrain, driver));
    swerve.setDefaultCommand(new SwerveDrive(swerve, driver));
  }
}