package frc.robot;
import frc.robot.auto.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.PneumaticHub;

public class RobotContainer {
  private Intake intake;
  private Indexer indexer;
  private IntakeDeploy deploy;
  private Vision vision;
  private SendableChooser<SequentialCommandGroup> autoSelector;
  private Drivetrain drivetrain;
  // private Swerve swerve;
  private XboxController driver;
  private XboxController codriver;
  //private Claw claw;
  private Coner coner;

  private EverybotArm arm;

  private PneumaticHub hub;

  public RobotContainer(ShuffleboardTab tab) {
    driver = new XboxController(0);
    codriver = new XboxController(1);
    vision = new Vision(tab, false, true);
    coner = new Coner(tab, true);
    intake = new Intake(Shuffleboard.getTab("Intake"));
    indexer = new Indexer(Shuffleboard.getTab("Indexer"));
    //claw = new Claw();

    hub = new PneumaticHub();

    arm = new EverybotArm(/*this.hub*/);

    // swerve = new Swerve(vision, true); /* CHOOSE ONE!!! */
    drivetrain = new Drivetrain(); /* ^^^ */

    deploy = new IntakeDeploy(this.hub);

    autoSelector = new SendableChooser<SequentialCommandGroup>();
    tab.add("Auto selector", autoSelector);
    autoSelector.setDefaultOption("Baseline", new Baseline());
    //sautoSelector.addOption("Proto Routine", new ProtoRoutine(drivetrain));
    // autoSelector.addOption("Proto Routine", new SwerveRoutine(swerve, vision));
    setDefaults();
    configureButtonBindings(driver, codriver);
  }
  
  public void configureButtonBindings(XboxController driver, XboxController codriver) {
    Trigger aButtonDriver = new Trigger(() -> driver.getAButton());
    Trigger bButtonDriver = new Trigger(() -> driver.getBButton());
    Trigger xButtonDriver = new Trigger(() -> driver.getXButton());
    Trigger yButtonDriver = new Trigger(() -> driver.getYButton());

    Trigger leftBumper = new Trigger(() -> driver.getLeftBumper());
    Trigger rightBumper = new Trigger(() -> driver.getRightBumper());

    Trigger dPadDown = new Trigger(() -> ((driver.getPOV() >= 180 && driver.getPOV() < 315) && driver.getPOV() != -1));
    Trigger dPadUp = new Trigger(() -> ((driver.getPOV() >= 315 || driver.getPOV() < 45) && driver.getPOV() != -1));
    Trigger dPadRight = new Trigger(() -> ((driver.getPOV() >= 45 && driver.getPOV() < 180) && driver.getPOV() != -1));

    //Trigger aButtonCodriver = new Trigger(() -> codriver.getAButton());
    // leftBumper.whileTrue(Commands.runEnd(() -> { swerve.slowMode = true; }, () -> { swerve.slowMode = false; }));
    // aButtonDriver.whileTrue(new RunIntake(intake));
    // bButtonDriver.onTrue(new SpecialRamseteSwerve(swerve, vision, true));
    // leftBumper.whileTrue(new RunIntake(intake, indexer));

    // rightBumper.whileTrue(new Shoot(intake, indexer));

    dPadUp.onTrue(new EverybotIntake(this.arm, false));
    dPadRight.onTrue(new EverybotSuction(this.arm));
    dPadDown.onTrue(new EverybotIntake(this.arm, true));

    leftBumper.whileTrue(new DriveCommand(drivetrain, driver, true));

    xButtonDriver.whileTrue(Commands.runEnd(() -> { coner.top.intake(); coner.bottom.intake(); }, 
    () -> { coner.top.stop(); coner.bottom.stop(); }, coner));

    aButtonDriver.whileTrue(Commands.runEnd(() -> { coner.top.outtakeMid(); coner.bottom.outtakeMid(); }, 
    () -> { coner.top.stop(); coner.bottom.stop(); }, coner));

    yButtonDriver.whileTrue(Commands.runEnd(() -> { coner.top.outtakeHigh(); coner.bottom.outtakeHigh(); }, 
    () -> { coner.top.stop(); coner.bottom.stop(); }, coner));

    // bButtonDriver.onTrue(deploy.twoPhase());
    // bButtonDriver.onTrue(new Balance(swerve, driver));
    //aButtonCodriver.toggleOnTrue(Commands.runOnce(() -> swerve.brake()));
  }

  public Command getAutonomousCommand() {
    return autoSelector.getSelected();
    //return new ProtoRoutine(drivetrain);
  }

  public void setDefaults() {
    drivetrain.setDefaultCommand(new DriveCommand(drivetrain, driver, false));
    // swerve.setDefaultCommand(new SwerveDrive(swerve, driver));
    //arm.setDefaultCommand(new Arm(this.arm, this.driver));
  }
}