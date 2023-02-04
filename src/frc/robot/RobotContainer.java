package frc.robot;
import frc.robot.Constants.ConerTypes;
import frc.robot.Constants.CuberTypes;
import frc.robot.auto.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {
    private GenericShootIntake cuber;
    private IntakeDeploy deploy;
    private Vision vision;
    private SendableChooser<Supplier<SequentialCommandGroup>> autoSelector;
    private SendableChooser<ConerTypes> coneShooterSelector;
    private SendableChooser<CuberTypes> cubeShooterSelector;
  	//private Drivetrain drivetrain;
  	private Swerve swerve;
  	private XboxController driver;
  	private XboxController codriver;
  	//private Claw claw;
  	private GenericShootIntake coner;
  	private EverybotArm arm;

  	public RobotContainer(ShuffleboardTab tab) {
    	driver = new XboxController(0);
    	codriver = new XboxController(1);
    	vision = new Vision(tab, false, true);
    
    	//claw = new Claw();
    	arm = null; 
    	coner = null;
    	cuber = null; // to be set up later
    
    	swerve = new Swerve(vision, true); /* CHOOSE ONE!!! */
    	//drivetrain = new Drivetrain(); /* ^^^ */

    	deploy = new IntakeDeploy();
		
    	autoSelector = new SendableChooser<Supplier<SequentialCommandGroup>>();
    	tab.add("Auto selector", autoSelector);
    	autoSelector.setDefaultOption("Baseline", () -> new Baseline());
    	//sautoSelector.addOption("Proto Routine", new ProtoRoutine(drivetrain));
    	// autoSelector.addOption("Proto Routine", new SwerveRoutine(swerve, vision));

		coneShooterSelector = new SendableChooser<ConerTypes>();
		tab.add("Cone shooter", coneShooterSelector);
		coneShooterSelector.setDefaultOption("Low Coner", ConerTypes.LOW);
		coneShooterSelector.addOption("Everybot arm w/ motors", ConerTypes.ARM_MOTORS);
		coneShooterSelector.addOption("Everybot arm w/ suction", ConerTypes.ARM_SUCTION);

		cubeShooterSelector = new SendableChooser<CuberTypes>();
		tab.add("Cube shooter", cubeShooterSelector);
		cubeShooterSelector.setDefaultOption("Low Cuber", CuberTypes.LOW);
  	}

	private EverybotArm generateArm() {
		if(arm == null) {
			arm = new EverybotArm(false);
		}
		return arm;
	}

	public void setUpShooters() {
		ConerTypes coneType = coneShooterSelector.getSelected();
		switch(coneType) {
			case LOW:
				coner = new Coner(true, false);
				break;
			case ARM_MOTORS:
				coner = new EverybotConer(generateArm(), false);
				break;
			case ARM_SUCTION:
				coner = new Suction(generateArm());
				break;
		}
		CuberTypes cubeType = cubeShooterSelector.getSelected();
		switch(cubeType) {
			case LOW:
				cuber = new Cuber(false);
				break;
		}
	}
  
  	public void configureButtonBindings() {
    	Trigger aButtonDriver = new Trigger(() -> driver.getAButton());
    	Trigger bButtonDriver = new Trigger(() -> driver.getBButton());
    	Trigger xButtonDriver = new Trigger(() -> driver.getXButton());
    	Trigger yButtonDriver = new Trigger(() -> driver.getYButton());

    	Trigger leftBumper = new Trigger(() -> driver.getLeftBumper());
    	Trigger rightBumper = new Trigger(() -> driver.getRightBumper());

    	leftBumper.whileTrue(Commands.runEnd(() -> { swerve.slowMode = true; }, () -> { swerve.slowMode = false; }));
    
		// eventually, this is what the code will look like: 
		// aButtonDriver.onTrue(new AlignSetup(coner, cuber, swerve, vision, driver));
		// bButtonDriver.whileTrue(new Shoot(coner, cuber, swerve));
		
		// xButtonDriver.whileTrue(new RunIntake(coner));
		// yButtonDriver.whileTrue(new RunIntake(cuber));

		// for testing:
		aButtonDriver.whileTrue(new Prep(cuber, cuber, swerve));
		bButtonDriver.whileTrue(new Shoot(cuber, cuber, swerve));
		
		//bButtonDriver.onTrue(new SpecialRamseteSwerve(swerve, vision, driver, true));
		//rightBumper.toggleOnTrue(Commands.runEnd(() -> arm.gotoPosition(Arm.outPosition), () -> arm.gotoPosition(Arm.inPosition)))
		// bButtonDriver.onTrue(deploy.twoPhase());
		// bButtonDriver.onTrue(new Balance(swerve, driver));
		//aButtonCodriver.toggleOnTrue(Commands.runOnce(() -> swerve.brake()));
  	}

	public Command getAutonomousCommand() {
		return autoSelector.getSelected().get();
	}

	public void setDefaults() {
		swerve.setDefaultCommand(new SwerveDrive(swerve, driver));
		if(arm != null) {
			arm.setDefaultCommand(new ManualMoveArm(arm, codriver));
		}
	}
}