package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.XboxController;

public class Drive extends CommandBase {
    private Drivetrain drivetrain;
    private XboxController driver;
   
   public Drive(XboxController _driver){
    drivetrain = new Drivetrain();
    driver = _driver;

   }
   
    public void initialize() {

    }
    public void execute() { 
        drivetrain.drive(driver.getLeftY());

    }
    public boolean isFinished() {
        return true;
    }
    public void end(boolean interrupted) {
        
    }
}
