package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConeIntake;



public class CIntake extends CommandBase {
    private ConeIntake coneIntake;

    public CIntake(ConeIntake _coneIntake){
        coneIntake = _coneIntake;
    }


    public void initialize() {
        boolean outTake = coneIntake.outTake;
        if(!outTake){
            coneIntake.intake();
        }
        else{
            coneIntake.outtake();
        }
    }
    public void execute() {
       
        
    }
    public boolean isFinished() {
        return true;
    }
    public void end(boolean interrupted) {
        
    }
}
