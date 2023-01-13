package frc.robot.modules;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ISwerveModule {
    public double getDrive();
    public Rotation2d getTurn();
    public SwerveModuleState getState();
    public void setDesiredState(SwerveModuleState desiredState, boolean preventTurn, boolean slow, boolean pid);
    public void simulationPeriodic(double dt);
    public void resetEncoders();
    public void setBrakeMode(boolean on);
    public void test();
}