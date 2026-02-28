package frc.robot.commands;
import frc.robot.subsystems.LimelightSubsystem;
public class LimelightMoveCommand {
    public final LimelightSubsystem m_LimelightSubsystem = new LimelightSubsystem();

    public void getMoveBool(){
        m_LimelightSubsystem.TAmove("limelight");
    }

    

}
