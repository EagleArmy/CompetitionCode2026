package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.NeckWheelSubsystem;

public class ShooterWithNeckCommand extends Command {
        private NeckWheelSubsystem m_NeckWheelSubsystem;
    private ShooterSubsystem m_ShooterSubsystem;

    public ShooterWithNeckCommand(NeckWheelSubsystem subsystem1, ShooterSubsystem subsystem2){
        m_NeckWheelSubsystem = subsystem1;
        m_ShooterSubsystem = subsystem2;
        addRequirements(m_NeckWheelSubsystem, m_ShooterSubsystem);
    }

    @Override 
    public void initialize() {}

    @Override
    public void execute (){
        // System.out.println("Neck works with Shooter!");
        m_NeckWheelSubsystem.start();
        m_ShooterSubsystem.start();
    }

    @Override
    public void end(boolean interrupted) { m_ShooterSubsystem.stop(); System.out.println("Shooting FINISHED!");}
  
    @Override
    public boolean isFinished() {
      return false;
    }
}
