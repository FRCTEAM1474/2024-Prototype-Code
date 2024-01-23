package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooterprototypesubsystem;

public class shooterprototypecommand extends CommandBase {
    double m_direction;
    
    public shooterprototypecommand(double direction){
        m_direction = direction;
        //m_topposition = topposition;
    }
    

    @Override
    public void initialize() {
        
        }

    @Override
    public void execute() {

        shooterprototypesubsystem.setspeedofShooterPrototypeMotor(m_direction);
        
    }
    @Override
    public void end(boolean interup){
        shooterprototypesubsystem.setspeedofShooterPrototypeMotor(0);
    }
}