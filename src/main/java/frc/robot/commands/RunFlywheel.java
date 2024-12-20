package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;

public class RunFlywheel extends Command {
    Flywheel flywheel = Flywheel.getInstance();
    public RunFlywheel(){
        addRequirements(flywheel);
    }
    @Override
    public void initialize(){
        flywheel.run(0);
    }
    @Override
    public void execute(){
        flywheel.run(0.25);
    }
    @Override
    public void end(boolean interrupted){
        flywheel.run(0);
    }
    @Override
    public boolean isFinished(){
        return false;
    }
    
}
