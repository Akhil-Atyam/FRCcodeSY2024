package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase{
    private static Flywheel instance;
    public static synchronized Flywheel getInstance(){
        if (instance == null) instance = new Flywheel(); {
            return instance;
        }
    }
    TalonFX motor;
    public Flywheel(){
        motor = new TalonFX(5, "rio");
    }
    public void run(double percent){
        motor.set(percent);
    }
}
