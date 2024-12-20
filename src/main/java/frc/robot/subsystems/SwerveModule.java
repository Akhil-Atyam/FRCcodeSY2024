package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase{
    // Basic values that way we can identify the module.
    // Like the location and the starting point (zero)
    //Module name is never used anywhere really, but is convenient to identify the name when making the array.
    private final String MODULE_NAME;
    private final int SPEED_ID;
    private final int ANGLE_ID;
    private final int ENCODER_ID;
    private final double ZERO;
    // Creating the variables into motors.
    private TalonFX speedMotor;
    private TalonFX angleMotor;
    // Creating the variable into an encoder.
    private CANcoder angleEncoder;
    // Variables into feedforward and pid controllers
    private PIDController anglePID;
    private PIDController speedPID;
    private SimpleMotorFeedforward speedFF;
    
    public SwerveModule(String name,int DriveID, int AngleID, int EncoderID, double zero, Translation2d location){
        this.MODULE_NAME = name;
        this.SPEED_ID = DriveID;
        this.ANGLE_ID = AngleID;
        this.ZERO = zero;
        this.ENCODER_ID = EncoderID;
        this.anglePID = Constants.drivetrain.ANGLE_PID;
        this.speedPID = Constants.drivetrain.SPEED_PID;
        this.speedMotor = new TalonFX(SPEED_ID, "rio");
        this.angleMotor = new TalonFX(ANGLE_ID, "rio");
        this.angleEncoder = new CANcoder(ENCODER_ID, "rio");
        this.speedFF = new SimpleMotorFeedforward(Constants.drivetrain.SPEED_FF.ks, Constants.drivetrain.SPEED_FF.kv);


    }
    public String getName(){
        return this.MODULE_NAME;
    }
    public void init(){
        try {
            // Speed Motor related setup. If no input then brake, ramp time in open and closed loops.
        speedMotor.setNeutralMode(NeutralModeValue.Brake);
        OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
        openLoopRampsConfigs.VoltageOpenLoopRampPeriod = Constants.drivetrain.RAMP_RATE;
        speedMotor.getConfigurator().apply(openLoopRampsConfigs);
        ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
        closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = Constants.drivetrain.RAMP_RATE;
        speedMotor.getConfigurator().apply(closedLoopRampsConfigs);
        // Angle Motor related setup. If no input then brake, ramp time in open and closed loops.
        angleMotor.setNeutralMode(NeutralModeValue.Brake);
        OpenLoopRampsConfigs openLoopRampsConfigsAngle = new OpenLoopRampsConfigs();
        openLoopRampsConfigsAngle.VoltageOpenLoopRampPeriod = Constants.drivetrain.RAMP_RATE;
        angleMotor.getConfigurator().apply(openLoopRampsConfigs);
        ClosedLoopRampsConfigs closedLoopRampsConfigsAngle = new ClosedLoopRampsConfigs();
        closedLoopRampsConfigsAngle.VoltageClosedLoopRampPeriod = Constants.drivetrain.RAMP_RATE;
        angleMotor.getConfigurator().apply(closedLoopRampsConfigs);
        //Clearing sticky faults. This is because whenever voltage hits <6.5V, an error occours. This resets it.
        speedMotor.clearStickyFaults();
        angleEncoder.clearStickyFaults();
        angleMotor.clearStickyFaults();
        //Magnet sensor config, basically the encoder thingy thing
        MagnetSensorConfigs sensorConfigs = new MagnetSensorConfigs();
        sensorConfigs.MagnetOffset = -(ZERO / 360.0);
        sensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        sensorConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        
        //Applying these configs to the encoder
        angleEncoder.getConfigurator().apply(sensorConfigs);
        angleMotor.getConfigurator().setPosition(angleEncoder.getAbsolutePosition().getValueAsDouble() * 360.0);

        //Essentially setting pid to a circle because degrees are from a circle and thats how circles work. also making sure no slippages so tolerance = 0
        anglePID.enableContinuousInput(-180, 180);
        anglePID.setTolerance(0);    
        } catch (Exception e) {
            // TODO: handle exception
            System.out.print("Init failure "+ MODULE_NAME + e.getMessage() + e.getStackTrace());
            System.out.print("You should probably fix this.");
        }
        



    }



    public void stopMotors(){
        try {
            angleMotor.set(0);
            speedMotor.set(0);
        } catch (Exception e) {
            // TODO: handle exception
            System.out.print("Motor stop failure "+ MODULE_NAME + e.getMessage() + e.getStackTrace());
            System.out.print("You should probably fix this.");
        }
    }

    public void setInverted(){
        angleMotor.setInverted(false);
    }
    public void setModuleState(SwerveModuleState desired) {
        try {
            SwerveModuleState optimized =
            SwerveModuleState.optimize(desired, Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValueAsDouble() * 360.0));
        
        double velocity = optimized.speedMetersPerSecond / Constants.drivetrain.DRIVE_METERS_PER_TICK;
        System.out.print("speed "+speedPID.calculate(getVelocity(), velocity) + speedFF.calculate(velocity));
        speedMotor.setVoltage(speedPID.calculate(getVelocity(), velocity) + speedFF.calculate(velocity));
        System.out.print("angle "+anglePID.calculate(angleEncoder.getAbsolutePosition().getValueAsDouble() * 360.0, optimized.angle.getDegrees()));
        angleMotor.setVoltage(anglePID.calculate(angleEncoder.getAbsolutePosition().getValueAsDouble() * 360.0, optimized.angle.getDegrees()));
        } catch (Exception e) {
            // TODO: handle exception
            System.out.print("Module Speed Set failure "+ MODULE_NAME + e.getMessage() + e.getStackTrace());
            System.out.print("You should probably fix this.");
        }
        
    }
    public void setPower(double anglePower, double speedPower){
        angleMotor.set(anglePower);
        speedMotor.set(speedPower);
    }
    private double getVelocity() { // RPM to MPS
        return (speedMotor.getVelocity().getValueAsDouble() * Constants.drivetrain.WHEEL_CIRCUMFRENCE) / Constants.drivetrain.DRIVE_GEARING;
      }
}
 