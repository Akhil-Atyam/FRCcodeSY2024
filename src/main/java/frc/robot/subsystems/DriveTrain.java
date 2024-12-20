// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//Imports etc
package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


//Creates the class
public class DriveTrain extends SubsystemBase {
  private static DriveTrain instance = null;

  /**
   * Singleton Constructor for {@link Swerve}
   * @return Single instance of {@link Swerve} common to all contexts.
   */
  public static synchronized DriveTrain getInstance() {
    if (instance == null) instance = new DriveTrain();
    return instance;
  }
  //Initializes robot size
  public final double L = Constants.robot.A_LENGTH;
  public final double W = Constants.robot.A_WIDTH;
  //creates the pid controllers for field centric and also some other things related towards it
  public PIDController rotationalPID = Constants.drivetrain.ROT_PID;
  public PIDController correctionalPID = Constants.drivetrain.CORRECTION_PID;
  //Creates speeds from the chassis speeds class.
  ChassisSpeeds speeds;
  //This creates a new swerve kinematics and inputs the motor locations.
  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      Constants.drivetrain.FL_LOCATION, Constants.drivetrain.FR_LOCATION, Constants.drivetrain.BL_LOCATION, Constants.drivetrain.BR_LOCATION
  );
  //Makes a new pigeon
  public Pigeon2 pigeon = new Pigeon2(Constants.ids.PIGEON);


  /**
  * THIS IS THE DRIVE COMMAND. IMPORT THIS INTO YOUR COMMAND AND IT WILL DRIVE.
  * You can just put raw joystick vals, the yjoystick is inverted in the function so you dont have to worry about that.
  * You can also put desired angle so that robot follows an angle.
  * @param xjoystick
  * @param yjoystick
  * @param thethajoystick
  */
  public void Drive(double xjoystick,double yjoystick,double thethajoystick) {

    //This boolean checks if the robot is on the blue or red team. 
    boolean isBlue= DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == Alliance.Blue : false;
    //The boolean is used in this. If it is on blue, it is on the other side of the field, so reverse (set to 180) ideal heading because that is what field centric is following.
    Rotation2d robotIdealHeading;
    if (isBlue){
      robotIdealHeading = Rotation2d.fromDegrees(180);
    }else {
      robotIdealHeading = Rotation2d.fromDegrees(0);
    }
    //Inverts y Joystick.
    yjoystick *= -1;
    
    //Updates the values of speeds and makes the field centric, remove .fromFieldRelativeSpeeds to make it robot centric.
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xjoystick*5,yjoystick*5,thethajoystick*2*Math.PI,robotIdealHeading);
    //Converts chassis speeds into module states.
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(correctSpeeds());
    //Applies the module states to all of the modules using for loop
      for (int i = 0; i<4;i++){
        moduleList[i].setModuleState(moduleStates[i]);
      }

  }
  //Creates the swerve modules using params.
  private SwerveModule[] moduleList = {
    new SwerveModule(
      "FL",
      Constants.ids.FL_SPEED,
      Constants.ids.FL_ANGLE,
      Constants.ids.FL_ENCODER,
      Constants.drivetrain.FL_ZERO,
      Constants.drivetrain.FL_LOCATION
    ),
    new SwerveModule(
      "FR",
      Constants.ids.FR_SPEED,
      Constants.ids.FR_ANGLE,
      Constants.ids.FR_ENCODER,
      Constants.drivetrain.FR_ZERO,
      Constants.drivetrain.FR_LOCATION
    ),
    new SwerveModule(
      "BL",
      Constants.ids.BL_SPEED,
      Constants.ids.BL_ANGLE,
      Constants.ids.BL_ENCODER,
      Constants.drivetrain.BL_ZERO,
      Constants.drivetrain.BL_LOCATION
    ),
    new SwerveModule(
      "BR",
      Constants.ids.BR_SPEED,
      Constants.ids.BR_ANGLE,
      Constants.ids.BR_ENCODER,
      Constants.drivetrain.BR_ZERO,
      Constants.drivetrain.BR_LOCATION
    )
  };
  //This is a pigeon corrector.
  public ChassisSpeeds correctSpeeds(){
    //This makes it so at first you get the previously requested speeds.
    ChassisSpeeds correctedSpeeds = speeds;
    //This checks the angular velocity of the robot.
    if (speeds.omegaRadiansPerSecond != 0.0) {
      //Makes sure the robot doesnt overshoot using pid.
      correctionalPID.setSetpoint(pigeon.getRotation2d().getDegrees());
    } else if (speeds.vxMetersPerSecond != 0 || speeds.vyMetersPerSecond != 0) {
      //If the robot is moving at a speed, then do this condition
      //Find correction value
      double correction = correctionalPID.calculate(pigeon.getRotation2d().getDegrees());
      //Set correction pid and omega radians per second
      correctedSpeeds.omegaRadiansPerSecond = correctionalPID.atSetpoint() ? 0.0 : correction;  
    }
    //Converts into states and returns the speeds.
    m_kinematics.toSwerveModuleStates(correctedSpeeds);
    return correctedSpeeds;
  }
  /**
   * Please note: The radians calculation will be done in the method. PLEASE INPUT AN ANGLE, NOT RADIANS. 
   * This function turns the robot to an angle you specify. Pretty simple imo.
   * @param angleValue
   */
  public void turnToAngle(double angleValue){
    ChassisSpeeds angle = new ChassisSpeeds(0,0,rotationalPID.calculate(Math.toRadians(angleValue)));
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(angle);
    //Applies the module states to all of the modules using for loop
      for (int i = 0; i<4;i++){
        moduleList[i].setModuleState(moduleStates[i]);
      }

  }
  //Stops motors, basically just a brake command.
  public void stop(){
    for (int i = 0; i<4;i++){
      moduleList[i].stopMotors();
    }
    System.out.print("Drivetrain Stopped");
  }
}
