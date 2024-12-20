// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants{
    public static final int kDriverControllerPort = 0;
  }
  public static class controller {
    public static final int PILOT_PORT = 0;
    public static final int COPILOT_PORT = 1;
    public static final double DEADBAND = 0.1;
    public static final double TRIGGER_THRESHOLD = 0.6;
  }

  public static final class field {
    public static double FIELD_WIDTH = Units.feetToMeters(26.9375);
    public static double FIELD_LENGTH = Units.feetToMeters(54.2708);

    //This is the safe zone for when we pass shot!
    public static Translation2d BLUE_CORNER_LOCATION = new Translation2d(0.10, 7.4);
    public static Translation2d RED_CORNER_LOCATION = new Translation2d(16.44, 7.4);

    public static Translation3d BLUE_SPEAKER_LOCATION = new Translation3d(0.10, 5.54, 2.04);
    public static Translation3d BLUE_AMP_LOCATION = new Translation3d(1.82, 8.15, 0.89);

    public static Translation3d RED_SPEAKER_LOCATION = new Translation3d(16.44, 5.54, 2.04);
    public static Translation3d RED_AMP_LOCATION = new Translation3d(14.70, 8.15, 0.89);

    public static Translation3d RED_LEFT_TRAP = new Translation3d(11.90, 3.72, 1.42);
    public static Translation3d RED_CENTER_TRAP = new Translation3d(11.24, 4.06, 1.42);
    public static Translation3d RED_RIGHT_TRAP = new Translation3d(11.85, 4.54, 1.42);

    // public static Translation3d CURRENT_SPEAKER_LOCATION = 
    //   DriverStation.getAlliance().get() == Alliance.Blue ?  
    //   BLUE_SPEAKER_LOCATION :
    //   RED_SPEAKER_LOCATION;

    public static double NOTE_RADIUS = Units.inchesToMeters(7.0);  

    public class DataPoints{
      public final double distance;
      public final double value;

      public DataPoints(double distance, double value){
        this.distance = distance;
        this.value = value;
      }
    }

    // private static InterpolatingDoubleTreeMap V_TRAP;
    // private static InterpolatingDoubleTreeMap ANGLE_TRAP;

    //     public static InterpolatingDoubleTreeMap getMap(DataPoints[] dataPoints){
    //   InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();

    //   for (DataPoints data : dataPoints) {
    //     map.put(data.distance, data.value);
    //   }

    //   return map;
    // }

    public static InterpolatingDoubleTreeMap V_SPEAKER = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap ANGLE_SPEAKER = new InterpolatingDoubleTreeMap();

    // private static InterpolatingDoubleTreeMap V_AMP;
    // private static InterpolatingDoubleTreeMap ANGLE_AMP;

    // //actual constants values
    // static {
    //   Goal.SPEAKER.ITM_A.put(0.0, 9.0);
    // }

    public enum Goal {
      SPEAKER(
        ANGLE_SPEAKER, V_SPEAKER,
        new Pose2d(BLUE_SPEAKER_LOCATION.getX(), BLUE_SPEAKER_LOCATION.getY(), Rotation2d.fromRadians(Math.PI)), // TODO: change rotation based on color
        10, //TODO: happy zone tuning
        Units.inchesToMeters(3.0 * 12.0 + 5.0 + (3.0 / 8.0)));


  

      public final InterpolatingDoubleTreeMap ITM_A, ITM_V;
      public final Pose2d position;
      public final double happyZone;
      public final double goalWidth;

      private Goal(InterpolatingDoubleTreeMap ITM_A, InterpolatingDoubleTreeMap ITM_V, Pose2d position, double happyZone, double goalWidth){
        this.ITM_A = ITM_A;
        this.ITM_V = ITM_V;
        this.position = position;
        this.happyZone = happyZone;
        this.goalWidth = goalWidth;
      }

    }
  }

  public static final class robot {
    public static final double A_LENGTH = Units.inchesToMeters(24.0); // Axel length (Meters).
    public static final double A_WIDTH = Units.inchesToMeters(24.0); // Axel width (Meters).
    public static final double A_CROSSLENGTH = Math.hypot(A_LENGTH, A_WIDTH);

    public static final double FALCON_ENCODER_TICKS =
        2048.0; // Counts per revolution of the Falcon 500 motor.
    public static final double FALCON_MAX_VEL = 6380.0;

    public static final double MAX_TEMP = 50.0;

    public static final Translation3d SHOULDER_PIVOT_POINT = 
      new Translation3d(A_LENGTH / 2, A_WIDTH / 2, 0.324);

    public static final double SHOULDER_PIVOT_HEIGHT = SHOULDER_PIVOT_POINT.getZ();
  }

  public static final class ids {
    
    public static final int FR_SPEED = 1;
    public static final int FR_ANGLE = 2;
    public static final int FR_ENCODER = 11;

    public static final int FL_SPEED = 3;
    public static final int FL_ANGLE = 4;
    public static final int FL_ENCODER = 12;

    public static final int BL_SPEED = 5;
    public static final int BL_ANGLE = 6;
    public static final int BL_ENCODER = 13;

    public static final int BR_SPEED = 7;
    public static final int BR_ANGLE = 8;
    public static final int BR_ENCODER = 14;

    public static final int PIGEON = 15;

    public static final int INTAKE = 16;

    public static final int SHOULDER_LEADER = 17;
    public static final int SHOULDER_FOLLOWER = 18;

    public static final int LEFT_SHOOTER = 19;
    public static final int RIGHT_SHOOTER = 20;

    public static final int LEFT_CLIMBER = 21;
    public static final int RIGHT_CLIMBER = 22;

    public static final int FEEDER = 23;

    public static final int SHOULDER_ENCODER = 24;

    public static final int FEEDER_BEAM_BREAKER = 5; // input = 0
    
    public static final int INTAKE_BEAM_BREAKER_1 = 8;
    public static final int INTAKE_BEAM_BREAKER_2 = 9;

    public static final int CLIMBER_LEFT_LIMIT = 4;
    public static final int CLIMBER_RIGHT_LIMIT = 1;
  }

  public static class drivetrain {
    public static final double DRIVE_GEARING = 4.75; // 5.14 : 1
    // 4.58056640625
    // 4.57177734375
    // 4.640625
    // 4.57470703125
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double DRIVE_TICKS_PER_ROTATION =
        robot.FALCON_ENCODER_TICKS * DRIVE_GEARING;
    public static final double WHEEL_CIRCUMFRENCE = Math.PI * WHEEL_DIAMETER;
    public static final double DRIVE_TICKS_PER_METER =
        DRIVE_GEARING / WHEEL_CIRCUMFRENCE;
    public static final double DRIVE_METERS_PER_TICK = 1 / DRIVE_TICKS_PER_METER;

    public static final double ANGLE_GEARING = 11.3142; // 11.3142 : 1
    public static final double ANGLE_TICKS_PER_ROTATION =
        robot.FALCON_ENCODER_TICKS * ANGLE_GEARING;
    public static final double ANGLE_TICKS_PER_DEGREE = ANGLE_TICKS_PER_ROTATION / 360.0;
    public static final double ANGLE_DEGREES_PER_TICK = 1.0 / ANGLE_TICKS_PER_DEGREE;

    public static final double MAX_VOLTS = 12.0;
    public static final double MAX_VELOCITY = 5.0; // ?
    public static final double MAX_ACCEL = 12.6;
    public static final double MAX_CACCEL = 8.0;
    public static final double MAX_RADIANS = 5 * Math.PI;
    public static final double RAMP_RATE = 0.5;

    public static final Matrix<N3, N1> STATE_STD_DEVS =
        VecBuilder.fill(0.05, 0.05, 0.001); // [x, y, theta]
    public static final Matrix<N3, N1> VISION_STD_DEVS =
        VecBuilder.fill(0.020, 0.020, 0.264); // [x, y, theta]

    public static final Translation2d FL_LOCATION =
        new Translation2d((Constants.robot.A_LENGTH / 2), (Constants.robot.A_WIDTH / 2));
    public static final Translation2d FR_LOCATION =
        new Translation2d((Constants.robot.A_LENGTH / 2), -(Constants.robot.A_WIDTH / 2));
    public static final Translation2d BL_LOCATION =
        new Translation2d(-(Constants.robot.A_LENGTH / 2), (Constants.robot.A_WIDTH / 2));
    public static final Translation2d BR_LOCATION =
        new Translation2d(-(Constants.robot.A_LENGTH / 2), -(Constants.robot.A_WIDTH / 2));

    public static final double FL_ZERO = 148.623046875;
    public static final double BL_ZERO = 115.751953125;
    public static final double BR_ZERO = -155.21484375;
    public static final double FR_ZERO = 85.95703125;

  public static final PIDController ANGLE_PID = new PIDController(0.008 * 12.0, 0.0, 0.0);
    public static final SimpleMotorFeedforward ANGLE_FF = new SimpleMotorFeedforward(0.0, 1);

    public static final PIDController SPEED_PID = new PIDController(0.1, 0.0, 0.0);
    public static final SimpleMotorFeedforward SPEED_FF = new SimpleMotorFeedforward(0, 0);

    // public static final PIDConstants XY_PID = new PIDConstants(3.0, 0.0, 0.0);

    public static final PIDController ROT_PID = new PIDController(0.15, 0.0, 0.006);

    // public static final PIDConstants CORRECTION_PID = new PIDConstants(-0.1, 0.0, -0.006);

    public static final PIDController CORRECTION_PID = new PIDController(0.1, 0.0, 0.006);
  }

  public static final class shoulder {

    // GEAR RATIO: 62.6:1 motor:mechanism
    public static final double GEAR_RATIO = 62.6;
    /**
     * CANCoder lies on a 40T gear, the shoulder pivots on a 48T gear
     */
    public static final double CANCODER_GEAR_RATIO = 1.2;

    public static final double MAX_VEL = 960.0;
    public static final double MAX_ACCEL = 720.0;
    public static final Constraints CONSTRAINTS = new Constraints(MAX_VEL, MAX_ACCEL);

    public static final ProfiledPIDController SHOULDER_PID = new ProfiledPIDController(0.325, 0.0, 0.02, CONSTRAINTS);
    
    public static final ArmFeedforward ARM_FEEDFORWARD = new ArmFeedforward(0.155, 0.3 * 34.5/33.0  , 0); // kS = verge of motion, kg = fixes setpoint after weak p

    public static final double ZERO = -141.2109375;
    public static final double ALLOWED_ERROR = 0.5;

    public static final double UPPER_LIMIT = 75.0; // 79.1
    public static final double LOWER_LIMIT = -75.0; // -77.5
    // 10 14

    public static final double HANDOFF_ANGLE = -58.4;
  }

  public static final class shooter {
    public static final double SHOOTER_DIAMETER_INCHES = 4.0;
    public static final double SHOOTER_DIAMETER_METERS = (SHOOTER_DIAMETER_INCHES) * 0.0254;
    public static final double SHOOTER_CIRCUMFERENCE = SHOOTER_DIAMETER_METERS * Math.PI;

    public static final SimpleMotorFeedforward LEFT_SHOOTER_FEEDFORWARD = new SimpleMotorFeedforward(0.3877, 0.40499 * 12.0/11.6 * 12.0/12.3, 0.065673); // kS = verge of motion, kV = volts/vel
    public static final PIDController LEFT_SHOOTER_PID = new PIDController(0.0072945, 0.0, 0.0); // 0.28219 * 1.1

    public static final SimpleMotorFeedforward RIGHT_SHOOTER_FEEDFORWARD = new SimpleMotorFeedforward(0.16979, 0.41132 * 12.0/11.5 * 12.0/12.2, 0.10509); // kS = verge of motion, kV = volts/vel
    public static final PIDController RIGHT_SHOOTER_PID = new PIDController(0.019097, 0.0, 0.0); // 0.35718 * 1.1

    public static final double MAX_VELOCITY = 20.0;
    
    public class DataPoints{
      public final double distance;
      public final double value;

      public DataPoints(double distance, double value){
        this.distance = distance;
        this.value = value;
      }
    }

    public static final DataPoints[] VELOCITY_DATA_POINTS = {};
    public static final DataPoints[] ANGLE_DATA_POINTS = {};
  }

  public static final class climber {
    public static final double METERS_PER_ROT_LEFT = 0.14283563 / 40.0; // 40 : 1 | 1 : 0.1428
    public static final double METERS_PER_ROT_RIGHT = 0.14283563 / 40.0;

    public static final double DIST = 0.2032; // 8 in

    public static final double LEFT_kP = 0.0;
    public static final double LEFT_kI = 0.0;
    public static final double LEFT_kD = 0.0;

    public static final double RIGHT_kP = 0.0;
    public static final double RIGHT_kI = 0.0;
    public static final double RIGHT_kD = 0.0;

    public static final double MAX_VELOCITY = 0.1;
    public static final double MAX_ACCEL = 0.1;

    public static final double LEFT_kS = 0.0;
    public static final double LEFT_kG = 0.0;
    public static final double LEFT_kV = 0.0;

    public static final double RIGHT_kS = 0.0;
    public static final double RIGHT_kG = 0.0;
    public static final double RIGHT_kV = 0.0;
  }

  public static final class sensors {
    public static final class pigeon {
      public static final double PIGEON_OFFSET_DEGREES = 0.0;
    }

  public static class limelight {
    public static final Pose3d FRONT_LIMELIGHT_LOCATION = new Pose3d(Units.inchesToMeters(12.0), 0.0, Units.inchesToMeters(9.95), new Rotation3d(0, 0, 0));
    
    public static final Pose3d BACK_LIMELIGHT_LOCATION = new Pose3d(Units.inchesToMeters(-17.0), 0.0, Units.inchesToMeters(6.5), new Rotation3d(0, 0, Math.PI));
  }

      

    // public static final class limelight {
    //   // public static final String FRONT_NAME = "-front";
    //   // public static final Translation3d FRONT_POSITION = new Translation3d(0.4476242, 0, 0.2650236); // translation/rotation in robot space from robot to limelight
    //   // public static final Rotation3d FRONT_ROTATION = 
    //   //   new Rotation3d(
    //   //     Units.degreesToRadians(0),
    //   //     Units.degreesToRadians(0),
    //   //     Units.degreesToRadians(0));

    //   public static final String BACK_NAME = "limelight-back";
    //   public static final Translation3d BACK_POSITION = new Translation3d(-0.2859024, 0, 0.1682242); 
    //   public static final Rotation3d BACK_ROTATION = 
    //     new Rotation3d(
    //       Units.degreesToRadians(180.0),
    //       Units.degreesToRadians(0),
    //       Units.degreesToRadians(180.0));
    // }
  }
}