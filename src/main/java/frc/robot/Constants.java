// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. wow who would have thought? This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final String kLimelightName = "limelight-rock";
  public static final int kPigeonID = 14;
  public static final int kDistanceSensorID = 0; // TODO: set CANrange ID

  public static class FieldConstants {
    public static final Translation2d kBlueHubPosition = new Translation2d(4.559, 4.02); // TODO: Configure
    public static final Translation2d kRedHubPosition = new Translation2d(11.593, 4.02);
    public static final Pose2d kLeftLadderPose = new Pose2d(0,0,new Rotation2d(0)); // Pose where robot is in position, not the actual field position!!!
    public static final Pose2d kRightLadderPose = new Pose2d(0,1,new Rotation2d(0)); // Pose where robot is in position, not the actual field position!!!
    public static final double kHubHeight = 2; // Todo: height of hub opening
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kHelperControllerPort = 1;
  }

  public static class SwerveConstants {
    public static final double kDeadband = 0.1;
    public static final double kAngularAccelerationFactor = 4;
    public static final String kCurrentRobot = "comp2025";
    public static final double kAirResistanceFactor = 0;
    public static final double kRobotLength = 12;

    public static final double kTurnP = 5;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0.2;

    public static final double kLLUpdateTranslationDeadband = 0.1;
    public static final double kLLUpdateRotationDeadband = 0.05;

    public static final double kTurnSlewRateLimit = 0.5;
    public static final double kDriveSlewRateLimit = 0.5;
  }

  public static class ChoreoConstants {
    public static final PIDController xController = new PIDController(10, 0, 0);
    public static final PIDController yController = new PIDController(10, 0, 0);
    public static final double kP_theta = 15;
    public static final double kI_theta = 0;
    public static final double kD_theta = 0.01;
  }

  public static class PoseEstimationConstants {
    public static final double kOdometryXStdDev = 0.1;
    public static final double kOdometryYStdDev = 0.1;
    public static final double kOdometryHeadingStdDev = 0.001; // set to a small number because we have gyro for a reason
    public static final double kVisionXStdDev = 0.9;
    public static final double kVisionYStdDev = 0.9;
    public static final double kVisionHeadingStdDev = 67000; // set to a large number because we have gyro for a reason
  }

  public static class AutonomousConstants {
    public static final boolean kEnableAllianceFlipping = false;
    public static final double kVisionPoseSampleTimeout = 0.25; // frick you lazare for making me make this a constant
    public static final double kAutoElevatorTopServoTimeout = 0.5;
    public static final double kStartEndDistanceError = 0.01;
    public static final double kStartEndRotationError = 0.01;
    public static final Pose2d kTestAutoAlignPose = new Pose2d(2, 2, new Rotation2d(0)); // TODO: update if needed
  }

  public static class ShooterConstants {
    public static final double kShooterHeight = 0.43; // TODO: Measure
    public static final double kShooterAngle = Math.toRadians(67);
    public static final int kDriveMotorId = 18;
    public static double kShooterDistanceFromCenter = 0.24;
    public static double kMinRobotDistanceFromHub = 0.1;
    public static final double kP = 0.0015;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kF = 0.000163;
    
    public static final int kIndexerBeltMotorId = 17;
    public static final int kIndexerRollerMotorId = 20;

    public static final double kIndexerBeltPower = 1;
    public static final double kIndexerRollerPower = -1;

    public static final double kMaxRPMOffsetBeforeShootFails = 50; //guess. please change
    public static final double kPassRPM = 5900;
    public static final double kStaticShootRPM = 5000;

    public static final double kMaxScoringRobotSpeed = 1; //guess. in meters per second

    public static final double kIdleDropoff = 50; //RPM rate at which shooter speed will fall off after shooting has stopped
    public static final double kVelocityCompensationFactor = 1.2;
  }

  public static class IntakeConstants {
    public static final double kdt = 0.02;
    public static final int k_RightIntakeId = 1;
    public static final int k_RightSpinId = 2;
    public static final double k_IntakePower = 1; 
    public static final double k_Kp = 0.175;
    public static final double k_Kd = 0;
    public static final double k_Ks = 0;
    public static final double k_Kg = 0.021;
    public static final double k_Kv = 0;

    public static final double k_TargetAngle = 0;
    public static final double k_UpAngle = 0.274414*2*Math.PI; //in radians

    public static final double k_EncoderConversionFactor = 2*Math.PI;
    public static final double k_TargetVelocity = 0;

    public static final int k_EncoderID = 15;
    }

  public static class ElevatorConstants {
    public static final int kElevatorMotorID = 19;
    public static final double kDownPosition = 0;
    public static final double kUpPosition = 15;
    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double kV = 0.0;
    public static final double kP = 1.5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }
}
