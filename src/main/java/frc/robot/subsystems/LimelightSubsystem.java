// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.wpilib.units.Units.Micro;

import java.lang.StackWalker.Option;
import java.util.ArrayList;
import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableEntry;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.util.struct.Struct;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.Alliance;
import org.wpilib.smartdashboard.*;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.RobotModeTriggers;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-rock");
  private final StructPublisher<Pose2d> outPosePublisher;
  private final StructPublisher<Pose2d> posePublisher;
  private final Pigeon2 m_gyro;
  private final CommandSwerveDrivetrain m_drivetrain;

  public LimelightSubsystem(CommandSwerveDrivetrain drivetrain, Pigeon2 gyro) {
    posePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Limelight/Pose", Pose2d.struct).publish();
    outPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Limelight/OutPose", Pose2d.struct).publish();
    m_gyro = gyro;
    m_drivetrain = drivetrain;
  }

  public Command sendRobotOrientationCommand() {
    return run(() -> {
      setRobotOrientation();
    }).ignoringDisable(true);

  }

  public Command resetPoseCommand() {
    return run(() -> {try {
        getRobotPose().ifPresent(pose -> {posePublisher.set(pose); m_drivetrain.resetPose(pose);});
      } catch (Exception e) {
        e.printStackTrace();
      }});
  }

  Pose2d lastPose = Pose2d.ZERO;
  public Optional<Pose2d> getRobotPose() {
    var est = (MatchState.getAlliance().equals(Optional.of(Alliance.RED))) ?
      LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight-rock") : 
      LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-rock");
    if (est == null || est.pose == null) {
      //SmartDashboard.putBoolean("Limelight Active", false);
      return Optional.empty(); 
    }
    if (rejectUpdate()) {
      return Optional.empty();
    }
    if (est.pose == Pose2d.ZERO) {
      return Optional.empty();
    }
    if (est.pose.minus(lastPose).getTranslation().getNorm() < Constants.SwerveConstants.kLLUpdateTranslationDeadband && est.pose.minus(lastPose).getRotation().getRadians() < Constants.SwerveConstants.kLLUpdateRotationDeadband) {
      return Optional.empty();
    }
    if (est.pose.getTranslation().getNorm() < Constants.SwerveConstants.kLLUpdateTranslationDeadband) {
      return Optional.empty();
    }
       // SmartDashboard.putBoolean("Limelight Active", true);
    var out = est.pose;
    outPosePublisher.set(out);
    lastPose = out;
    return Optional.of(out);
  }

  public double getLatency() {
    double[] value = table.getEntry("botpose_orb_wpiblue").getDoubleArray((double[]) null);
    return value[6];
  }

  public boolean setFuducialFilter(double[] idFilters) {
    return table.getEntry("fiducial_id_filters_set").setDoubleArray(idFilters);
  }

  public Command poseCommand() {
    return sendRobotOrientationCommand();
  }

  public LimelightHelpers.PoseEstimate getPoseEstimate() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-rock");
  }

  public void setRobotOrientation() {
    LimelightHelpers.SetRobotOrientation("limelight-rock", 
      m_gyro.getYaw().getValueAsDouble(), m_gyro.getAngularVelocityZDevice().getValueAsDouble(), 
      m_gyro.getPitch().getValueAsDouble(), m_gyro.getAngularVelocityYDevice().getValueAsDouble(), 
      m_gyro.getRoll().getValueAsDouble(), m_gyro.getAngularVelocityXDevice().getValueAsDouble());
  }

  public Boolean rejectUpdate() {
    Boolean doRejectUpdate = false;
    if (Math.abs(m_gyro.getAngularVelocityZDevice().getValueAsDouble()) > 360) {
      doRejectUpdate = true;
    }
    if (LimelightHelpers.getTargetCount("limelight-rock") == 0) {
      doRejectUpdate = true;
    }
    return doRejectUpdate;
  }

  public Command resetOdometryFromVisionPoseSamples(CommandSwerveDrivetrain drivetrain) {
    ArrayList<Pose2d> sampledPoses = new ArrayList<Pose2d>();
    return Commands.sequence(Commands.run(() -> {
      getRobotPose().ifPresent(pose -> {sampledPoses.add(pose);});
    }).withTimeout(Constants.AutonomousConstants.kVisionPoseSampleTimeout), Commands.runOnce(() -> {
      Pose2d averagePose = Pose2d.ZERO;
      for (Pose2d pose : sampledPoses) {
        averagePose.plus(new Transform2d(pose.getTranslation(), pose.getRotation()));
      }
      averagePose.div(sampledPoses.size());
      drivetrain.resetPose(averagePose);
    }));
  }
}