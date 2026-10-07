// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

import static edu.wpi.first.units.Units.Rotation;

import java.lang.StackWalker.Option;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkFlex m_driveMotor = new SparkFlex(Constants.ShooterConstants.kDriveMotorId, MotorType.kBrushless);
  private final SparkFlexConfig m_motorConfig = new SparkFlexConfig();
  private final CommandSwerveDrivetrain m_drivetrain;
  private final LimelightSubsystem m_limelightSubsystem;
  private final CANdleSubsystem m_CANdle;

  // SHOOTER MODE CONFIGURATION
  private final boolean forceLimelight = false;
  private final boolean useRobotVelocityCompensation = true; // TODO: set during testing & stuff
  private final boolean useShooterOffsetCompensation = true; // if true, shooter is aimed at hub; if false, robot is aimed at hub
  private final boolean useZippyZoomMath = false; // takes priority over useRobotVelocityCompensation (they're mutually exclusive)

  private final SparkMax m_indexerBeltMotor = new SparkMax(Constants.ShooterConstants.kIndexerBeltMotorId, MotorType.kBrushed);
  private final SparkMax m_indexerRollerMotor = new SparkMax(Constants.ShooterConstants.kIndexerRollerMotorId, MotorType.kBrushed);

  public Trigger autoAimTeleopTrigger = new Trigger(() -> {return false;});
  public Trigger manualAimTeleopTrigger = new Trigger(() -> {return false;});
  public Supplier<Double> xAxis = () -> {return 0d;};
  public Supplier<Double> yAxis = () -> {return 0d;};

  private final StructPublisher<Pose2d> m_targetAnglePublisher = NetworkTableInstance.getDefault().getStructTopic("Shooter/FacingTarget", Pose2d.struct).publish();
  private final DoublePublisher rpmPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Shooter/RPM").publish();
  private final DoublePublisher targetVelocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Shooter/Target Velocity").publish();
  private final BooleanPublisher alliancePublisher = NetworkTableInstance.getDefault().getBooleanTopic("Is Alliance Red").publish();

  private Alliance currentAlliance = ((DriverStation.getAlliance().equals(Optional.of(Alliance.Red)))) ? Alliance.Red : Alliance.Blue;
  private Translation2d absoluteHubPosition = Constants.FieldConstants.kBlueHubPosition;

  @SuppressWarnings("removal")
  public ShooterSubsystem(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight, CANdleSubsystem CANdle) {
    m_motorConfig.closedLoop.p(Constants.ShooterConstants.kP)
      .i(Constants.ShooterConstants.kI)
      .d(Constants.ShooterConstants.kD)
      .velocityFF(Constants.ShooterConstants.kF)
      .iZone(0.5);

    m_motorConfig.voltageCompensation(11);
    m_motorConfig.idleMode(IdleMode.kCoast);
    m_driveMotor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_drivetrain = drivetrain;
    m_limelightSubsystem = limelight;
    m_CANdle = CANdle;
    
    SmartDashboard.putNumber("Shooter/Test Shooter RPM", 0);
  }

  public double getDistanceFromPose(Pose2d currentPose) {
    Translation2d currentPosition = new Translation2d(currentPose.getX(), currentPose.getY());
    return currentPosition.getDistance(Constants.FieldConstants.kBlueHubPosition); 
  }

  public Optional<Pose2d> getCurrentPose() {
    Optional<Pose2d> pose;
    if (forceLimelight) {
      pose = m_limelightSubsystem.getRobotPose();
      if(DriverStation.isAutonomous()) {
        pose = Optional.of(m_drivetrain.getState().Pose);
      }
    } else {
      pose = Optional.of(m_drivetrain.getState().Pose);
    }
    alliancePublisher.set(DriverStation.getAlliance() == Optional.of(Alliance.Red));
    return pose;
  }

  public Optional<Translation2d> getCurrentPosition() {
    var currentPose = getCurrentPose();
    return currentPose.map(pose -> new Translation2d(pose.getX(), pose.getY())).or(() -> Optional.of(m_drivetrain.getPose().getTranslation()));
  }

  public Optional<Translation2d> getEffectiveHubPosition() { // gets robot velocity-compensated hub position
   return getCurrentPosition().map(position -> {
  Translation2d actualHubPosition = absoluteHubPosition.minus(position);
    double flightTime = calculateFlightTime(actualHubPosition.getNorm());
    ChassisSpeeds currentChassisSpeeds = m_drivetrain.getState().Speeds;

    Translation2d effectiveHubPosition = new Translation2d(
      actualHubPosition.getX() - Constants.ShooterConstants.kVelocityCompensationFactor*flightTime*currentChassisSpeeds.vxMetersPerSecond,
      actualHubPosition.getY() - Constants.ShooterConstants.kVelocityCompensationFactor*flightTime*currentChassisSpeeds.vyMetersPerSecond
    );
    return effectiveHubPosition;
   });
  }

  private Translation2d flipX(Translation2d input) {
    return new Translation2d(16.5-input.getX(), input.getY());
  }

  @SuppressWarnings("unused")
  public Optional<Translation2d> getCurrentHubPosition() {
    if (useRobotVelocityCompensation && !useZippyZoomMath) {
      return getEffectiveHubPosition();
    } else {
      return getCurrentPose().flatMap(pose -> Optional.of(absoluteHubPosition.minus(pose.getTranslation()))); 
    }
  }

  public double calculateFlightTime(double distance) {
    return distance/calculateTargetVelocity(distance);
  }

  public double calculateTargetVelocity(double distance) {
    double g = -9.81;
    double c = Constants.FieldConstants.kHubHeight-Constants.ShooterConstants.kShooterHeight;
    double theta = Constants.ShooterConstants.kShooterAngle;
    SmartDashboard.putNumber("Hub Distance", distance);
    // Will return NaN if too close to Hub. Motor is n
    return Math.sqrt(((g*g)*(distance*distance))/(-2*Math.pow(Math.cos(theta),2)*(g*distance*Math.tan(theta)-(g*c))));
  }

  public double calculateRPMFromVelocity(double velocity) { // TODO: do this
    double slope = 605;
    double intercept = 375;
    return (slope*velocity)-intercept;
  }

  public Optional<Double> calculateTargetAngle() { // calculates target angle while accounting for useShooterOffsetCompensation
    var hubPosition = getCurrentHubPosition();

    return hubPosition
            .map(position -> Math.atan2(position.getY(), position.getX()) + (useShooterOffsetCompensation ? Math.acos(Constants.ShooterConstants.kShooterDistanceFromCenter/position.getNorm()) - Math.PI/2 : 0));
  }

  public Command driveAtInputRPM() {
    return Commands.sequence(driveShooterCommand(() -> Optional.of(SmartDashboard.getNumber("Shooter/Test Shooter RPM", 0)))).finallyDo(() -> m_driveMotor.set(0));
  }

  public Command pointingTest() {
    return Commands.run(() -> {
      
    });
  }

  //Returns robot angle offset and then shooter exit velocity (!!! not rpm). takes in exit velocity in m/s
  public Pair<Double, Double> zippyZoomMath(double desiredExitVelocity, Translation2d targetPoint) {
    Translation2d robotVelocity = new Translation2d(m_drivetrain.getState().Speeds.vxMetersPerSecond, m_drivetrain.getState().Speeds.vyMetersPerSecond);
    Translation2d robotToTarget = targetPoint.minus(m_drivetrain.getPose().getTranslation());
    Translation2d unitInTargetDirection = robotToTarget.div(robotToTarget.getNorm());
    Translation2d unitInOrthDirection = unitInTargetDirection.rotateBy(new Rotation2d(Math.PI/2));

    double robotVelocityTowardsPoint = robotVelocity.dot(unitInTargetDirection);
    double robotVelocityOrthToPoint = robotVelocity.dot(unitInOrthDirection)*Constants.SwerveConstants.kAirResistanceFactor;
    double newExitVelocity = Math.sqrt(Math.pow(desiredExitVelocity - robotVelocityTowardsPoint, 2) + Math.pow(robotVelocityOrthToPoint, 2));
    
    double angleOffset = -Math.asin(robotVelocityOrthToPoint/newExitVelocity);

    return new Pair<Double,Double>(angleOffset, newExitVelocity);
  }

  public Pair<Command, Supplier<Optional<SwerveRequest>>> shooterCommand() {
    return new Pair<Command,Supplier<Optional<SwerveRequest>>>(Commands.runOnce(() -> {
      autoAimTeleopTrigger.and(() -> !getRobotTooCloseToHub()).and(manualAimTeleopTrigger.negate()).whileTrue(Commands.parallel(
        driveShooterCommand(() -> getCurrentHubPosition().map(hubPosition -> getOutputRPM(hubPosition))), 
        Commands.run(() -> getHubDistance().ifPresent(distance -> targetVelocityPublisher.set(calculateTargetVelocity(distance)))),
        Commands.runOnce(() -> m_CANdle.setState(new StrobeAnimation(0, 7)
                                          .withColor(RGBWColor.fromHSV(52, 46.3, 100))
                                          .withFrameRate(100)))
        ));
      manualAimTeleopTrigger.and(autoAimTeleopTrigger.negate()).whileTrue(driveShooterCommand(() -> Optional.of(Constants.ShooterConstants.kPassRPM)));
      manualAimTeleopTrigger.and(autoAimTeleopTrigger).whileTrue(driveShooterCommand(() -> Optional.of(Constants.ShooterConstants.kStaticShootRPM)));
      autoAimTeleopTrigger.or(manualAimTeleopTrigger).negate().whileTrue(shooterIdle());
  }).alongWith(runOnce(() -> 
                    m_CANdle.setState(new SolidColor(0, 7).withColor(RGBWColor.fromHSV(37, 79.2, 100))))), 
  this::getAimRequest);
  }

  private double getOutputRPM(Translation2d hub) {
      Translation2d hubPosition = hub;
      double hubDistance = hubPosition.getNorm();
      double targetVelocity;
      if (useZippyZoomMath) {
        targetVelocity = zippyZoomMath(calculateTargetVelocity(hubDistance), hubPosition).getSecond();
      } else{
        targetVelocity = calculateTargetVelocity(hubDistance);
      }
      double RPMSetpoint = calculateRPMFromVelocity(targetVelocity);
      return RPMSetpoint;
  }

  private boolean getRobotTooCloseToHub() {
      Translation2d hubPosition = getCurrentHubPosition().orElse(Translation2d.kZero);
      double hubDistance = hubPosition.getNorm();
      SmartDashboard.putBoolean("Can Shoot", !(hubDistance < Constants.ShooterConstants.kMinRobotDistanceFromHub));
      return hubDistance < Constants.ShooterConstants.kMinRobotDistanceFromHub;
  }

  private Optional<Double> getHubDistance() {
    return getCurrentHubPosition().map(hubPosition -> hubPosition.getNorm());
  }

  public Pair<Command, Supplier<Optional<SwerveRequest>>> autoShooterCommand() {
    return new Pair<Command, Supplier<Optional<SwerveRequest>>>(
      Commands.parallel(driveShooterCommand(() -> getCurrentHubPosition().map(hubPosition -> getOutputRPM(hubPosition))), Commands.run(() -> getHubDistance().ifPresent(distance -> targetVelocityPublisher.set(calculateTargetVelocity(distance))))),
      this::getPointRequest
    );
  }

  private Command driveShooterCommand(Supplier<Optional<Double>> RPM) {
    return Commands.run(() -> RPM.get().ifPresent(rpm ->
      {
    m_driveMotor.getClosedLoopController().setSetpoint(-rpm, ControlType.kVelocity);
    rpmPublisher.set(this.m_driveMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter/Target RPM", -rpm);
    SignalLogger.writeDouble("Shooter RPM", -rpm);
    if (Math.abs(-this.m_driveMotor.getEncoder().getVelocity() - (rpm)) < Constants.ShooterConstants.kMaxRPMOffsetBeforeShootFails) {
      m_indexerBeltMotor.set(Constants.ShooterConstants.kIndexerBeltPower);
      m_indexerRollerMotor.set(Constants.ShooterConstants.kIndexerRollerPower);
    } else {
      m_indexerBeltMotor.set(0);
      m_indexerRollerMotor.set(0);
    }
   })).finallyDo(() -> {m_indexerBeltMotor.set(0); m_indexerRollerMotor.set(0);}); 
  }


  double lastRPM = 0;
  private Command shooterIdle() {
    Timer offTime = new Timer();
    return Commands.sequence( 
    Commands.run(() -> {
      m_driveMotor.set(0); 
      m_CANdle.setState(new SolidColor(0, 7).withColor(RGBWColor.fromHSV(37, 79.2, 100)));
      rpmPublisher.set(m_driveMotor.getEncoder().getVelocity());}));
  }

  public Optional<SwerveRequest> getAimRequest() {
    Optional<Translation2d> hubPosition = getCurrentHubPosition();
    Pose2d currentPose = m_drivetrain.getPose();
    Optional<Double> targetAngle;

    if (useZippyZoomMath) {
      targetAngle = hubPosition.map(position -> zippyZoomMath(calculateTargetVelocity(position.getNorm()), position).getFirst());
    } else {
      targetAngle = calculateTargetAngle();
    }
    targetAngle.ifPresent(angle -> m_targetAnglePublisher.set(new Pose2d(m_drivetrain.getPose().getTranslation(), Rotation2d.fromRadians(angle))));
    if (targetAngle.isPresent()) {
    SignalLogger.writeStruct("Target Pose", Pose2d.struct, new Pose2d(currentPose.getTranslation(), Rotation2d.fromRadians(targetAngle.get())));
  }
  
  return targetAngle.map(angle ->new SwerveRequest.FieldCentricFacingAngle()
      .withTargetDirection(Rotation2d.fromRadians(angle))
      .withHeadingPID(Constants.SwerveConstants.kTurnP, Constants.SwerveConstants.kTurnI, Constants.SwerveConstants.kTurnD)
      .withVelocityX(xAxis.get()*Constants.ShooterConstants.kMaxScoringRobotSpeed)
      .withVelocityY(yAxis.get()*Constants.ShooterConstants.kMaxScoringRobotSpeed));
}

  public Optional<SwerveRequest> getPointRequest() {
    Optional<Translation2d> hubPosition = getCurrentHubPosition();
    Optional<Pose2d> currentPose = getCurrentPose();
    Optional<Double> targetAngle;

    if (useZippyZoomMath) {
      targetAngle = hubPosition.map(position -> zippyZoomMath(calculateTargetVelocity(position.getNorm()), position).getFirst());
    } else {
      targetAngle = calculateTargetAngle();
    }
    if (currentPose.isPresent() && targetAngle.isPresent()) {
    m_targetAnglePublisher.set(new Pose2d(currentPose.get().getTranslation(), Rotation2d.fromRadians(targetAngle.get())));
    SignalLogger.writeStruct("Target Pose", Pose2d.struct, new Pose2d(currentPose.get().getTranslation(), Rotation2d.fromRadians(targetAngle.get())));
  }
  return targetAngle.map(angle ->new SwerveRequest.FieldCentricFacingAngle()
      .withTargetDirection(Rotation2d.fromRadians(angle))
      .withHeadingPID(Constants.SwerveConstants.kTurnP, Constants.SwerveConstants.kTurnI, Constants.SwerveConstants.kTurnD)
      .withVelocityX(0)
      .withVelocityY(0));
  }

  
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
        
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
