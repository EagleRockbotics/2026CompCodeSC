// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoHub.Warnings;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.controls.*;
import choreo.auto.AutoFactory;
import org.wpilib.math.controller.ElevatorFeedforward;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTableInstance;
//import org.wpilib.hardware.servo.Servo;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.RobotModeTriggers;
import org.wpilib.command2.button.Trigger;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final CANrange distanceSensor = new CANrange(6, TunerConstants.kSwerveCANBus);

  private final SparkMax m_motor = new SparkMax(Constants.CANPortConstants.portTwo,Constants.ElevatorConstants.kElevatorMotorID, SparkLowLevel.MotorType.kBrushless);
  private final SparkBaseConfig m_motorConfig = new SparkMaxConfig().idleMode(IdleMode.kCoast).voltageCompensation(11);
  private final PIDController m_controller = new PIDController(Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI, Constants.ElevatorConstants.kD);

  // private final Servo m_leftServo = new Servo(Constants.ElevatorConstants.kLeftServoChannel);
  // private final Servo m_rightServo = new Servo(Constants.ElevatorConstants.kRightServoChannel);
  // private final Servo m_topServo = new Servo(Constants.ElevatorConstants.kTopServoChannel);

  private final RelativeEncoder m_encoder = m_motor.getEncoder();
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(Constants.ElevatorConstants.kS, Constants.ElevatorConstants.kG, Constants.ElevatorConstants.kV);

  private boolean sideServosReleased;

  public Trigger raiseElevatorTrigger = new Trigger(() -> {return false;});
  public Trigger lowerElevatorTrigger = new Trigger(() -> {return false;});
  public Trigger releaseSideServosTrigger = new Trigger(() -> {return false;});
  public Trigger runTopServoTrigger = new Trigger(() -> {return false;});
  public Trigger enableElevatorTrigger = new Trigger(() -> {return false;});

  public Trigger backLeftButtonTrigger = new Trigger(() -> {return false;});
  public Trigger backRightButtonTrigger = new Trigger(() -> {return false;});
  public Supplier<Double> backLeftButtonAxis = () -> {return 0d;};
  public Supplier<Double> backRightButtonAxis = () -> {return 0d;};

  public boolean useDistanceSensor = false;

  private final DoublePublisher encoderPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Elevator/Encoder").publish();
  private final DoublePublisher PIDOutputPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Elevator/PIDOutput").publish();
  private final DoublePublisher feedforwardOutputPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Elevator/FeedforwardOutput").publish();

  public ElevatorSubsystem() {
    m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_encoder.setPosition(0);
    m_controller.setSetpoint(0);
    sideServosReleased = false;
  }

  public Command raiseElevatorCommand() {
    return Commands.runOnce(() -> {
      System.out.println("test");
      m_controller.setSetpoint(Constants.ElevatorConstants.kUpPosition);
    });
  }
  public Command lowerElevatorCommand() {
    return Commands.runOnce(() -> {
      m_controller.setSetpoint(Constants.ElevatorConstants.kDownPosition);
    });
  }
  public Command setElevatorVoltageCommand() {
    return Commands.run(() -> {
      double PIDOutput = m_controller.calculate(m_encoder.getPosition().get());
      double feedforwardOutput = m_feedforward.calculate(m_encoder.getVelocity().get());
      m_motor.setVoltage(PIDOutput + feedforwardOutput);

      encoderPublisher.set(m_encoder.getPosition().get());
      PIDOutputPublisher.set(PIDOutput);
      feedforwardOutputPublisher.set(feedforwardOutput);
    });
  }

  public Command elevatorCommand() {
    // return Commands.runOnce(() -> { // fix me
    //   enableElevatorTrigger.whileTrue(Commands.run(() -> { // this is dogshit
    //     raiseElevatorTrigger.whileTrue(raiseElevatorCommand());
    //     lowerElevatorTrigger.whileTrue(lowerElevatorCommand());
    //     backLeftButtonTrigger.whileTrue(Commands.run(() -> {
    //       m_profiledController.setConstraints(new TrapezoidProfile.Constraints(
    //         Math.min(Constants.ElevatorConstants.kMaxVelocity*backLeftButtonAxis.get(),Constants.ElevatorConstants.kMaxVelocity), Constants.ElevatorConstants.kMaxAcceleration
    //       ));
    //       lowerElevatorCommand();
    //     }).onlyIf(backRightButtonTrigger.negate()));
    //     backRightButtonTrigger.whileTrue(Commands.run(() -> {
    //       m_profiledController.setConstraints(new TrapezoidProfile.Constraints(
    //         Math.min(Constants.ElevatorConstants.kMaxVelocity*backRightButtonAxis.get(),Constants.ElevatorConstants.kMaxVelocity), Constants.ElevatorConstants.kMaxAcceleration
    //       ));
    //       raiseElevatorCommand();
    //     }).onlyIf(backLeftButtonTrigger.negate()));
    //     releaseSideServosTrigger.onTrue(releaseSideServosCommand());
    //     runTopServoTrigger.onTrue(runTopServoCommand());
    //   }));
    // });
    return Commands.parallel(setElevatorVoltageCommand(), Commands.runOnce(() -> {
          raiseElevatorTrigger.onTrue(raiseElevatorCommand());
          lowerElevatorTrigger.onTrue(lowerElevatorCommand());
    }));
  }

  // public Command teleopAlignWithLadder(CommandSwerveDrivetrain drivetrain) {
  //   Pose2d currentPose = drivetrain.getState().Pose;
  //   Pose2d targetPose = currentPose.getTranslation().getDistance(Constants.FieldConstants.kLeftLadderPose.getTranslation()) <
  //     currentPose.getTranslation().getDistance(Constants.FieldConstants.kRightLadderPose.getTranslation()) ?
  //     Constants.FieldConstants.kLeftLadderPose.plus(new Transform2d(new Translation2d(Constants.ElevatorConstants.kElevatorPositionFrontOffset, 0), new Rotation2d(Math.PI))) : 
  //     Constants.FieldConstants.kRightLadderPose.plus(new Transform2d(new Translation2d(Constants.ElevatorConstants.kElevatorPositionFrontOffset, 0), new Rotation2d(Math.PI)));
    
  //   return drivetrain.moveToPose(targetPose);
  // }

  public Command moveToLadder(CommandSwerveDrivetrain drivetrain) {
    Pose2d currentPose = drivetrain.getState().Pose;
    Pose2d targetPose = currentPose.getTranslation().getDistance(Constants.FieldConstants.kLeftLadderPose.getTranslation()) <
      currentPose.getTranslation().getDistance(Constants.FieldConstants.kRightLadderPose.getTranslation()) ?
      Constants.FieldConstants.kLeftLadderPose : Constants.FieldConstants.kRightLadderPose;
    
    if (useDistanceSensor) {
      return drivetrain.moveToDistanceSensorPoint(distanceSensor, Constants.FieldConstants.kLeftLadderPose.getX() + Constants.SwerveConstants.kRobotLength/2, targetPose);
    } else {
      return drivetrain.moveToPose(targetPose);
    }
  }

  @Override
  public void periodic() {
    encoderPublisher.set(m_encoder.getPosition().get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
