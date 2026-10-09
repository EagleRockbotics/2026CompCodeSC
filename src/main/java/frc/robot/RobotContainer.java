// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.autos.ChoreoTraj;
import frc.robot.autos.ChoreoTraj;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AutoHandlingSubsystem;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Radian;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Rotation;
import static org.wpilib.units.Units.RotationsPerSecond;

import java.io.File;
import java.lang.constant.DirectMethodHandleDesc;
import java.lang.invoke.ConstantCallSite;
import java.lang.reflect.Method;
import java.lang.reflect.Type;
import java.util.Dictionary;
import java.util.Hashtable;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import org.wpilib.util.Pair;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.measure.Angle;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.system.Filesystem;
import org.wpilib.driverstation.Joystick;
import org.wpilib.driverstation.XboxController;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.CommandXboxController;
import org.wpilib.command2.button.RobotModeTriggers;
import org.wpilib.command2.button.Trigger;
import org.wpilib.command2.sysid.SysIdRoutine.Direction;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including`
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final CommandGamepad driveStick = new CommandGamepad(
      Constants.OperatorConstants.kDriverControllerPort);
  private final CommandGamepad helperStick = new CommandGamepad(
      Constants.OperatorConstants.kHelperControllerPort);
  private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
  private final AutoHandlingSubsystem m_autoHandler = new AutoHandlingSubsystem(m_drivetrain);
  private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem(m_drivetrain, m_drivetrain.getPigeon2());
  private final CANdleSubsystem m_CANdleSubsystem = new CANdleSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem(m_drivetrain, m_limelightSubsystem, m_CANdleSubsystem);
  private final Pair<Command, Supplier<Optional<SwerveRequest>>> m_shooterPair = m_shooterSubsystem.shooterCommand();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  // private final IntakeSubsystem m_intakeSubsytem = new IntakeSubsystem();

  // Code copied from CTRE Swerve template
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity 

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandGamepad joystick = driveStick; 
  
  private final SlewRateLimiter driveFilter = new SlewRateLimiter(Constants.SwerveConstants.kDriveSlewRateLimit);
  private final SlewRateLimiter turnFilter = new SlewRateLimiter(Constants.SwerveConstants.kTurnSlewRateLimit);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
   //@SuppressWarnings("WPILib.NoDiscard")
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // TODO: Whenever you make a new subsytem, put it in this function.
    // m_autoHandler.setupAutoReflection(this, m_drivetrain, m_autoHandler);
    // m_autoHandler.publishChooser();
    //@SuppressWarnings("WPILib.NoDiscard")@SuppressWarnings("WPILib.NoDiscard");
    m_drivetrain.resetGyro();
    m_drivetrain.resetPose(Pose2d.ZERO);
    }

  public Command updateLimelightCommand() {
    // return m_limelightSubsystem.sendRobotOrientationCommand();
    return Commands.none();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * org.wpilib.command2.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link org.wpilib.command2.button.CommandPS4Controller
   * PS4} controllers or
   * {@link org.wpilib.command2.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        m_drivetrain.applyRequest(() -> idle).ignoringDisable(true));


    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // joystick.back().and(joystick.y()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
    // joystick.back().and(joystick.x()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
    // joystick.start().and(joystick.y()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
    // joystick.start().and(joystick.x()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

    // helperStick.x().and(RobotModeTriggers.test()).whileTrue(m_shooterSubsystem.driveAtInputRPM());

    // Reset the field-centric heading on left bumper press.
    joystick.button(0).onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));

    m_drivetrain.registerTelemetry(logger::telemeterize);

    m_shooterSubsystem.autoAimTeleopTrigger = joystick.rightTrigger().and(RobotModeTriggers.teleop());
    m_shooterSubsystem.manualAimTeleopTrigger = joystick.leftTrigger();
    m_shooterSubsystem.xAxis = () -> -joystick.getLeftY();
    m_shooterSubsystem.yAxis = () -> -joystick.getLeftX();
    joystick.button(3).and(RobotModeTriggers.teleop()).onTrue(Commands.runOnce(() -> {
      m_drivetrain.resetPose(new Pose2d(0, 0, Rotation2d.ZERO));
      m_drivetrain.resetGyro();
    }));

    // joystick.rightTrigger().and(RobotModeTrigq13gers.test()).whileTrue(Commands.sequence(
    //   Commands.runOnce(() -> {
    //     // m_drivetrain.resetPose(m_shooterSubsystem.getCurrentPose().get());
    //   }), 
    //   m_drivetrain.applyOptionalRequest(m_shooterSubsystem::getPointRequest)));
    // joystick.x().and(RobotModeTriggers.test()).onTrue(Commands.runOnce(() -> {
    //   m_limelightSubsystem.getRobotPose().ifPresent(pose -> {m_drivetrain.resetPose(pose);});
    // }));
    //  joystick.y().and(RobotModeTriggers.test()).onTrue(resetGyro());
    //  joystick.x().and(RobotModeTriggers.test()).onTrue(Commands.runOnce(() -> {m_drivetrain.resetPose(new Pose2d());}));

    // m_elevatorSubsystem.backLeftButtonAxis = () -> {return helperStick.getLeftTriggerAxis();};
    // m_elevatorSubsystem.backRightButtonAxis = () -> {return helperStick.getRightTriggerAxis();};
    // m_elevatorSubsystem.backLeftButtonTrigger = helperStick.leftTrigger();
    // m_elevatorSubsystem.backRightButtonTrigger = helperStick.rightTrigger();
    m_elevatorSubsystem.lowerElevatorTrigger = helperStick.getHID().povDown();
    m_elevatorSubsystem.raiseElevatorTrigger = helperStick.getHID().povUp();

    // m_intakeSubsytem.runIntakeTrigger = driveStick.rightBumper();
    // m_intakeSubsytem.reverseIntakeTrigger = driveStick.leftBumper();
    // m_intakeSubsytem.resetEncoderTrigger = helperStick.y();
    // m_intakeSubsytem.manualIntakeTrigger = helperStick.rightBumper();
    // m_intakeSubsytem.manualControlAxis = helperStick::getLeftY;

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getTeleopCommand() {
    return Commands.parallel(
        m_drivetrain.applyRequest(this::getDriveRequest),
        m_shooterPair.getFirst(), 
        // m_intakeSubsytem.runCommand(),
        m_elevatorSubsystem.elevatorCommand()
        // m_limelightSubsystem.resetPoseCommand()
    );
  }

  private SwerveRequest getDriveRequest() {
    if (m_shooterSubsystem.autoAimTeleopTrigger.getAsBoolean() || (m_shooterSubsystem.autoAimTeleopTrigger.getAsBoolean() && m_shooterSubsystem.manualAimTeleopTrigger.getAsBoolean())) {
      return m_shooterPair.getSecond().get().orElse(drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
      .withVelocityY(-joystick.getLeftX() * MaxSpeed).withRotationalRate(0));
    }
    return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
      .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
      .withRotationalRate(-joystick.getRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
  }

  

  public Command getTestCommand() {
    return Commands.none();
    // return m_elevatorSubsystem.elevatorCommand();
  }

  public Command getAutoCommand() {
    // m_drivetrain.resetThetaController();
    // return Commands.sequence(Commands.runOnce(() -> m_drivetrain.resetThetaController()), Commands.runOnce(() -> m_drivetrain.resetPose(new Pose2d(0, 0, Rotation2d.kZero))), m_autoHandler.getAutonomousCommand());
    return Commands.none();
  }

  public Command resetGyro() {
    return Commands.runOnce(
      () -> {m_drivetrain.getPigeon2().setYaw(0.0, 10);}
    );
  }

  public Command setRobotLimelightOrientationCommand() {
    return Commands.runOnce(
      () -> {
        // m_limelightSubsystem.setRobotOrientation();
      }
    );
  }

  public Command addVisionMeasurementCommand() {
    return Commands.runOnce(
      () -> {
        // m_drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(
        //   Constants.PoseEstimationConstants.kVisionXStdDev, // scale later
        //   Constants.PoseEstimationConstants.kVisionYStdDev, // scale later
        //   Constants.PoseEstimationConstants.kVisionHeadingStdDev
        // ));
        // LimelightHelpers.PoseEstimate poseEstimate = m_limelightSubsystem.getPoseEstimate();
        // if (!m_limelightSubsystem.rejectUpdate()) {
        //   m_drivetrain.addVisionMeasurement(
        //      poseEstimate.pose,
        //      poseEstimate.timestampSeconds
        //   );
        // }
      }
    );
  }
/* 
  public AutoRoutine driveTest(AutoFactory factory) {
    AutoRoutine routine = factory.newRoutine("driveTest");
    AutoTrajectory driveFwdOneMeter = ChoreoTraj.NewPath.asAutoTraj(routine);
    routine.active().onTrue(Commands.parallel(
        Commands.sequence(driveFwdOneMeter.resetOdometry(), driveFwdOneMeter.cmd())));
    driveFwdOneMeter.done().onTrue(m_drivetrain.goToEndPose(driveFwdOneMeter));
    return routine;
  }

  public AutoRoutine turnTest(AutoFactory factory) {
    AutoRoutine routine = factory.newRoutine("turnTest");
    AutoTrajectory spin = ChoreoTraj.Spinny.asAutoTraj(routine);
    routine.active().onTrue(Commands.sequence(spin.resetOdometry(), spin.cmd()));
    spin.done().onTrue(m_drivetrain.goToEndPose(spin));
    return routine;
  }

  public AutoRoutine movingTurnTest(AutoFactory factory) {
    AutoRoutine routine = factory.newRoutine("movingTurnTest");
    AutoTrajectory traj = ChoreoTraj.MovingSpinny.asAutoTraj(routine);
    routine.active().onTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));
    traj.done().onTrue(m_drivetrain.goToEndPose(traj));
    return routine;
  }

  public AutoRoutine autoElevatorRoutine(AutoFactory factory) {
    AutoRoutine routine = factory.newRoutine("autoElevatorRoutine");
    AutoTrajectory traj = ChoreoTraj.AutonomousMoveToElevatorPosition1.asAutoTraj(routine);
    routine.active().onTrue(Commands.sequence(
      // traj.resetOdometry(), traj.cmd(),
      // m_elevatorSubsystem.runTopServoCommand().withTimeout(Constants.AutonomousConstants.kAutoElevatorTopServoTimeout),
      // m_elevatorSubsystem.raiseElevatorCommand(),
      // m_elevatorSubsystem.moveToLadder(m_drivetrain),
      // m_elevatorSubsystem.lowerElevatorCommand()
    ));
    return routine;
  }

  public AutoRoutine testMainRoutine(AutoFactory factory) {
    // Robot should start at ~(2.0 m, 2.0 m); resets pose from vision samples, aligns to desired starting pose, shoots, and then runs the auto elevator
    AutoRoutine routine = factory.newRoutine("testMainRoutine");
    routine.active().onTrue(Commands.sequence(
      // m_limelightSubsystem.resetOdometryFromVisionPoseSamples(m_drivetrain),
      // m_drivetrain.moveToPose(Constants.AutonomousConstants.kTestAutoAlignPose),
      // m_shooterSubsystem.autoShooterCommand().getFirst()
      // , autoElevatorRoutine(factory).cmd()
    ));
    return routine;
  }

  public AutoRoutine shooterAuto(AutoFactory factory) {
    AutoRoutine routine = factory.newRoutine("shooterAuto");
    AutoTrajectory driveToShootPose = ChoreoTraj.DriveToShootPose.asAutoTraj(routine);
    routine.active().onTrue(Commands.sequence(
      Commands.runOnce(m_drivetrain.getPigeon2()::reset),
      driveToShootPose.resetOdometry(),
      driveToShootPose.cmd(),
      Commands.runOnce(() -> {
        new Pose2d(new Translation2d(2, 4), new Rotation2d(0));
      }),
      // m_limelightSubsystem.resetOdometryFromVisionPoseSamples(m_drivetrain),
      Commands.deadline(m_shooterSubsystem.autoShooterCommand().getFirst().withTimeout(5), m_drivetrain.applyOptionalRequest(m_shooterSubsystem.autoShooterCommand().getSecond())
      .onlyIf(() -> {
            Optional<Pose2d> currentPose = m_shooterSubsystem.getCurrentPose();
            if (currentPose.isEmpty()) {return false;} else {
              return currentPose.get().getTranslation().getNorm() > Constants.SwerveConstants.kLLUpdateTranslationDeadband;
            }
          })
    )));
    return routine;
  }
*/
  public void publishAutoChooser() {
    this.m_autoHandler.publishChooser();
  }

  public void resetAutoRoutines() {
    this.m_autoHandler.resetRoutines();
  }

}
