// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.RobotModeTriggers;
import org.wpilib.command2.button.Trigger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.controller.PIDController;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.driverstation.XboxController;
import org.wpilib.hardware.bus.CANPort;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.*;
import frc.robot.Constants;

import java.util.function.Supplier;





public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax m_motorRightIntake = new SparkMax(Constants.CANPortConstants.portOne,Constants.IntakeConstants.k_RightIntakeId, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax m_motorRightSpin= new SparkMax(Constants.CANPortConstants.portOne,Constants.IntakeConstants.k_RightSpinId, SparkLowLevel.MotorType.kBrushed);
  private final PIDController m_pid  =  new PIDController(Constants.IntakeConstants.k_Kp, 0, Constants.IntakeConstants.k_Kd);

  private final SparkFlexConfig m_motorConfig = new SparkFlexConfig();

  private final CANcoder m_RightEncoder = new CANcoder(Constants.IntakeConstants.k_EncoderID, TunerConstants.kSwerveCANBus);
  private final ArmFeedforward m_armFeed = new ArmFeedforward(Constants.IntakeConstants.k_Ks, Constants.IntakeConstants.k_Kg, Constants.IntakeConstants.k_Kv);
  public Trigger runIntakeTrigger = new Trigger(() -> {return false;});
  public Trigger reverseIntakeTrigger = new Trigger(() -> {return false;});
  public Trigger resetEncoderTrigger = new Trigger(() -> {return false;});
  public Trigger manualIntakeTrigger = new Trigger(() -> {return false;});

  public Supplier<Double> manualControlAxis = () -> 0d;

  private final DoublePublisher m_encoderPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Intake/Encoder").publish();
  private final DoublePublisher m_anglePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Intake/Angle").publish();


  public IntakeSubsystem() {
  }
  public Command runCommand() {
    return runOnce(
        () -> {
          Trigger autoIntakeTrigger = runIntakeTrigger;
          autoIntakeTrigger.and(RobotModeTriggers.teleop()).whileTrue(runIntakeCommand());
          runIntakeTrigger.negate().and(RobotModeTriggers.teleop()).whileTrue(returnToUpPositionCommand());
          resetEncoderTrigger.onTrue(resetEncoderCommand().onlyIf(() -> RobotModeTriggers.utility().getAsBoolean())); /*changed named from test mode to utility? */
         }).alongWith(returnToUpPositionCommand().onlyWhile(runIntakeTrigger.negate()::getAsBoolean));
  }

  //Creates the output needed for the motor spin to a certain radian
  // the arm feedforward is only dependent on the angle and speed we're trying to get the arm to.
  // Also you should never have numbers in your code with no description of what they are.
  // I can understand what "20" means in this function but putting it in a constant would make the code easier to read
  // The output of the encoder further needs to be converted into radians
  public double calculateMotorOutput(double desiredAngle) {
    double output = (m_pid.calculate((m_RightEncoder.getPosition().getValueAsDouble()*Constants.IntakeConstants.k_EncoderConversionFactor), desiredAngle)
     + m_armFeed.calculate((m_RightEncoder.getPosition().getValueAsDouble()*Constants.IntakeConstants.k_EncoderConversionFactor), /* m_RightEncoder.getVelocity().getValueAsDouble() / 20 */ Constants.IntakeConstants.k_TargetVelocity));
     m_anglePublisher.set((m_RightEncoder.getPosition().getValueAsDouble()*Constants.IntakeConstants.k_EncoderConversionFactor));
     m_encoderPublisher.set(m_RightEncoder.getPosition().getValueAsDouble());
    return -output; // TODO: REMOVE INVERISON IF BROKEN!!!
  

  }

  public Command runIntakeCommand() {
    return Commands.run(() -> {
     int inversionFactor = 1;
      if (reverseIntakeTrigger.getAsBoolean()) {
        inversionFactor = -1;
      }
      m_motorRightIntake.setThrottle(calculateMotorOutput(Constants.IntakeConstants.k_TargetAngle));
      m_motorRightSpin.setThrottle(inversionFactor*Constants.IntakeConstants.k_IntakePower);
    }).finallyDo(() -> m_motorRightSpin.setThrottle(0));
  } 
  
  public Command returnToUpPositionCommand() {
    return Commands.run(() -> {
      m_motorRightIntake.setThrottle(calculateMotorOutput(Constants.IntakeConstants.k_UpAngle));
    });
  }
  
  public Command resetEncoderCommand() {
    return Commands.runOnce(() -> {
      m_RightEncoder.setPosition(0);
    });
  }

  public Command publishAngleCommand() {
    return Commands.run(() -> {
        m_anglePublisher.set((m_RightEncoder.getPosition().getValueAsDouble()*Constants.IntakeConstants.k_EncoderConversionFactor));
        m_encoderPublisher.set(m_RightEncoder.getPosition().getValueAsDouble());
        
    }).alongWith(returnToUpPositionCommand());
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
