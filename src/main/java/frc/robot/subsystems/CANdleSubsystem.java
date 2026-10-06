// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANdle;

import frc.robot.Constants;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;

public class CANdleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final CANdle m_CANdle;

  public CANdleSubsystem() {
    m_CANdle = new CANdle(42, Constants.CANBusConstants.mainBus);
  }

  public synchronized void setState(ControlRequest animation) {
    m_CANdle.setControl(animation);
  }
}