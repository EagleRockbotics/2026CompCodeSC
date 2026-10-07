// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANdle;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANdleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final CANdle m_CANdle;

  public CANdleSubsystem() {
    m_CANdle = new CANdle(42);
  }

  public synchronized void setState(ControlRequest animation) {
    m_CANdle.setControl(animation);
  }
}