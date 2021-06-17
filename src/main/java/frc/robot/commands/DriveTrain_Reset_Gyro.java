// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;

public class DriveTrain_Reset_Gyro extends InstantCommand {

  private DriveTrain m_driverTrain;

  /** Creates a new DriveTrain_Reset_Gyro. */
  public DriveTrain_Reset_Gyro(DriveTrain subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driverTrain = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driverTrain.resetHeadingGyro();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
