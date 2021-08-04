// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;

public class DriveTrain_Shift_Low extends InstantCommand {
  private DriveTrain m_driveTrain;

  /** Creates a new DriveTrain_Shift_High. */
  public DriveTrain_Shift_Low(DriveTrain subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain =subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.my_shiftLow();;
  }


}