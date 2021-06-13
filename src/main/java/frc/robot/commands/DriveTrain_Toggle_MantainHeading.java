// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveTrain_Toggle_MantainHeading extends InstantCommand {
  private DriveTrain m_driverTrain;

  public DriveTrain_Toggle_MantainHeading(DriveTrain driverTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driverTrain = driverTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driverTrain.my_toggle_preserveHeading();
  }
}
