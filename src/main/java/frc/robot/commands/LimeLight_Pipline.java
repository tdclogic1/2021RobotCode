// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import oi.limelightvision.limelight.frc.LimeLight;

public class LimeLight_Pipline extends CommandBase {
  private LimeLight m_limeLight;
  private int m_pipeline;

  /** Creates a new LimeLight_Pipline. */
  public LimeLight_Pipline(LimeLight limelight, int pipeline) {
    m_limeLight = limelight;
    m_pipeline = pipeline;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriverStation.reportError("LimeLight Pipeline", false);
    m_limeLight.setPipeline(m_pipeline);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_limeLight.setPipeline(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
