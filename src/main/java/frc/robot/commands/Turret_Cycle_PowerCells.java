// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretFeed;

public class Turret_Cycle_PowerCells extends CommandBase {
  private final TurretFeed m_turretFeed;

  /** Creates a new Turret_Cycle_PowerCells. */
  public Turret_Cycle_PowerCells(TurretFeed subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turretFeed = subsystem;
    addRequirements(m_turretFeed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_turretFeed.my_IndexerRun(.75);
    m_turretFeed.my_AgitatorRun(.75);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turretFeed.my_IndexerRun(0.0);
    m_turretFeed.my_AgitatorRun(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
