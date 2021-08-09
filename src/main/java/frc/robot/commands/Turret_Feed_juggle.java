// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretFeed;

public class Turret_Feed_juggle extends CommandBase {
  private TurretFeed m_turretFeed;
  /** Creates a new Turret_Feed_juggle. */
  public Turret_Feed_juggle(TurretFeed turretFeed) {
    m_turretFeed = turretFeed;
    addRequirements(m_turretFeed);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  private double scancount;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    scancount = scancount + 1;

    if(scancount<50){
      m_turretFeed.my_AgitatorRun(0.2);
    }else if(scancount<100){
      m_turretFeed.my_AgitatorRun(-0.2);
    }else{
      scancount = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turretFeed.my_AgitatorRun(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
