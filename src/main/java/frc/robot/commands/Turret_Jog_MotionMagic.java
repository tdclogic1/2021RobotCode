// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretAim_MM;

public class Turret_Jog_MotionMagic extends CommandBase {
  private final TurretAim_MM m_TurretAim_MM;
  /** Creates a new Turret_Jog_MotionMagic. */
  public Turret_Jog_MotionMagic(TurretAim_MM subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_TurretAim_MM = subsystem;
    addRequirements(m_TurretAim_MM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
