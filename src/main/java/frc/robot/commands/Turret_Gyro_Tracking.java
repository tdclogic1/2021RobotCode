// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.TurretAim_MM;

public class Turret_Gyro_Tracking extends CommandBase {

  private final TurretAim_MM m_TurretAim_MM;

  private DoubleSupplier m_gyroAngle;

  /** Creates a new Turret_Gyro_Tracking. */
  public Turret_Gyro_Tracking(TurretAim_MM subsystem, DoubleSupplier gyroAngle) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_TurretAim_MM = subsystem;
    addRequirements(m_TurretAim_MM);
    m_gyroAngle = gyroAngle;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double m_Setpoint = MathUtil.inputModulus(m_gyroAngle.getAsDouble(), -70, 290);
    
    m_TurretAim_MM.my_Aim_MotoionMagic(m_Setpoint);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_TurretAim_MM.my_Aim_PercentOutput(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
