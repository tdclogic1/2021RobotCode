// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.TurretAim_MM;
import oi.limelightvision.limelight.frc.LimeLight;

public class Turret_Vision_MotionMagic extends CommandBase {
  private final TurretAim_MM m_TurretAim_MM;

  private final LimeLight m_limeLight;
  private boolean m_continueToServo;
  private double m_Setpoint;
  private DoubleSupplier m_gyroAngle;
  private double m_iterationsSinceLostTarget = 0;
  private final double m_LostTarget_Iterations = 10;

  public Turret_Vision_MotionMagic(TurretAim_MM subsystem, DoubleSupplier gyroAngle, LimeLight limeLight) {

    // Use addRequirements() here to declare subsystem dependencies.
    m_TurretAim_MM = subsystem;
    addRequirements(m_TurretAim_MM);
    m_gyroAngle = gyroAngle;
    m_limeLight = limeLight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limeLight.setPipeline(1);
    m_iterationsSinceLostTarget = m_LostTarget_Iterations;
  }


  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double kp = .9;

    if (m_limeLight.getIsTargetFound()) {
      m_Setpoint = get_Jog_Setpoint(m_limeLight.getdegRotationToTarget() * kp);
      m_TurretAim_MM.my_Aim_MotoionMagic(m_Setpoint);
      m_iterationsSinceLostTarget = 0;
    } else {
      m_iterationsSinceLostTarget++;
      //Delay before returning to Gyro Searching
      if (m_iterationsSinceLostTarget >= m_LostTarget_Iterations) {
          double m_Setpoint = MathUtil.inputModulus(m_gyroAngle.getAsDouble(), -70, 290);
          m_TurretAim_MM.my_Aim_MotoionMagic(m_Setpoint);
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_limeLight.setPipeline(0);
    m_TurretAim_MM.my_Aim_PercentOutput(0.0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double get_Jog_Setpoint(double setpoint) {

    m_Setpoint = (m_TurretAim_MM.get_currentPos() + (setpoint));

    if (m_Setpoint > m_TurretAim_MM.get_MaxPos()) {
      return m_TurretAim_MM.get_MaxPos();
    } else if (m_Setpoint < m_TurretAim_MM.get_minPos()) {
      return m_TurretAim_MM.get_minPos();
    } else {
      return m_Setpoint;
    }
  }

  private double deadband(double setpoint) {
    double deadband = .1;
    if (Math.abs(setpoint) < deadband) {
      return 0.0;
    } else {
      return Math.copySign(setpoint, setpoint);
    }
  }
}
