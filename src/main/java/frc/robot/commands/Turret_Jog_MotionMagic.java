// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretAim_MM;

public class Turret_Jog_MotionMagic extends CommandBase {
  private final TurretAim_MM m_TurretAim_MM;

  private  DoubleSupplier m_jograte;
  private boolean m_continueToServo;
  private double m_Setpoint;
  
 
  public Turret_Jog_MotionMagic(TurretAim_MM subsystem, DoubleSupplier jograte) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    m_TurretAim_MM = subsystem;
    addRequirements(m_TurretAim_MM);
    m_jograte = jograte;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double joystick = deadband(m_jograte.getAsDouble());
    m_Setpoint = get_Jog_Setpoint(joystick);	

    SmartDashboard.putNumber("Jog joystick", joystick);
    SmartDashboard.putNumber("Jog Setpoint", m_Setpoint);
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

  private double get_Jog_Setpoint(double setpoint){
   
    double maxStep = 10;  

    m_Setpoint = (m_TurretAim_MM.get_currentPos() + (setpoint * maxStep));
    
    if(m_Setpoint > m_TurretAim_MM.get_MaxPos( )){
      return  m_TurretAim_MM.get_MaxPos( );
    }else if(m_Setpoint < m_TurretAim_MM.get_minPos( )){
      return m_TurretAim_MM.get_minPos( );
    }else{
      return m_Setpoint;
    }
  }

  private double deadband(double setpoint){
    double deadband = .1;
    if(Math.abs(setpoint)<deadband){
      return 0.0;
    }else{
      return Math.copySign(setpoint, setpoint);
    }
  }
}
