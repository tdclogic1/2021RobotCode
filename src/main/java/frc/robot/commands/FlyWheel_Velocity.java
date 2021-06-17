// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlyWheel_Vel_PID;

public class FlyWheel_Velocity extends CommandBase {

  private final FlyWheel_Vel_PID m_FlyWheel_Vel_PID;

  private final DoubleSupplier m_Setpoint;

  /** Creates a new FlyWheel_Vel_PID_Velocity. */
  public FlyWheel_Velocity(FlyWheel_Vel_PID subsystem, DoubleSupplier setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_FlyWheel_Vel_PID = subsystem;
    addRequirements(m_FlyWheel_Vel_PID);

    m_Setpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rpm = m_Setpoint.getAsDouble();
    m_FlyWheel_Vel_PID.my_FlyWheel_Vel_PIDVelocity(rpm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted){
      DriverStation.reportError("FlyWheel_Vel_PID Velocity Interupted", false);
    }else{
      DriverStation.reportError("FlyWheel_Vel_PID Velocity Done", false);
    }
   
    m_FlyWheel_Vel_PID.my_FlyWheel_Vel_PIDPercentOutput(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
