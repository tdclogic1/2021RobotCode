// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlyWheel;

public class FlyWheel_Velocity extends CommandBase {

  private final FlyWheel m_FlyWheel;

  private final DoubleSupplier m_Setpoint;

  /** Creates a new FlyWheel_Velocity. */
  public FlyWheel_Velocity(FlyWheel subsystem, DoubleSupplier setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_FlyWheel = subsystem;
    addRequirements(m_FlyWheel);

    m_Setpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rpm = m_Setpoint.getAsDouble();
    m_FlyWheel.my_FlyWheelVelocity(rpm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted){
      DriverStation.reportError("FlyWheel Velocity Interupted", false);
    }else{
      DriverStation.reportError("FlyWheel Velocity Done", false);
    }
   
    m_FlyWheel.my_FlyWheelPercentOutput(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
