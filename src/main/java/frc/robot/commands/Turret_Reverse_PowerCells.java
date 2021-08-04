// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.FlyWheel_Vel_PID;
import frc.robot.subsystems.TurretFeed;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Turret_Reverse_PowerCells extends ParallelCommandGroup {
  /** Creates a new Turret_Reverse_PowerCells. */
  public Turret_Reverse_PowerCells(TurretFeed turretFeed, FlyWheel_Vel_PID flyWheel_Vel_PID) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new Turret_Cycle_PowerCells(turretFeed,() -> true,false), new FlyWheel_Run(flyWheel_Vel_PID, () -> -0.2));
  }
}
