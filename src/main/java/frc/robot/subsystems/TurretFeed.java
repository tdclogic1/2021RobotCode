// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretFeed extends SubsystemBase {

  private WPI_VictorSPX motorAgitator;
  private WPI_VictorSPX motorIndexer;

  /** Creates a new TurretFeed. */
  public TurretFeed() {
    motorAgitator = new WPI_VictorSPX(Constants.kCAN_TurretAgitator);

    motorIndexer = new WPI_VictorSPX(Constants.kCAN_TurretIndexer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

public void my_IndexerRun(double speed) {
  motorIndexer.set(speed);
}

public void my_AgitatorRun(double speed) {
  motorAgitator.set(speed);
}
}
