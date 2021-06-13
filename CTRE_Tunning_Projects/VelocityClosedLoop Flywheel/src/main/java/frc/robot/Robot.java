/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Description:
 * The VelocityClosedLoop example demonstrates the velocity closed-loop servo.
 * Tested with Logitech F350 USB Gamepad inserted into Driver Station]
 * 
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor() below.
 * Use Percent Output Mode (Holding A and using Left Joystick) to confirm talon is driving 
 * forward (Green LED on Talon/Victor) when the postion sensor is moving in the postive 
 * direction. If this is not the case, flip the boolean input in setSensorPhase().
 * 
 * Controls:
 * Button 1: When held, start and run Velocity Closed Loop on Talon/Victor
 * Left Joystick Y-Axis:
 * 	+ Percent Output: Throttle Talon forward and reverse, use to confirm hardware setup
 * 	+ Velocity Closed Loop: Servo Talon forward and reverse [-500, 500] RPM
 * 
 * Gains for Velocity Closed Loop may need to be adjusted in Constants.java
 * 
 * Supported Version:
 * - Talon FX: 20.2.3.0
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;;

public class Robot extends TimedRobot {
    /* Hardware */

	TalonFX  motorFlyWheel_Vel_PIDMaster = new TalonFX(12);
	TalonFX motorFlyWheel_Vel_PIDSlave = new TalonFX(3);
       
    Joystick _joy = new Joystick(0);
    
    /* String for output */
    StringBuilder _sb = new StringBuilder();
    
    /* Loop tracker for prints */
	int _loops = 0;

	public void robotInit() {
        /* Factory Default all hardware to prevent unexpected behaviour */
		motorFlyWheel_Vel_PIDMaster.configFactoryDefault();
		motorFlyWheel_Vel_PIDSlave.configFactoryDefault();

		/* Invert if required */
		motorFlyWheel_Vel_PIDMaster.setInverted(true);
		motorFlyWheel_Vel_PIDSlave.setInverted(false);

		/* Set Slave to Follow Master */
		motorFlyWheel_Vel_PIDSlave.follow(motorFlyWheel_Vel_PIDMaster);
		
		/* Config neutral deadband to be the smallest possible */
		motorFlyWheel_Vel_PIDMaster.configNeutralDeadband(0.001);

		/* Config sensor used for Primary PID [Velocity] */
        motorFlyWheel_Vel_PIDMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                                            Constants.kPIDLoopIdx, 
											Constants.kTimeoutMs);
											
											

		/* Config the peak and nominal outputs */
		motorFlyWheel_Vel_PIDMaster.configNominalOutputForward(0, Constants.kTimeoutMs);
		motorFlyWheel_Vel_PIDMaster.configNominalOutputReverse(0, Constants.kTimeoutMs);
		motorFlyWheel_Vel_PIDMaster.configPeakOutputForward(1, Constants.kTimeoutMs);
		motorFlyWheel_Vel_PIDMaster.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* Config the Velocity closed loop gains in slot0 */
		motorFlyWheel_Vel_PIDMaster.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
		motorFlyWheel_Vel_PIDMaster.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
		motorFlyWheel_Vel_PIDMaster.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
		motorFlyWheel_Vel_PIDMaster.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // motorFlyWheel_Vel_PIDMaster.setSensorPhase(true);
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		/* Get gamepad axis */
		double leftYstick = -1 * _joy.getY();

		/* Get Talon/Victor's current output percentage */
		double motorOutput = motorFlyWheel_Vel_PIDMaster.getMotorOutputPercent();
		
		/* Prepare line to print */
		_sb.append("\tout:");
		/* Cast to int to remove decimal places */
		_sb.append((int) (motorOutput * 100));
		_sb.append("%");	// Percent

		_sb.append("\tspd:");
		_sb.append(motorFlyWheel_Vel_PIDMaster.getSelectedSensorVelocity(Constants.kPIDLoopIdx));
		_sb.append("u"); 	// Native units

        /** 
		 * When button 1 is held, start and run Velocity Closed loop.
		 * Velocity Closed Loop is controlled by joystick position x500 RPM, [-500, 500] RPM
		 */
		if (_joy.getRawButton(1)) {
			/* Velocity Closed Loop */

			/**
			 * Convert 500 RPM to units / 100ms.
			 * 2048 Units/Rev * 500 RPM / 600 100ms/min in either direction:
			 * velocity setpoint is in units/100ms
			 */
			double maxRPM = 6380;
			double setpoint = leftYstick;
			double velocityUnitsper100ms = 2048.0 / 600.0;
			double targetVelocity_UnitsPer100ms = setpoint * maxRPM * velocityUnitsper100ms;

			
			/* 500 RPM in either direction */
			motorFlyWheel_Vel_PIDMaster.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);

			/* Append more signals to print when in speed mode. */
			_sb.append("\terr:");
			_sb.append(motorFlyWheel_Vel_PIDMaster.getClosedLoopError(Constants.kPIDLoopIdx));
			_sb.append("\ttrg:");
			_sb.append(targetVelocity_UnitsPer100ms);
		} else {
			/* Percent Output */

			motorFlyWheel_Vel_PIDMaster.set(TalonFXControlMode.PercentOutput, leftYstick);
		}

        /* Print built string every 10 loops */
		if (++_loops >= 10) {
			_loops = 0;
			System.out.println(_sb.toString());
        }
        /* Reset built string */
		_sb.setLength(0);
	}
}
