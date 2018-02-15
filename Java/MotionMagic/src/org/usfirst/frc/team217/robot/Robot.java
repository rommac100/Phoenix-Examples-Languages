/**
 * Example demonstrating the motion magic control mode.
 * Tested with Logitech F710 USB Gamepad inserted into Driver Station.
 * 
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor() below.
 *
 * After deploying/debugging this to your RIO, first use the left Y-stick 
 * to throttle the Talon manually.  This will confirm your hardware setup/sensors
 * and will allow you to take initial measurements.
 * 
 * Be sure to confirm that when the Talon is driving forward (green) the 
 * position sensor is moving in a positive direction.  If this is not the 
 * cause, flip the boolean input to the setSensorPhase() call below.
 *
 * Once you've ensured your feedback device is in-phase with the motor,
 * and followed the walk-through in the Talon SRX Software Reference Manual,
 * use button1 to motion-magic servo to target position specified by the gamepad stick.
 */
package org.usfirst.frc.team217.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.*;

public class Robot extends IterativeRobot {
	TalonSRX _talonFrontLeft = new TalonSRX(13);
	TalonSRX _talonBackLeft = new TalonSRX(14);
	TalonSRX _talonFrontRight = new TalonSRX(11);
	TalonSRX _talonBackRight = new TalonSRX(12);
	Joystick _joy = new Joystick(0);
	StringBuilder _sb = new StringBuilder();

	public void robotInit() {
		setInfo(_talonFrontLeft);
		setInfo(_talonFrontRight);
		_talonFrontLeft.setInverted(true);
		_talonFrontRight.setInverted(false);
		_talonBackRight.setInverted(false);
		_talonBackLeft.setInverted(true);
		
	}
	
	public void setInfo(TalonSRX _talon)
	{
		/* first choose the sensor */
		_talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		_talon.setSensorPhase(false);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		/* set the peak and nominal outputs */
		_talon.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon.configPeakOutputForward(1, Constants.kTimeoutMs);
		_talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* set closed loop gains in slot0 - see documentation */
		_talon.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		_talon.config_kF(0, 0.1030543579, Constants.kTimeoutMs);
		_talon.config_kP(0, 0.02095*2*2*2*2*2, Constants.kTimeoutMs);
		_talon.config_kI(0, 0, Constants.kTimeoutMs);
		_talon.config_kD(0, 0, Constants.kTimeoutMs);
		/* set acceleration and vcruise velocity - see documentation */
		_talon.configMotionCruiseVelocity(7445, Constants.kTimeoutMs);
		_talon.configMotionAcceleration(7445, Constants.kTimeoutMs);
		/* zero the sensor */
		_talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
	}
	
	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		_talonBackLeft.set(ControlMode.Follower, _talonFrontLeft.getDeviceID());
		_talonBackRight.set(ControlMode.Follower, _talonFrontRight.getDeviceID());
		/* get gamepad axis - forward stick is positive */
		double leftYstick = -1.0 * _joy.getY();
		leftYstick *= Math.abs(leftYstick*leftYstick);
		/* calculate the percent motor output */
		/* calculate the percent motor output */
		double motorOutput = _talonFrontLeft.getMotorOutputPercent();
		double motorOutput1 = _talonFrontRight.getMotorOutputPercent();
		
		/* prepare line to print */
		_sb.append("\tOut%:");
		_sb.append(motorOutput);
		_sb.append("\tVel:");
		_sb.append(_talonFrontLeft.getSelectedSensorVelocity(Constants.kPIDLoopIdx));
		
		_sb.append("\tOut1%:");
		_sb.append(motorOutput1);
		_sb.append("\tVel1:");
		_sb.append(_talonFrontRight.getSelectedSensorVelocity(Constants.kPIDLoopIdx));
		
		if (_joy.getRawButton(1)) {
			/* Motion Magic - 4096 ticks/rev * 10 Rotations in either direction */
			double targetPos = leftYstick * 8192 * 3;
			_talonFrontLeft.set(ControlMode.MotionMagic, targetPos);
			_talonFrontRight.set(ControlMode.MotionMagic, targetPos);
			/* append more signals to print when in speed mode. */
			_sb.append("\terr:");
			_sb.append(_talonFrontLeft.getClosedLoopError(Constants.kPIDLoopIdx));
			
			_sb.append("\terr1:");
			_sb.append(_talonFrontRight.getClosedLoopError(Constants.kPIDLoopIdx));
			
			_sb.append("\ttrg:");
			_sb.append(targetPos);
		} else {
			/* Percent voltage mode */
			_talonFrontRight.set(ControlMode.PercentOutput, leftYstick);
			_talonFrontLeft.set(ControlMode.PercentOutput, leftYstick);
		}
		/* instrumentation */
		Instrum.Process(_talonFrontLeft, _sb);
		try {
			TimeUnit.MILLISECONDS.sleep(10);
		} catch (Exception e) {
		}
	}
}
