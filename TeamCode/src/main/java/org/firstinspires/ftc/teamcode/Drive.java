package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drive {

	private DcMotorEx fl;
	private DcMotorEx fr;
	private DcMotorEx rl;
	private DcMotorEx rr;


	private IMU imu;


	// constructor class
	public Drive(HardwareMap hardwareMap) {
		// motor setup
		fl = hardwareMap.get(DcMotorEx.class, "FrontLeft");
		fr = hardwareMap.get(DcMotorEx.class, "FrontRight");
		rl = hardwareMap.get(DcMotorEx.class, "RearLeft");
		rr = hardwareMap.get(DcMotorEx.class, "RearRight");

		fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		// IMU setup
		imu = hardwareMap.get(IMU.class, "imu");
		IMU.Parameters params = new IMU.Parameters(
			new RevHubOrientationOnRobot(
					// change values based on real robot
					RevHubOrientationOnRobot.LogoFacingDirection.UP,
					RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
			)
		);
		imu.initialize(params);
		imu.resetYaw();

	}

	public void mechanumDrive(double movementY, double movementX, double rotation, double MAX_TICKS_PS) {
		double frontLeft    =   movementY + movementX + rotation;
		double rearLeft     =   movementY - movementX + rotation;
		double frontRight   =   movementY - movementX - rotation;
		double rearRight    =   movementY + movementX - rotation;

		// Normalizing
		double max = Math.abs(frontLeft);
		max = Math.max(max, Math.abs(frontRight));
		max = Math.max(max, Math.abs(rearLeft));
		max = Math.max(max, Math.abs(rearRight));
		max = Math.max(max, 1.0);

		// Apply normalized values
		frontLeft   /= max;
		rearLeft    /= max;
		frontRight  /= max;
		rearRight   /= max;

		// Apply power
		fl.setVelocity(frontLeft * MAX_TICKS_PS);
		rl.setVelocity(rearLeft * MAX_TICKS_PS);
		fr.setVelocity(frontRight * MAX_TICKS_PS);
		rr.setVelocity(rearRight * MAX_TICKS_PS);
	}

	// when using field centric drive do not forget to reset the heading
	public void fieldCentricDrive(double movementY, double movementX, double rotation, double MAX_TICKS_PS) {
		double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

		// Rotate the input vector by the robots heading
		double rotX = movementX * Math.cos(-heading) - movementY * Math.sin(-heading);
		double rotY = movementX * Math.sin(-heading) + movementY * Math.cos(-heading);

		// feed into mecanum math
		double frontLeft		=	rotY + rotX + rotation;
		double rearLeft		=	rotY - rotX + rotation;
		double frontRight	= 	rotY - rotX - rotation;
		double rearRight	=	rotY + rotX - rotation;

		// normalize
		double max = Math.abs(frontLeft);
		max = Math.max(max, Math.abs(frontRight));
		max = Math.max(max, Math.abs(rearLeft));
		max = Math.max(max, Math.abs(rearRight));
		max = Math.max(max, 1.0);

		// Apply normalized values
		frontLeft   /= max;
		rearLeft    /= max;
		frontRight  /= max;
		rearRight   /= max;

		// Apply power
		fl.setVelocity(frontLeft * MAX_TICKS_PS);
		rl.setVelocity(rearLeft * MAX_TICKS_PS);
		fr.setVelocity(frontRight * MAX_TICKS_PS);
		rr.setVelocity(rearRight * MAX_TICKS_PS);
	}

}
