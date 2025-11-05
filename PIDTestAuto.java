//tune this. this does not have IMU for now, but the IMU should be added eventually for better control and accuracy

package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class PIDTestAuto extends LinearOpMode {

    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
	
	

// TUNE THESE. PID IS VERY SENSITIVE. Increase these by small numbers each time. that is why we need to use OnBot first. These may need to be changed frequently because PID needs tuning when there is a weight change.
	
    double Kp = 0.005;        //Tune this first. If the robot barely moves, increase. It the robot overshoots the target, reduce. 
	
    double Ki = 0.0;        //apparently this should be kept at zero allways, unless if the robot stops short of the target every time. If the robot drifts more, turn it back off
	
    double Kd = 0.0005;        //Tune this second. Apparently, this should be kept relatively small. If the bot overshoots and shakes, increase. If the bot becomes sluggish, reduce.
	
	

    @Override
    public void runOpMode() {
        
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
		frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
		backLeftMotor.setDirection(DcMotor.Direction.FORWARD;

        waitForStart();

        if (opModeIsActive()) {
            
		
		//Put auton sequence here
					
					
				// Move forward 1000 ticks
            pidDrive(1000, Kp, Ki, Kd);
            sleep(500);



				//I commented out the rest of the sequence for now so we can tune the PID using only the first instruction
				
				
				// Move backward 1000 ticks
            //pidDrive(-1000, Kp, Ki, Kd);
            //sleep(500);

				// Strafe right 1200 ticks
            //pidStrafe(1200, Kp, Ki, Kd);
            //sleep(500);

				// Strafe left 1200 ticks
            //pidStrafe(-1200, Kp, Ki, Kd);

				//stop
            stopMotors();
        }
    }

// this is for moving forward and backward. Positive is forward, negative is backward
    public void pidDrive(double targetPosition, double Kp, double Ki, double Kd) {
        resetEncoders();

        double error;
        double integral = 0;
        double derivative;
        double previousError = 0;
        double output;

        while (opModeIsActive()) {
            double currentPosition = (
                Math.abs(frontLeftMotor.getCurrentPosition()) +
                Math.abs(frontRightMotor.getCurrentPosition()) +
                Math.abs(backLeftMotor.getCurrentPosition()) +
                Math.abs(backRightMotor.getCurrentPosition())
            ) / 4.0;

            error = targetPosition - currentPosition;
            integral += error;
            derivative = error - previousError;

            output = (Kp * error) + (Ki * integral) + (Kd * derivative);
            output = Math.max(-1, Math.min(1, output));

            // check this
            frontLeftMotor.setPower(output);
            frontRightMotor.setPower(output);
            backLeftMotor.setPower(output);
            backRightMotor.setPower(output);

            telemetry.addData("Mode", "Driving");
            telemetry.addData("Target", targetPosition);
            telemetry.addData("Current", currentPosition);
            telemetry.addData("Error", error);
            telemetry.addData("Power", output);
            telemetry.update();

            if (Math.abs(error) < 20) break;

            previousError = error;
        }

        stopMotors();
    }

    // this is for strafing. negative is for going left, positive is for going right
    public void pidStrafe(double targetPosition, double Kp, double Ki, double Kd) {
        resetEncoders();

        double error;
        double integral = 0;
        double derivative;
        double previousError = 0;
        double output;

        while (opModeIsActive()) {
            double currentPosition = (
                Math.abs(frontLeftMotor.getCurrentPosition()) +
                Math.abs(frontRightMotor.getCurrentPosition()) +
                Math.abs(backLeftMotor.getCurrentPosition()) +
                Math.abs(backRightMotor.getCurrentPosition())
            ) / 4.0;

            error = targetPosition - currentPosition;
            integral += error;
            derivative = error - previousError;

            output = (Kp * error) + (Ki * integral) + (Kd * derivative);
            output = Math.max(-1, Math.min(1, output));

            // Strafing pattern. test these and change 
            frontLeftMotor.setPower(output);
            frontRightMotor.setPower(-output);
            backLeftMotor.setPower(-output);
            backRightMotor.setPower(output);

            telemetry.addData("Mode", "Strafing");
            telemetry.addData("Target", targetPosition);
            telemetry.addData("Current", currentPosition);
            telemetry.addData("Error", error);
            telemetry.addData("Power", output);
            telemetry.update();

            if (Math.abs(error) < 20) break;

            previousError = error;
        }

        stopMotors();
    }

    // this is fro stopping
    public void stopMotors() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
	


    public void resetEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



		//the reason why I made it run without encoder here is because since we have logic to control the motor powers and stuff. 
		//Also, since we are making our own PID control, we don't want the control hub to take control of this 
		//(it has a built-in PIDF system that cn't be tuned as easily and is less accurate)
		
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
