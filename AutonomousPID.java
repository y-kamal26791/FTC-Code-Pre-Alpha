//test this code and tune PID


package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous
public class RedGoal extends LinearOpMode {

    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private IMU imu;
    private YawPitchRollAngles lastAngles;
    private double globalAngle;

    // PID constants *TUNE THESE*
    double Kp = 0.02;
    double Ki = 0.0;
    double Kd = 0.002;

    @Override
    public void runOpMode() {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        

        // IMU stuff
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
        RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);


        
        telemetry.update();

        resetEncoders();

        waitForStart();

        //  Go backward 700 ticks
        movePID(-700, 0);


        movePID(700, 0);
        //  Strafe left 100 ticks
        //strafePID(-100, 0);

        sleep(10);
        
        //  Turn right to 50 degrees (uses IMU)
        turnToAngle(50);

        //  Stop
        stopMotors();
    }

    //PID drive straight
    private void movePID(int targetTicks, double holdAngle) {
        resetEncoders();

        double integral = 0;
        double lastError = 0;

        while (opModeIsActive()) {
            int avgPos = (frontLeftMotor.getCurrentPosition() + frontRightMotor.getCurrentPosition()
                        + backLeftMotor.getCurrentPosition() + backRightMotor.getCurrentPosition()) / 4;

            double error = targetTicks - avgPos;
            integral += error;
            double derivative = error - lastError;
            double output = (Kp * error) + (Ki * integral) + (Kd * derivative);
            output = Math.max(-1, Math.min(1, output));

            // Heading correction using IMU
            double angleError = holdAngle - getAngle();
            double correction = angleError * 0.01;

            double leftPower = output + correction;
            double rightPower = output - correction;

            frontLeftMotor.setPower(leftPower);
            backLeftMotor.setPower(leftPower);
            frontRightMotor.setPower(rightPower);
            backRightMotor.setPower(rightPower);

            telemetry.addData("Move Target", targetTicks);
            telemetry.addData("Position", avgPos);
            telemetry.addData("Angle", getAngle());
            telemetry.addData("Power", output);
            telemetry.update();

            if (Math.abs(error) < 10) break;
            lastError = error;
        }
        stopMotors();
    }

    //PID strafe
    private void strafePID(int targetTicks, double holdAngle) {
        resetEncoders();

        double integral = 0;
        double lastError = 0;

        while (opModeIsActive()) {
            
            int avgPos = (frontLeftMotor.getCurrentPosition() - frontRightMotor.getCurrentPosition()
                        - backLeftMotor.getCurrentPosition() + backRightMotor.getCurrentPosition()) / 4;

            double error = targetTicks - avgPos;
            integral += error;
            double derivative = error - lastError;
            double output = (Kp * error) + (Ki * integral) + (Kd * derivative);
            output = Math.max(-1, Math.min(1, output));

            double angleError = holdAngle - getAngle();
            double correction = angleError * 0.01;

            //this is apparently supposed to make it easier for mecanum strafing
            double fl = output + correction;
            double fr = -output - correction;
            double bl = -output + correction;
            double br = output - correction;

            frontLeftMotor.setPower(fl);
            frontRightMotor.setPower(fr);
            backLeftMotor.setPower(bl);
            backRightMotor.setPower(br);

            telemetry.addData("Strafe Target", targetTicks);
            telemetry.addData("Position", avgPos);
            telemetry.addData("Angle", getAngle());
            telemetry.addData("Power", output);
            telemetry.update();

            if (Math.abs(error) < 10) break;
            lastError = error;
        }
        stopMotors();
    }

    //PID turning with IMU
    private void turnToAngle(double targetAngle) {
        double error;
        double lastError = 0;
        double integral = 0;

        while (opModeIsActive()) {
            double currentAngle = getAngle();
            error = targetAngle - currentAngle;

            integral += error;
            double derivative = error - lastError;
            double output = (Kp * error) + (Ki * integral) + (Kd * derivative);
            output = Math.max(-0.5, Math.min(0.5, output)); // limit for smooth turn

            // check this again. i think all of them need to be positive but im not sure
            frontLeftMotor.setPower(-output);
            backLeftMotor.setPower(-output);
            frontRightMotor.setPower(output);
            backRightMotor.setPower(output);

            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Current Angle", currentAngle);
            telemetry.update();

            if (Math.abs(error) < 1.0) break;
            lastError = error;
        }
        stopMotors();
    }

    //Encoder stuff
    private void resetEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void stopMotors() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    //more IMU stuff
    private void resetAngle() {
        lastAngles = imu.getRobotYawPitchRollAngles();
        globalAngle = 0;
    }

    private double getAngle() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double yaw = angles.getYaw(AngleUnit.DEGREES);
        
        if (lastAngles == null) {
            
            lastAngles = angles;
        }
        
        double lastYaw = lastAngles.getYaw(AngleUnit.DEGREES);
        double deltaAngle =  yaw - lastYaw;
        
        

        if (deltaAngle < -180) deltaAngle += 360;
        else if (deltaAngle > 180) deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngles = angles;

        return globalAngle;
}

    }

