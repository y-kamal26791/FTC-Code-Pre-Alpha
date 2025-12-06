package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@TeleOp
public class AprilTagLLTest extends LinearOpMode {

    Limelight3A limelight3A;
    Hardware robot = new Hardware();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(3);
        limelight3A.start();

        waitForStart();
        if (isStopRequested()) return;

        driveToTagDistance(0.40); // stop 40cm away

        while (opModeIsActive()) {
            idle();
        }
    }

    public void driveToTagDistance(double targetDistanceMeters) {

        while (opModeIsActive()) {

            LLResult llResult = limelight3A.getLatestResult();
            if (llResult == null || !llResult.isValid()) continue;

            Pose3D pose3D = llResult.getBotpose_MT2();
            if (pose3D == null) continue;

            Position pos = pose3D.getPosition();

            double x = pos.x;
            double z = pos.z;
            double distance = Math.sqrt(x*x + z*z);

            double error = distance - targetDistanceMeters;
            double kP = 0.9;
            double power = Math.max(-0.4, Math.min(0.4, kP * error));

            setAllDrivePower(power);

            telemetry.addData("Distance", distance);
            telemetry.addData("Target", targetDistanceMeters);
            telemetry.addData("Power", power);
            telemetry.update();

            if (Math.abs(error) < 0.05) {
                setAllDrivePower(0);
                break;
            }

            sleep(10);
        }
    }

    public void setAllDrivePower(double p) {
        robot.frontLeft.setPower(p);
        robot.frontRight.setPower(p);
        robot.backLeft.setPower(p);
        robot.backRight.setPower(p);
    }
}
