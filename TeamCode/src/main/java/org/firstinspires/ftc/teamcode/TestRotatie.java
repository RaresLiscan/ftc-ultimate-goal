package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="test rotatie")
//@Disabled
public class TestRotatie extends LinearOpMode {

    private RobotMap robot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotMap(hardwareMap, this);
        //robot.servoWobble.setPosition(0);
        //robot.servoIntake.setPosition(0.5);
        robot.zeroPowerBeh();
        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("Motor mode: ", robot.dreaptaFata.getMode());
            telemetry.addData("Other motor mode: ", robot.dreaptaSpate.getMode());
            telemetry.update();
            sleep(4000);
            robot.runUsingEncoders(500, 0.4, 2);
            telemetry.addData("Motor mode: ", robot.dreaptaFata.getMode());
            telemetry.addData("Other motor mode: ", robot.dreaptaSpate.getMode());
            telemetry.update();
            sleep(4000);
            robot.rotateConstantSpeed(90, 0.4, 5);
            telemetry.addData("Motor mode: ", robot.dreaptaFata.getMode());
            telemetry.addData("Other motor mode: ", robot.dreaptaSpate.getMode());
            telemetry.update();
            sleep(4000);
        }

    }
}