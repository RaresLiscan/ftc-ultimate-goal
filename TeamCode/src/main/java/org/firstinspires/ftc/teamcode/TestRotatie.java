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
        //robot.lansareRing.setPosition(0.5);
        robot.zeroPowerBeh();
        waitForStart();

        if (opModeIsActive()) {
            robot.runUsingEncodersLongRun(robot.cmToTicks(150), 1, 4);
        }

    }
}