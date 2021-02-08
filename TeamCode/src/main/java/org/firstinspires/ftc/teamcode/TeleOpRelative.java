package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Relative TeleOP", group="Linear Opmode")

public class TeleOpRelative extends LinearOpMode {

    private RobotMap robot = null;


    @Override
    public void runOpMode() {

        ElapsedTime runtime = new ElapsedTime();
        robot = new RobotMap(hardwareMap, this);
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {


            Polar rotation = Polar.fromCartesian(gamepad1.left_stick_x, -gamepad1.left_stick_y);

            if (Math.abs(gamepad1.right_stick_x) > 0.1) {
                robot.teleOpDrive(0, gamepad1.right_stick_x);
            }
            else {
                robot.driveFieldRelativeAngle(rotation.getR(), rotation.getTheta());
            }

        }
    }
}
