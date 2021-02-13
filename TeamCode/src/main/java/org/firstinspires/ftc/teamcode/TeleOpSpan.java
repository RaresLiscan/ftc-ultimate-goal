package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp Span")
public class TeleOpSpan extends LinearOpMode {

    RobotMap robot;
    ElapsedTime runtime;
    private final double COMMAND_DELAY = 0.35;
    private double ridicareShooterPosition = 1;
    private final double RIDICARE_SHOOTER_SPEED = 0.001;

    private final double TOWER_GOAL_HEIGHT = 0.61;
    private final double POWER_SHOT_HEIGHT = 0.7;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotMap(hardwareMap, this);
        runtime = new ElapsedTime();
        runtime.reset();

        robot.servoWobble.setPosition(1);

        waitForStart();

        while(opModeIsActive()) {


            //Fata - spate + rotire
            double forward = -gamepad1.left_stick_y;
            double leftRight = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            if (robot.motorShooter.getPower() > 0) {
                rotate /= 4;
            }

            robot.teleOpDrive(forward, rotate);
//            robot.driveFieldRelative3(leftRight, forward, rotate);
            //Shooter si intake
            if ((gamepad1.right_bumper || gamepad2.right_bumper) && runtime.seconds() > COMMAND_DELAY){
                if (robot.motorShooter.getPower() != 0) {
                    robot.motorShooter.setPower(0);
                }
                else {
                    robot.motorShooter.setPower(1);
                }
                robot.motorIntake.setPower(0);
                runtime.reset();
            }
            if ((gamepad1.left_bumper || gamepad2.left_bumper) && runtime.seconds() > COMMAND_DELAY){
                robot.motorShooter.setPower(0);
                if (robot.motorIntake.getPower() != 0) {
                    robot.motorIntake.setPower(0);
                }
                else {
                    robot.motorIntake.setPower(-1);
                }
                runtime.reset();
            }

            //Lansare inel
            if (gamepad1.left_trigger > 0.5) {
                robot.servoIntake.setPosition(1);
                sleep(600);
                robot.servoIntake.setPosition(0.5);
            }
            if (gamepad1.right_trigger == 0) {
                robot.servoIntake.setPosition(0.5);
            }
            if (gamepad1.right_trigger == 1) {
                robot.servoIntake.setPosition(1);
                sleep(500);
                robot.servoIntake.setPosition(0.6);
                sleep(350);
                robot.servoIntake.setPosition(1);
                sleep(350);
                robot.servoIntake.setPosition(0.6);
                sleep(350);
                robot.servoIntake.setPosition(1);
                sleep(350);
                robot.servoIntake.setPosition(0);
            }

            if (gamepad2.dpad_up) {
                robot.ridicareShooter.setPosition(TOWER_GOAL_HEIGHT);
            }
            if (gamepad2.dpad_down) {
                robot.ridicareShooter.setPosition(POWER_SHOT_HEIGHT);
            }

            telemetry.addData("Ridicare servo position: ", robot.ridicareShooter.getPosition());
            telemetry.update();

            // Ridicare wobble
            if (gamepad2.y){
                robot.servoRidicare.setPosition(0);
            }
            else if (gamepad2.a && robot.servoWobble.getPosition() != 0){
                robot.servoRidicare.setPosition(1);
            }
            else if (gamepad2.b){
                robot.servoRidicare.setPosition(0.5);
            }

            //prindere wobble
            if (gamepad2.x && runtime.seconds() > COMMAND_DELAY){
                if (robot.servoWobble.getPosition() > 0.7) {
                    robot.servoWobble.setPosition(0);
                }
                else {
                    robot.servoWobble.setPosition(1);
                }
                runtime.reset();
            }

        }
    }
}
