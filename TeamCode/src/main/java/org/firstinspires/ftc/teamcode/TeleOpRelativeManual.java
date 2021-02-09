package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@TeleOp(name="Manual Relative TeleOP", group="Linear Opmode")
@Disabled
public class TeleOpRelativeManual extends LinearOpMode {

    private RobotMap robot = null;
    double  contPower;

    @Override
    public void runOpMode() {

        ElapsedTime runtime = new ElapsedTime();
        robot = new RobotMap(hardwareMap, this);
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {


            Polar rotation = Polar.fromCartesian(gamepad1.left_stick_x, -gamepad1.left_stick_y);

//            if (Math.abs(gamepad1.right_stick_x) > 0.1) {
//                robot.teleOpDrive(0, gamepad1.right_stick_x);
//            }
//            else {
//                robot.driveFieldRelativeAngle(rotation.getR(), rotation.getTheta());
//            }

            telemetry.addData("Hypothesis: ", rotation.getR());
            telemetry.update();
            robot.driveRelativeManualRotation(rotation.getR(), rotation.getTheta(), gamepad1.right_stick_x);

            // Prindere wobble
            if (gamepad1.dpad_left){
                robot.servoWobble.setPosition(1);
            }
            else if (gamepad1.dpad_right){
                robot.servoWobble.setPosition(0);
            }


            // Servo loader
            if (gamepad1.right_bumper != false) {
                //robot.servoIntake.setPower(1);
            }
            else //robot.servoIntake.setPower(0);


                // Motor shooter
                if (gamepad1.y){
                    robot.motorShooter.setPower(1);
                }
                else if(gamepad1.a){
                    robot.motorShooter.setPower(0);
                }
        }
    }
}
