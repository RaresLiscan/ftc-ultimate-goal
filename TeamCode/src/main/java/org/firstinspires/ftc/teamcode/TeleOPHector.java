package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Hector TeleOP", group="Linear Opmode")

public class TeleOPHector extends LinearOpMode {

    private RobotMap robot = null;



    @Override
    public void runOpMode() {

        ElapsedTime runtime = new ElapsedTime();
        robot = new RobotMap(hardwareMap, this);
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double stangaFataPower;
            double dreaptaFataPower;
            double stangaSpatePower;
            double dreaptaSpatePower;


            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;

            stangaFataPower  = robot.SQRT(-drive - turn) ;
            dreaptaFataPower= robot.SQRT(drive - turn) ;
            stangaSpatePower  = robot.SQRT(-drive - turn) ;
            dreaptaSpatePower= robot.SQRT(drive - turn) ;


            robot.stangaFata.setPower(stangaFataPower);
            robot.dreaptaFata.setPower(dreaptaFataPower);
            robot.stangaSpate.setPower(stangaSpatePower);
            robot.dreaptaSpate.setPower(dreaptaSpatePower);

            telemetry.addData("Unghi: ", robot.getAngle());
            telemetry.update();


            if (gamepad1.x){
                robot.servoWobble.setPosition(1);

            }
            else if (gamepad1.x && robot.servoWobble.getPosition() != 1) {
                robot.servoWobble.setPosition(0);

            }

        }
    }
}
