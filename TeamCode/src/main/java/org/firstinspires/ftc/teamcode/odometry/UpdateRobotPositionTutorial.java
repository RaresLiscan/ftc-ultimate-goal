package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class UpdateRobotPositionTutorial extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight;

    DcMotor leftEncoder, rightEncoder, middleEncoder;
    BNO055IMU imu;

    static final double TICK_PER_REVOLUTION = 1440;
    static final double WHEEL_DIAMETER = 100/25.4;
    static final double GEAR_RATIO = 1;

    static final double TICKS_PER_INCH = WHEEL_DIAMETER * Math.PI * GEAR_RATIO / TICK_PER_REVOLUTION;

    GlobalCoordinateSystemTutorial positionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        leftEncoder = hardwareMap.dcMotor.get("leftEncoder");
        rightEncoder = hardwareMap.dcMotor.get("rightEncoder");
        middleEncoder = hardwareMap.dcMotor.get("middleEncoder");

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        resetOdometryEncoders();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Status: ", "Ready!");
        telemetry.update();

        waitForStart();

        positionUpdate = new GlobalCoordinateSystemTutorial(leftEncoder, rightEncoder, middleEncoder, TICKS_PER_INCH, 100);
        Thread position = new Thread(positionUpdate);
        position.start();

        while (opModeIsActive()) {
            float left = gamepad1.left_stick_y;
            float right = gamepad1.left_stick_y;
            float rotation = gamepad1.right_stick_x;

            frontLeft.setPower(left + rotation);
            frontRight.setPower(right + rotation);
            backLeft.setPower(left + rotation);
            backRight.setPower(right + rotation);

            telemetry.addData("X Position: ", positionUpdate.returnXCoordinate() / TICKS_PER_INCH);
            telemetry.addData("Y Position: ", positionUpdate.returnYCoordinate() / TICKS_PER_INCH);
            telemetry.addData("Orientation (degrees): ", positionUpdate.returnOrientation());
            telemetry.update();
        }
        positionUpdate.stop();
    }

    private void resetOdometryEncoders() {
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middleEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
