package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;



public class RobotMap {

    public DcMotor stangaSpate = null;
    public DcMotor dreaptaSpate = null;
    public DcMotor stangaFata = null;
    public DcMotor dreaptaFata = null;
    public DcMotor motorShooter = null;
    public DcMotor motorIntake = null;
    public Servo servoWobble, servoRidicare, servoIntake = null;
    public BNO055IMU imu = null;
    Orientation lastAngles = new Orientation();
    double globalAngle, correction, prevAngle;
    private LinearOpMode opMode;
    private double imuOffset = 0;
    private double maxSpeed = 1.0;



    public RobotMap (HardwareMap hardwareMap, LinearOpMode opMode) {

        stangaFata = hardwareMap.get(DcMotor.class, "stangaFata");
        dreaptaFata = hardwareMap.get(DcMotor.class, "dreaptaFata");
        stangaSpate = hardwareMap.get(DcMotor.class, "stangaSpate");
        dreaptaSpate = hardwareMap.get(DcMotor.class, "dreaptaSpate");
        servoWobble = hardwareMap.get(Servo.class, "servoWobble");
        servoRidicare = hardwareMap.get(Servo.class, "servoRidicare");
        servoIntake = hardwareMap.get(Servo.class, "servoIntake");
        motorShooter = hardwareMap.get(DcMotor.class, "motorShooter");
        motorIntake = hardwareMap.get(DcMotor.class, "motorIntake");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        while (imu.isGyroCalibrated());

        this.opMode = opMode;
    }

    public void driveFieldRelativeAngle(double y, double angle) {
        double angle_in = angle + Math.PI;  // convert to robot coordinates
//        opMode.telemetry.addData("angle_in: ", angle_in);
//        opMode.telemetry.addData("prevAngle: ", prevAngle);
//        opMode.telemetry.update();

        double delta = AngleUnit.normalizeRadians(Math.toRadians(getAngle()) - angle_in);

        double MAX_ROTATE = 0.3; //This is to shrink how fast we can rotate so we don't fly past the angle
//        if (y <= 0.05) {
//            delta = 0;
//        }
//        else {
////            delta = Range.clip(delta, -MAX_ROTATE, MAX_ROTATE);
//            delta *= 0.4;
//        }
        if (y <= 0.05) {
            delta = 0;
        }
        else if (y < 0.5) {
            y /= 1.5;
            delta *= 0.3;
        }
        else {
            delta *= 0.3;
        }
        teleOpDrive(y, delta);
    }



    public void driveRelativeManualRotation(double y, double angle, double rotationSpeed) {
        double angle_in = angle + Math.PI/2;  // convert to robot coordinates
//        opMode.telemetry.addData("Unghi: ", angle);
//        opMode.telemetry.update();

        double delta = AngleUnit.normalizeRadians(Math.toRadians(getAngle()) - angle_in);

        double MAX_ROTATE = 1; //This is to shrink how fast we can rotate so we don't fly past the angle

        if (delta <= Math.PI / 4) {
            teleOpDrive(y, rotationSpeed);
        }
    }

    void setSpeeds(double frontLeftSpeed, double frontRightSpeed, double backLeftSpeed, double backRightSpeed) {
        double largest = maxSpeed;
        largest = Math.max(largest, Math.abs(frontLeftSpeed));
        largest = Math.max(largest, Math.abs(frontRightSpeed));
        largest = Math.max(largest, Math.abs(backLeftSpeed));
        largest = Math.max(largest, Math.abs(backRightSpeed));

        stangaFata.setPower(frontLeftSpeed/largest);
        stangaSpate.setPower(backLeftSpeed/largest);
        dreaptaFata.setPower(frontRightSpeed/largest);
        dreaptaSpate.setPower(backRightSpeed/largest);
    }

    public void teleOpDrive(double forward, double rotate) {
        double frontLeftSpeed = SQRT(forward - rotate);
        double frontRightSpeed = SQRT(-forward - rotate);
        double backLeftSpeed = SQRT(forward - rotate);
        double backRightSpeed = SQRT(-forward - rotate);

        setSpeeds(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
    }

    public void teleOpDriveRotire(double forward,double rotate){

        double frontLeftSpeed = SQRT(0- rotate);
        double frontRightSpeed = SQRT(0- rotate);
        double backLeftSpeed = SQRT(0 - rotate);
        double backRightSpeed = SQRT( 0- rotate);

        setSpeeds(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
    }

    public void runUsingEncoders (int distance, double power, double timeout) {
        ElapsedTime runtime = new ElapsedTime();

        stangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        opMode.sleep(50);

        stangaFata.setTargetPosition(distance);
        stangaSpate.setTargetPosition(distance);
        dreaptaFata.setTargetPosition(-distance);
        dreaptaSpate.setTargetPosition(-distance);

        stangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double p = 0.05;

        stangaFata.setPower(p);
        stangaSpate.setPower(p);
        dreaptaFata.setPower(p);
        dreaptaSpate.setPower(p);

        runtime.reset();

        while (opMode.opModeIsActive() && stangaSpate.isBusy() && stangaFata.isBusy() && dreaptaSpate.isBusy() && dreaptaFata.isBusy() && runtime.seconds() < timeout) {
            if (p < power) p += 0.05;

            stangaFata.setPower(p);
            stangaSpate.setPower(p);
            dreaptaFata.setPower(p);
            dreaptaSpate.setPower(p);

        }

        stopDriving();

    }


    public void zeroPowerBeh() {
        stangaSpate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stangaFata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreaptaFata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreaptaSpate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public double SQRT(double x) {
        if (x < 0) return -Math.sqrt(Math.abs(x));
        return Math.sqrt(x);
    }

    public void rotateConstantSpeed(int degrees, double power, int timeout)
    {

        opMode.sleep(300);
        dreaptaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stangaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dreaptaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stangaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ElapsedTime runtime = new ElapsedTime();

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
//            stangaFata.setPower(power);
//            dreaptaSpate.setPower(-power);
//            stangaSpate.setPower(power);
//            dreaptaFata.setPower(-power);
            power = -power;
        }
        else if (degrees > 0)
        {   // turn left.
//            stangaFata.setPower(power);
//            dreaptaSpate.setPower(-power);
//            stangaSpate.setPower(power);
//            dreaptaFata.setPower(-power);
        }
        else return;

        opMode.telemetry.addData("Motor Mode: ", dreaptaFata.getMode());
        opMode.telemetry.addData("Other motor mode: ", dreaptaSpate.getMode());
        opMode.telemetry.update();
        opMode.sleep(5000);


        // set power to rotate.
        stangaFata.setPower(power);
        dreaptaFata.setPower(power);
        stangaSpate.setPower(power);
        dreaptaSpate.setPower(power);

        // rotate until turn is completed.
        if (degrees < 0)
        {

            while (opMode.opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opMode.opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        stopDriving();
        //        rightMotor.setPower(0);
        //        leftMotor.setPower(0);

        // wait for rotation to stop.
        opMode.sleep(200);
        // reset angle tracking on new heading.
        resetAngle();
    }

    public void rotate(int degrees, double power, int timeout)
    {

        stangaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dreaptaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stangaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dreaptaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ElapsedTime runtime = new ElapsedTime();

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            stangaFata.setPower(power);
            dreaptaSpate.setPower(-power);
            stangaSpate.setPower(power);
            dreaptaFata.setPower(-power);
            power = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            stangaFata.setPower(power);
            dreaptaSpate.setPower(-power);
            stangaSpate.setPower(power);
            dreaptaFata.setPower(-power);


        }
        else return;


        // set power to rotate.
        stangaFata.setPower(power);
        dreaptaFata.setPower(power);
        stangaSpate.setPower(power);
        dreaptaSpate.setPower(power);


        //        leftMotor.setPower(leftPower);
        //        rightMotor.setPower(rightPower);

        // rotate until turn is completed.
        boolean powerModified = false;
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opMode.opModeIsActive() && getAngle() == 0 && runtime.seconds() < timeout) { }

            while (opMode.opModeIsActive() && getAngle() > degrees && runtime.seconds() < timeout) {
//                if (getAngle() - degrees < 30 && !powerModified) {
//                    //lower the power in order to rotate with more precision
//                    power /= 2;
//                    stangaFata.setPower(power);
//                    dreaptaFata.setPower(power);
//                    stangaSpate.setPower(power);
//                    dreaptaSpate.setPower(power);
//                    powerModified = true;
//                }
            }
        }
        else    // left turn.
            while (opMode.opModeIsActive() && getAngle() < degrees && runtime.seconds() < timeout) {
//                if (degrees - getAngle() < 30 && !powerModified) {
//                    //lower the power in order to rotate with more precision
//                    power /= 2;
//                    stangaFata.setPower(power);
//                    dreaptaFata.setPower(power);
//                    stangaSpate.setPower(power);
//                    dreaptaSpate.setPower(power);
//                    powerModified = true;
//                }
            }

        // turn the motors off.
        stopDriving();
        //        rightMotor.setPower(0);
        //        leftMotor.setPower(0);

        // wait for rotation to stop.

        // reset angle tracking on new heading.
//        resetAngle();
    }

    public double getHeading(AngleUnit angleUnit) {
        Orientation angles;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        return angleUnit.fromRadians(angles.firstAngle + imuOffset);
    }

    public double getCurrentAngle() {
        Orientation angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentAngle = angle.firstAngle;
        if (currentAngle < -180) {
            while (currentAngle < -180) {
                currentAngle += 360;
            }
        }
        else {
            while (currentAngle > 180) {
                currentAngle -= 360;
            }
        }
        return currentAngle;
    }

    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public void stopDriving() {
        stangaSpate.setPower(0);
        dreaptaSpate.setPower(0);
        stangaFata.setPower(0);
        dreaptaFata.setPower(0);
    }


}