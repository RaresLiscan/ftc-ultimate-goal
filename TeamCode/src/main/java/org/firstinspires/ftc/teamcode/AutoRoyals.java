package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
@Autonomous(name="Auto Demo Royals")
@Disabled
public class AutoRoyals extends LinearOpMode {

    private RobotMap robot = null;

    private final double TOWER_GOAL_HEIGHT = 0.61;
    private final double POWER_SHOT_HEIGHT = 0.7;

    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;


    OpenCvCamera webcam;
    AutoRoyals.SkystoneDeterminationPipeline pipeline;

    /*
     * The core values which define the location and size of the sample regions
     */
    public static Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(180,150);

    public static int REGION_WIDTH = 45;
    public static int REGION_HEIGHT = 35;

    public static int FOUR_RING_THRESHOLD = 143;
    public static int ONE_RING_THRESHOLD = 135;

    @Override
    public void runOpMode() throws InterruptedException {

        dashboard = FtcDashboard.getInstance();
//        dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();

        robot = new RobotMap(hardwareMap, this);
        robot.servoWobble.setPosition(0);
        robot.ridicareShooter.setPosition(POWER_SHOT_HEIGHT);
        robot.servoIntake.setPosition(0.5);
        robot.zeroPowerBeh();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new AutoRoyals.SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
                dashboard.startCameraStream(webcam, 0);
            }
        });

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
                dashboard.startCameraStream(webcam, 0);
            }
        });

        waitForStart();


        if (opModeIsActive()) {

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();
            sleep(500);


//            if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.FOUR) {
//                FourRings();
//            }
//            else if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.ONE) {
//                OneRing();
//            }
//            else if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.NONE) {
//                NoRing();
//            }

        }

    }

    private void FourRings() {

        //Porneste motoarele
        robot.servoRidicare.setPosition(0.8);
        robot.motorShooter.setPower(1);

        //Se pozitioneaza la tragere
        robot.runUsingEncodersLongRun(robot.cmToTicks(160), 1, 3);
        robot.rotateConstantSpeed(2, 0.3, 2);

        //Trage primul ring
        robot.servoIntake.setPosition(1);
        sleep(350);
        robot.servoIntake.setPosition(0.5);

        //Se pozitioneaza la power shot-ul din stanga
        robot.ridicareShooter.setPosition(POWER_SHOT_HEIGHT + 0.04);
        robot.rotateConstantSpeed(9, 0.3, 2);
        sleep(100);

        //Trage al doilea ring
        robot.servoIntake.setPosition(1);
        sleep(350);
        robot.servoIntake.setPosition(0.5);
        sleep(150);

        //Se pozitioneaza la tower goal
        robot.ridicareShooter.setPosition(TOWER_GOAL_HEIGHT);
        robot.rotateConstantSpeed(-23, 0.3, 3);

        //Trage al trilea ring
        robot.servoIntake.setPosition(1);
        sleep(350);
        robot.servoIntake.setPosition(0.5);

        //Opreste shooterul
        robot.motorShooter.setPower(0);

        //Se pozitioneaza pentru wobble si il duce
        robot.rotateConstantSpeed(-14, 0.4, 2);
        robot.runUsingEncodersLongRun(robot.cmToTicks(160), 1, 5);

        //lasa primul wobble
        robot.servoWobble.setPosition(0.8);
        robot.servoRidicare.setPosition(1);
        sleep(500);

        //Se intoarce dupa al doilea
        robot.runUsingEncoders(robot.cmToTicks(-25), 1, 2);
        robot.rotateConstantSpeed(-143, 0.4, 3);
        //Se duce dupa al doilea
        robot.runUsingEncodersLongRun(robot.cmToTicks(250),1,7);

        //Prinde al doilea wobble
        robot.servoWobble.setPosition(0);
        sleep(500);
        robot.servoRidicare.setPosition(0.8);

        //Se intoarce catre patrat
        robot.rotateConstantSpeed(165, 0.65, 5);

        //Duce al doilea wobble
        robot.runUsingEncodersLongRun(robot.cmToTicks(247),1,7);


        //Lasa al doilea wobble
        robot.servoWobble.setPosition(0.8);
        robot.servoRidicare.setPosition(1);
        sleep(500);

        //Parcheaza
        robot.runUsingEncodersLongRun(robot.cmToTicks(-85),1,5);
    }

    private void NoRing(){

        //Porneste motoarele
        robot.servoRidicare.setPosition(0.8);
        robot.motorShooter.setPower(1);

        //Se pozitioneaza la tragere
        robot.runUsingEncodersLongRun(robot.cmToTicks(160), 1, 3);
        robot.rotateConstantSpeed(2, 0.3, 2);

        //Trage primul ring
        robot.servoIntake.setPosition(1);
        sleep(350);
        robot.servoIntake.setPosition(0.5);

        //Se pozitioneaza la power shot-ul din stanga
        robot.ridicareShooter.setPosition(POWER_SHOT_HEIGHT + 0.04);
        robot.rotateConstantSpeed(9, 0.3, 2);
        sleep(100);

        //Trage al doilea ring
        robot.servoIntake.setPosition(1);
        sleep(350);
        robot.servoIntake.setPosition(0.5);
        sleep(150);

        //Se pozitioneaza la tower goal
        robot.ridicareShooter.setPosition(TOWER_GOAL_HEIGHT);
        robot.rotateConstantSpeed(-23, 0.3, 3);

        //Trage al trilea ring
        robot.servoIntake.setPosition(1);
        sleep(350);
        robot.servoIntake.setPosition(0.5);

        //Opreste shooterul
        robot.motorShooter.setPower(0);

        //Se pozitioneaza pentru wobble si il duce
        //robot.runUsingEncoders(robot.cmToTicks(20),1,3);
        robot.rotateConstantSpeed(-57,0.4,3);
        robot.runUsingEncoders(robot.cmToTicks(85),1,2);

        //lasa primul wobble
        robot.servoWobble.setPosition(0.8);
        robot.servoRidicare.setPosition(1);
        sleep(500);

        //Se intoarce dupa al doilea
        robot.runUsingEncoders(robot.cmToTicks(-25), 1, 2);
        robot.rotateConstantSpeed(-97, 0.4, 3);

        //Se duce dupa al doilea
        robot.runUsingEncodersLongRun(robot.cmToTicks(140),1,7);

        //Prinde al doilea wobble
        robot.servoWobble.setPosition(0);
        sleep(500);
        robot.servoRidicare.setPosition(0.8);

        //Se intoarce catre patrat
        robot.rotateConstantSpeed(160, 0.65, 5);

        //Duce al doilea wobble
        robot.runUsingEncodersLongRun(robot.cmToTicks(140),1,7);


        //Lasa al doilea wobble
        robot.servoWobble.setPosition(0.8);
        robot.servoRidicare.setPosition(1);
        sleep(500);

    }

    private void OneRing(){
        //Porneste motoarele
        robot.servoRidicare.setPosition(0.8);
        robot.motorShooter.setPower(1);

        //Se pozitioneaza la tragere
        robot.runUsingEncodersLongRun(robot.cmToTicks(160), 1, 3);
        robot.rotateConstantSpeed(2, 0.3, 2);

        //Trage primul ring
        robot.servoIntake.setPosition(1);
        sleep(350);
        robot.servoIntake.setPosition(0.5);

        //Se pozitioneaza la power shot-ul din stanga
        robot.ridicareShooter.setPosition(POWER_SHOT_HEIGHT + 0.04);
        robot.rotateConstantSpeed(9, 0.3, 2);
        sleep(100);

        //Trage al doilea ring
        robot.servoIntake.setPosition(1);
        sleep(350);
        robot.servoIntake.setPosition(0.5);
        sleep(150);

        //Se pozitioneaza la tower goal
        robot.ridicareShooter.setPosition(TOWER_GOAL_HEIGHT);
        robot.rotateConstantSpeed(-23, 0.3, 3);

        //Trage al trilea ring
        robot.servoIntake.setPosition(1);
        sleep(350);
        robot.servoIntake.setPosition(0.5);

        //Opreste shooterul
        robot.motorShooter.setPower(0);

        //Se pozitioneaza pentru wobble si il duce
        robot.rotateConstantSpeed(-5,0.4,2);
        robot.runUsingEncoders(robot.cmToTicks(72),1,2);

        //Lasat primul wobble
        robot.servoWobble.setPosition(0.8);
        robot.servoRidicare.setPosition(1);
        sleep(500);

        //Mers dupa al doilea wobble
        robot.runUsingEncoders(robot.cmToTicks(-32),1,2);
        robot.rotateConstantSpeed(-137,0.4,3);
        robot.runUsingEncodersLongRun(robot.cmToTicks(180),1,6);

        //Prinde al doilea wobble
        robot.servoWobble.setPosition(0);
        sleep(500);
        robot.servoRidicare.setPosition(0.8);

        //Se intoarce catre patrat
        robot.rotateConstantSpeed(160, 0.65, 5);

        //Duce al doilea wobble
        robot.runUsingEncodersLongRun(robot.cmToTicks(190),1,7);

        //Lasa al doilea wobble
        robot.servoWobble.setPosition(0.8);
        robot.servoRidicare.setPosition(1);
        sleep(500);

        //Parcare
        robot.runUsingEncoders(robot.cmToTicks(-25),1,3);


    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 0, 0, 0);




        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        public void updateRegions() {
            region1_pointA = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x,
                    REGION1_TOPLEFT_ANCHOR_POINT.y);
            region1_pointB = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                    REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        }

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }

}

