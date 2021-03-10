/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name="Autonomie finala")
public class EasyOpenCvTesting extends LinearOpMode
{
    OpenCvCamera webcam;
    SkystoneDeterminationPipeline pipeline;
    private RobotMap robot;

    /*
     * The core values which define the location and size of the sample regions
     */
    public static Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(170,140);

    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;

    public static int REGION_WIDTH = 45;
    public static int REGION_HEIGHT = 35;

    public static int FOUR_RING_THRESHOLD = 143;
    public static int ONE_RING_THRESHOLD = 135;

    private final double TOWER_GOAL_HEIGHT = 0.55;

    @Override
    public void runOpMode()
    {
        robot = new RobotMap(hardwareMap, this);
        robot.servoWobble.setPosition(0);
        robot.ridicareShooter.setPosition(TOWER_GOAL_HEIGHT);
        robot.lansareRing.setPosition(0.5);
        robot.zeroPowerBeh();

        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);


        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
//        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
                dashboard.startCameraStream(webcam, 0);
            }
        });

        while (!opModeIsActive() && !isStopRequested()) {
            dashboardTelemetry.addData("Analysis", pipeline.getAnalysis());
            dashboardTelemetry.addData("Position", pipeline.position);
            dashboardTelemetry.update();
        }

        waitForStart();

        if (opModeIsActive())
        {

//            pipeline.updateRegions();
//
//            dashboardTelemetry.addData("Analysis", pipeline.getAnalysis());
//            dashboardTelemetry.addData("Position", pipeline.position);
//            dashboardTelemetry.update();
//
//            // Don't burn CPU cycles busy-looping in this sample
//            sleep(50);

            if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.FOUR) {
                FourRings();
            }
            else if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.ONE) {
                OneRing();
            }
            else if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.NONE) {
                NoRing();
            }
        }
    }

    private void startAndShoot() {
        //Porneste motoarele
//        robot.servoRidicare.setPosition(0.8);
        robot.motorShooter.setPower(1);
        webcam.closeCameraDevice();

        //Se pozitioneaza la tragere
        robot.runUsingEncodersLongRun(robot.cmToTicks(150), 1, 3);


        //Se pozitioneaza la tower goal
        robot.ridicareShooter.setPosition(TOWER_GOAL_HEIGHT);
        robot.rotateConstantSpeed(-9, 0.3, 3);

        robot.lansareRing.setPosition(1);
        sleep(350);
        robot.lansareRing.setPosition(0.5);
        sleep(350);
        robot.lansareRing.setPosition(1);
        sleep(350);
        robot.lansareRing.setPosition(0.5);
        sleep(350);
        robot.lansareRing.setPosition(1);
        sleep(350);
        robot.lansareRing.setPosition(0.5);
        sleep(150);
        //Opreste shooterul
        robot.motorShooter.setPower(0);
    }

    private void FourRings() {


        startAndShoot();
        //Se pozitioneaza pentru wobble si il duce
        robot.rotateConstantSpeed(-12, 0.4, 2);
        robot.runUsingEncodersLongRun(robot.cmToTicks(160), 1, 5);

        //lasa primul wobble
        robot.servoWobble.setPosition(0.8);
//        robot.servoRidicare.setPosition(1);
        sleep(500);

        //Se intoarce dupa al doilea
        robot.runUsingEncodersLongRun(robot.cmToTicks(-100), 1, 5);
        robot.rotateConstantSpeed(-139, 0.4, 3);
        //Se duce dupa al doilea
        robot.motorIntake.setPower(-1);
        robot.runUsingEncodersLongRun(robot.cmToTicks(225),0.75,7);
        sleep(500);
        robot.runUsingEncoders(robot.cmToTicks(-155), 1, 4);
        robot.motorIntake.setPower(0);
        robot.motorShooter.setPower(1);
        robot.rotateConstantSpeed(167, 0.5, 3);
        robot.shoot3Rings();
        robot.motorShooter.setPower(0);
        robot.runUsingEncoders(robot.cmToTicks(35), 1, 3);
//        robot.rotateConstantSpeed(90, 0.4, 2);
//        robot.servoWobble.setPosition(0);
//        sleep(500);
//
//        //Prinde al doilea wobble
//        robot.servoWobble.setPosition(0);
//        sleep(500);
////        robot.servoRidicare.setPosition(0.8);
//
//        //Se intoarce catre patrat
//        robot.rotateConstantSpeed(168, 0.4, 5);
//
//        //Duce al doilea wobble
//        robot.runUsingEncodersLongRun(robot.cmToTicks(262),1,7);
//
//
//        //Lasa al doilea wobble
//        robot.servoWobble.setPosition(0.8);
////        robot.servoRidicare.setPosition(1);
//        sleep(500);
//
//        //Parcheaza
//        robot.runUsingEncodersLongRun(robot.cmToTicks(-85),1,5);
    }

    private void NoRing(){

        startAndShoot();

        //Se pozitioneaza pentru wobble si il duce
        //robot.runUsingEncoders(robot.cmToTicks(20),1,3);
        robot.rotateConstantSpeed(-51,0.4,3);
        robot.runUsingEncoders(robot.cmToTicks(105),1,2);

        //lasa primul wobble
        robot.servoWobble.setPosition(0.8);
//        robot.servoRidicare.setPosition(1);
        sleep(500);

        //Se intoarce dupa al doilea
        robot.runUsingEncoders(robot.cmToTicks(-35), 1, 2);
        robot.rotateConstantSpeed(-101, 0.4, 3);

        //Se duce dupa al doilea
        robot.runUsingEncodersLongRun(robot.cmToTicks(155),1,7);

        //Prinde al doilea wobble
        robot.servoWobble.setPosition(0);
        sleep(500);
//        robot.servoRidicare.setPosition(0.8);

//        //Se intoarce catre patrat
//        robot.rotateConstantSpeed(160, 0.65, 5);
//
//        //Duce al doilea wobble
        robot.runUsingEncodersLongRun(robot.cmToTicks(-157),1,7);
        robot.rotateConstantSpeed(90, 0.65, 5);
        robot.runUsingEncoders(robot.cmToTicks(15),1,3);

//
//
//        //Lasa al doilea wobble
        robot.servoWobble.setPosition(0.8);
        sleep(500);
        robot.runUsingEncoders(robot.cmToTicks(-30),1,3);
////        robot.servoRidicare.setPosition(1);
//        sleep(500);
//
//        robot.runUsingEncoders(-robot.cmToTicks(30), 1, 3);
//        robot.rotateConstantSpeed(60, 0.4, 3);
//        robot.runUsingEncoders(robot.cmToTicks(45), 1, 3);

    }

    private void OneRing(){
        startAndShoot();

        //Se pozitioneaza pentru wobble si il duce
        robot.rotateConstantSpeed(-5,0.4,2);
        robot.runUsingEncoders(robot.cmToTicks(90),1,2);

        //Lasat primul wobble
        robot.servoWobble.setPosition(0.8);
//        robot.servoRidicare.setPosition(1);
        sleep(500);

        //Mers dupa al doilea wobble
        robot.runUsingEncoders(robot.cmToTicks(-95),1,2);
        robot.rotateConstantSpeed(-121,0.4,3);
        robot.motorIntake.setPower(-1);
        robot.runUsingEncodersLongRun(robot.cmToTicks(155),1,6);

        //Prinde al doilea wobble
        robot.servoWobble.setPosition(0);
        sleep(500);
//        robot.servoRidicare.setPosition(0.8);
//
//        //Se intoarce catre patrat
//        robot.rotateConstantSpeed(158, 0.65, 5);

        //Duce al doilea wobble
        robot.motorIntake.setPower(0);
        robot.runUsingEncodersLongRun(robot.cmToTicks(-75),1,7);
        //Se intoarce catre patrat
        robot.rotateConstantSpeed(137, 0.6, 5);
        robot.motorShooter.setPower(1);
        robot.runUsingEncodersLongRun(robot.cmToTicks(70),1,7);
        robot.lansareRing.setPosition(1);
        sleep(400);
        robot.lansareRing.setPosition(0.5);


        //Lasa al doilea wobble
        robot.runUsingEncodersLongRun(robot.cmToTicks(60),1,7);
        robot.servoWobble.setPosition(0.8);
//        robot.servoRidicare.setPosition(1);
        sleep(500);

        //Parcare
        robot.runUsingEncoders(robot.cmToTicks(-30),1,3);


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