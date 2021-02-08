package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class GlobalCoordinateSystemTutorial implements Runnable {

    DcMotor leftEncoder, rightEncoder, middleEncoder;

    boolean isRunning = true;

    double leftEncoderPosition, rightEncoderPosition, middleEncoderPosition;
    double changeInOrientation;
    double OLDLeftEncoderPosition, OLDRightEncoderPosition, OLDMiddleEncoderPosition;

    double globalX, globalY, robotOrientation;

    double encoderWheelDistance;
    double middleEncoderTickOffset;

    int sleepTime;

    File sideWheelSeparationFile = AppUtil.getInstance().getSettingsFile("sideWheelsSeparationFile");
    File middleTickOffsetFile = AppUtil.getInstance().getSettingsFile("middleTickOffsetFile");

    public GlobalCoordinateSystemTutorial(DcMotor leftEncoder, DcMotor rightEncoder, DcMotor middleEncoder, double TICKS_PER_INCH, int threadSleepDelay) {
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.middleEncoder = middleEncoder;
        sleepTime = threadSleepDelay;

        encoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(sideWheelSeparationFile).trim()) * TICKS_PER_INCH;
        middleEncoderTickOffset = Double.parseDouble(ReadWriteFile.readFile(middleTickOffsetFile).trim());
    }

    public void positionUpdate() {
        leftEncoderPosition = leftEncoder.getCurrentPosition();
        rightEncoderPosition = rightEncoder.getCurrentPosition();

        double leftChange = leftEncoderPosition - OLDLeftEncoderPosition;
        double rightChange = rightEncoderPosition - OLDRightEncoderPosition;;

        changeInOrientation = (leftChange - rightChange) / encoderWheelDistance;
        robotOrientation += changeInOrientation;

        middleEncoderPosition = middleEncoder.getCurrentPosition();
        double rawHorizontalChange = middleEncoderPosition - OLDMiddleEncoderPosition;
        double horizontalChange = rawHorizontalChange - (changeInOrientation * middleEncoderTickOffset);

        double sides = (rightChange + leftChange) / 2;
        double frontBack =  horizontalChange;

        globalX = sides * Math.sin(robotOrientation) + frontBack * Math.cos(robotOrientation);
        globalY = sides * Math.cos(robotOrientation) + frontBack * Math.sin(robotOrientation);

        OLDLeftEncoderPosition = leftEncoderPosition;
        OLDRightEncoderPosition = rightEncoderPosition;
        OLDMiddleEncoderPosition = middleEncoderPosition;
    }

    public double returnXCoordinate() {
        return globalX;
    }

    public double returnYCoordinate() {
        return globalY;
    }

    public double returnOrientation() {
        return Math.toDegrees(robotOrientation) % 360;
    }

    public void stop() {
        isRunning = false;
    }

    @Override
    public void run() {
        while (isRunning) {
            positionUpdate();
        }

        try {
            Thread.sleep(sleepTime);
        }
        catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
