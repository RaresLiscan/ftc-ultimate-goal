package org.firstinspires.ftc.teamcode.pid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotMap;

@Autonomous(name="PID Calibration")
public class PIDMovementAuto extends LinearOpMode {

    RobotMap robot = null;

    FtcDashboard dashboard;

    public static double TARGET_POS = 100;
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotMap(hardwareMap, this);

        DcMotor[] motors = {robot.stangaFata, robot.stangaSpate, robot.dreaptaSpate, robot.dreaptaFata};
        PIDMovement movement = new PIDMovement(motors, pidCoefficients);
        dashboard = FtcDashboard.getInstance();

        waitForStart();

        movement.moveMotor(TARGET_POS);

    }
}
