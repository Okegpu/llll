package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "DriveForward1Sec", group = "Test")
public class secondLucaAutonomous extends LinearOpMode {

    // ================= DRIVE =================
    private DcMotor fLMotor, fRMotor, bLMotor, bRMotor;

    // ================= SHOOTER =================
    private DcMotor shootMotor;
    private DcMotor pushMotor;

    // ================= SENSORS =================
    private Limelight3A limelight;
    private IMU imu;

    // ================= AIM CONSTANTS =================
    private static final double AIM_KP  = 0.025;
    private static final double AIM_MIN = 0.08;
    private static final double AIM_MAX = 0.35;
    private static final double AIM_TOL = 0.7;

    private static final double CAM_TX_OFFSET = 9.0;
    private static final double OFFSET_SLOPE  = 0.25;

    @Override
    public void runOpMode() {

        // ================= HARDWARE =================
        fLMotor = hardwareMap.get(DcMotor.class, "front_left");
        fRMotor = hardwareMap.get(DcMotor.class, "front_right");
        bLMotor = hardwareMap.get(DcMotor.class, "back_left");
        bRMotor = hardwareMap.get(DcMotor.class, "back_right");

        shootMotor = hardwareMap.get(DcMotor.class, "top_motor");
        pushMotor  = hardwareMap.get(DcMotor.class, "p_motor");

        fLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bLMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        fLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ================= LIMELIGHT =================
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0); // AprilTag pipeline
            limelight.start();
        } catch (Exception e) {
            limelight = null;
        }

        // ================= IMU =================
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ================= DRIVE FORWARD =================

        //driveForward(1.0);
        //sleep(1000);
        //stopDrive();

        // ================= AUTO AIM =================
        autoAim();

        // ================= SHOOT =================
        shootMotor.setPower(1.0);   // spin up
        sleep(1200);

        pushMotor.setPower(-1.0);   // fire
        sleep(700);

        pushMotor.setPower(0.0);
        shootMotor.setPower(0.0);
    }

    // ================= AUTO AIM =================
    private void autoAim() {

        if (limelight == null) return;

        long timeout = System.currentTimeMillis() + 2500;

        while (opModeIsActive() && System.currentTimeMillis() < timeout) {

            LLResult r = limelight.getLatestResult();
            if (r == null || !r.isValid()) continue;

            double tx = r.getTx();
            double ty = r.getTy();

            double distanceFactor = Math.max(-15, Math.min(0, ty));
            double dynamicOffset = CAM_TX_OFFSET + (distanceFactor * OFFSET_SLOPE);
            double correctedTx = tx + dynamicOffset;

            double targetHeading = normalize(getHeading() + correctedTx);

            while (opModeIsActive()) {
                double error = normalize(targetHeading - getHeading());
                double turn = error * AIM_KP;

                if (Math.abs(turn) < AIM_MIN)
                    turn = Math.copySign(AIM_MIN, turn);

                turn = Math.max(-AIM_MAX, Math.min(AIM_MAX, turn));
                mecanum(0, 0, turn);

                if (Math.abs(error) < AIM_TOL) break;
            }

            mecanum(0, 0, 0);
            break;
        }
    }

    // ================= DRIVE HELPERS =================
    private void driveForward(double power) {
        fLMotor.setPower(-power);
        bLMotor.setPower(-power);
        fRMotor.setPower(-power);
        bRMotor.setPower(-power);
    }

    private void stopDrive() {
        fLMotor.setPower(0);
        bLMotor.setPower(0);
        fRMotor.setPower(0);
        bRMotor.setPower(0);
    }

    private void mecanum(double x, double y, double rx) {
        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;

        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr),
                                Math.max(Math.abs(bl), Math.abs(br)))));

        fLMotor.setPower(fl / max);
        fRMotor.setPower(fr / max);
        bLMotor.setPower(bl / max);
        bRMotor.setPower(br / max);
    }

    private double getHeading() {
        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        return ypr.getYaw(AngleUnit.DEGREES);
    }

    private double normalize(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }
}
