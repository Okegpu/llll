package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "ChassisDrive", group = "Drive")
public class TeleopRed extends OpMode {

    // ================= DRIVE =================
    private DcMotor fLMotor, fRMotor, bLMotor, bRMotor;

    // ================= MECHANISMS =================
    private DcMotor shootMotor;
    private DcMotor inMotor;
    private DcMotor pushMotor;
    private CRServo spinServo;

    // ================= SENSORS =================
    private Limelight3A limelight;
    private IMU imu;

    // ================= SHOOT MODES =================
    private enum ShotMode { OFF, CLOSE, MID, FAR }
    private ShotMode shotMode = ShotMode.OFF;

    // ================= TOGGLES =================
    private boolean feedOn = false;
    private boolean intakeOn = false;
    private boolean rtLast = false;
    private boolean ltLast = false;

    // ================= AUTO AIM =================
    private boolean autoAim = false;
    private double desiredHeading = 0.0;

    private double llTx = 0.0;
    private double llTy = 0.0;
    private boolean llValid = false;

    // ================= AIM TUNING =================
    private static final double AIM_KP  = 0.025;
    private static final double AIM_MIN = 0.08;
    private static final double AIM_MAX = 0.35;
    private static final double AIM_TOL = 0.7;

    // Camera offset (camera is right of shooter)
    private static final double CAM_TX_OFFSET = 9.0   ;
    private static final double OFFSET_SLOPE  = 0.25;

    @Override
    public void init() {

        fLMotor = hardwareMap.get(DcMotor.class, "front_left");
        fRMotor = hardwareMap.get(DcMotor.class, "front_right");
        bLMotor = hardwareMap.get(DcMotor.class, "back_left");
        bRMotor = hardwareMap.get(DcMotor.class, "back_right");

        shootMotor = hardwareMap.get(DcMotor.class, "top_motor");
        inMotor    = hardwareMap.get(DcMotor.class, "in_motor");
        pushMotor  = hardwareMap.get(DcMotor.class, "p_motor");
        spinServo  = hardwareMap.get(CRServo.class, "spin_servo");

        fLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bLMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0); // AprilTag pipeline
            limelight.start();
        } catch (Exception e) {
            limelight = null;
        }

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));
    }

    @Override
    public void loop() {

        // ================= DRIVE =================
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        if (!autoAim) {
            mecanum(x, y, rx);
        }

        // ================= MANUAL SHOOT MODE =================
        if (gamepad1.dpad_left)  shotMode = ShotMode.CLOSE;
        if (gamepad1.dpad_right) shotMode = ShotMode.MID;
        if (gamepad1.dpad_up)    shotMode = ShotMode.FAR;
        if (gamepad1.dpad_down)  shotMode = ShotMode.OFF;

        switch (shotMode) {
            case CLOSE: shootMotor.setPower(0.9); break;
            case MID:
            case FAR:   shootMotor.setPower(1.0); break;
            case OFF:   shootMotor.setPower(0.0); break;
        }

        // ================= PUSH MOTOR TOGGLE =================
        boolean rtNow = gamepad1.right_trigger > 0.4;
        if (rtNow && !rtLast) feedOn = !feedOn;
        rtLast = rtNow;

        if (shotMode == ShotMode.OFF) feedOn = false;
        pushMotor.setPower(feedOn ? -1.0 : 0.0);

        // ================= INTAKE TOGGLE =================
        boolean ltNow = gamepad2.left_trigger > 0.4;
        if (ltNow && !ltLast) intakeOn = !intakeOn;
        ltLast = ltNow;

        if (intakeOn) {
            inMotor.setPower(-1.0);
            spinServo.setPower(1.0);
        } else {
            inMotor.setPower(0.0);
            spinServo.setPower(0.0);
        }



        // ================= LIMELIGHT READ =================
        llValid = false;
        if (limelight != null) {
            LLResult r = limelight.getLatestResult();
            if (r != null && r.isValid()) {
                llValid = true;
                llTx = r.getTx();
                llTy = r.getTy();
            }
        }

        // ================= AUTO AIM START =================
        if (gamepad1.x && llValid) {
            double distanceFactor = Math.max(-15, Math.min(0, llTy));
            double dynamicOffset = CAM_TX_OFFSET + (distanceFactor * OFFSET_SLOPE);
            double correctedTx = llTx + dynamicOffset;

            desiredHeading = normalize(getHeading() + correctedTx);
            autoAim = true;
        }

        if (gamepad1.a) autoAim = false;

        // ================= AUTO AIM LOOP =================
        if (autoAim) {
            double error = normalize(desiredHeading - getHeading());
            double turn = error * AIM_KP;

            if (Math.abs(error) < 6) turn *= 0.4;

            if (Math.abs(turn) < AIM_MIN)
                turn = Math.copySign(AIM_MIN, turn);

            turn = Math.max(-AIM_MAX, Math.min(AIM_MAX, turn));
            mecanum(0, 0, turn);

            if (Math.abs(error) < AIM_TOL) {
                mecanum(0, 0, 0);
                autoAim = false;
            }
        }

        telemetry.addData("tx", llTx);
        telemetry.addData("ty", llTy);
        telemetry.addData("ShotMode", shotMode);
        telemetry.addData("AutoAim", autoAim);
        telemetry.update();
    }

    // ================= HELPERS =================
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
