package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "MotorTest", group = "Test")
public class MotorTest extends OpMode {

    private DcMotor bLMotor, bRMotor, fLMotor, fRMotor;
    private DcMotor inMotor, pushMotor, shootMotor;
    private CRServo spinServo;

    // Shooter toggle
    private boolean shooterOn = false;
    private boolean yPressedLast = false;

    // Jam clear timing
    private long jamClearEndTime = 0;
    private boolean clearingJam = false;

    @Override
    public void init() {

        bLMotor = hardwareMap.get(DcMotor.class, "back_left");
        bRMotor = hardwareMap.get(DcMotor.class, "back_right");
        fLMotor = hardwareMap.get(DcMotor.class, "front_left");
        fRMotor = hardwareMap.get(DcMotor.class, "front_right");

        shootMotor = hardwareMap.get(DcMotor.class, "top_motor");
        inMotor = hardwareMap.get(DcMotor.class, "in_motor");
        pushMotor = hardwareMap.get(DcMotor.class, "p_motor");
        spinServo = hardwareMap.get(CRServo.class, "spin_servo");

        // Motor directions
        fLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bLMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        fRMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        bRMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        // Brake mode
        fLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servo Directions
        spinServo.setDirection(CRServo.Direction.FORWARD);
    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            bLMotor.setPower(0.5);
        } else {
            bLMotor.setPower(0);
        }

        if (gamepad1.a) {
            bRMotor.setPower(0.5);
        } else {
            bRMotor.setPower(0);
        }

        if (gamepad1.y) {
            fLMotor.setPower(0.5);
        } else {
            fLMotor.setPower(0);
        }

        if (gamepad1.b) {
            fRMotor.setPower(0.5);
        } else {
            fRMotor.setPower(0);
        }


        if (gamepad1.dpad_up) {
            shootMotor.setPower(0.5);
        } else {
            shootMotor.setPower(0);
        }

        if (gamepad1.dpad_left) {
            inMotor.setPower(0.5);
        } else {
            inMotor.setPower(0);
        }

        if (gamepad1.dpad_right) {
            pushMotor.setPower(0.5);
        } else {
            pushMotor.setPower(0);
        }

        telemetry.addData("bLMotor", gamepad1.x);
        telemetry.addData("bRMotor", gamepad1.a);
        telemetry.addData("fLMotor", gamepad1.y);
        telemetry.addData("fRMotor", gamepad1.b);
        telemetry.addData("shootMotor", gamepad1.dpad_up);
        telemetry.addData("inMotor", gamepad1.dpad_left);
        telemetry.addData("pushMotor", gamepad1.dpad_right);
    }
}