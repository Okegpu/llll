package org.firstinspires.ftc.teamcode.systems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;

public class Turret {
    private final OpMode parent;
    private DcMotorEx turret;
    private Limelight3A limelight;

    private static final double GOAL_X = 144;
    private static final double GOAL_Y = 144;

    private static final double MOTOR_OUT_TICKS_PER_REV = 383.6;
    private static final double EXTRA_GEAR_RATIO = 100.0 / 54.0;
    private static final double TICKS_PER_REV = MOTOR_OUT_TICKS_PER_REV * EXTRA_GEAR_RATIO;
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_RAD = (TICKS_PER_REV * GEAR_RATIO) / (2 * Math.PI);

    private static final double MIN_ANGLE = Math.toRadians(-90);
    private static final double MAX_ANGLE = Math.toRadians(90);

    private static final double kP = 0.005;
    private static final double kI = 0.0;
    private static final double kD = 0.0002;

    private double integral = 0;
    private double lastError = 0;

    public Turret(OpMode parent) {
        this.parent = parent;
    }

    public void init() {
        this.turret = parent.hardwareMap.get(DcMotorEx.class, "turret");
        this.limelight = parent.hardwareMap.get(Limelight3A.class, "limelight");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void loop(Follower follower) {
        LLResult result = limelight.getLatestResult();
        Pose3D botpose = result.getBotpose();
        if (botpose == null) return;

        Position position = botpose.getPosition();
        follower.setPose(new Pose(position.x, position.y));

        double robotX = position.x;
        double robotY = position.y;

        double dx = GOAL_X - robotX;
        double dy = GOAL_Y - robotY;

        double targetAngle = Math.atan2(dy, dx) - follower.getHeading();

        targetAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, targetAngle));

        // pid type shit
        int currentTicks = turret.getCurrentPosition();
        double currentAngle = currentTicks / TICKS_PER_RAD;

        double error = targetAngle - currentAngle;
        integral += error;
        double derivative = error - lastError;
        lastError = error;

        double output = (kP * error) + (kI * integral) + (kD * derivative);
        turret.setPower(output);
    }
}
