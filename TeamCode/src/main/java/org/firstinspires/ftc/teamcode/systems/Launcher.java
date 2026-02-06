package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Launcher {
    private final OpMode parent;

    public static double[][] launcherValues = { // distance, flywheel rpm (radians per second), angle (in degrees), make sure distance is in ascending order
            {0, 0, 0},
            {},
            {},
            {},
            {},
            {},
            {},
            {}
    };

    private Servo angle;
    private DcMotorEx flywheel;

    public Launcher(OpMode parent) {
        this.parent = parent;
    }

    public void init() {
        this.angle = parent.hardwareMap.get(Servo.class, "launcher_angle");
        this.flywheel = parent.hardwareMap.get(DcMotorEx.class, "flywheel");
    }

    public void launch(int distance) {
        double[] lowVal = {};
        double[] highVal = {};

        for (int i = 0; i < launcherValues.length; i++) {
            double[] current = launcherValues[i];
            if (current[0] > distance) {
                highVal = current;
                lowVal = launcherValues[i - 1];
                break;
            }
        }

        double percent = (distance - lowVal[0]) / (highVal[0] - lowVal[0]);
        double rpm = lowVal[1] + ((highVal[1] - lowVal[1]) * percent);
        double ang = lowVal[2] + ((highVal[2] - lowVal[2]) * percent);

        flywheel.setVelocity(rpm, AngleUnit.RADIANS);
        angle.setPosition(Math.toRadians(ang) / Math.PI);
    }
}
