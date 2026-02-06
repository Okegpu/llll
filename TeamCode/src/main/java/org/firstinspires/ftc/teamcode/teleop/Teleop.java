package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.systems.Launcher;
import org.firstinspires.ftc.teamcode.systems.Turret;

@TeleOp(name = "TeleOp")
public class Teleop extends OpMode {
    private Follower follower;
    private Launcher launcher;
    private Turret turret;

    @Override
    public void init() {
        this.follower = Constants.createFollower(this.hardwareMap);
        this.turret = new Turret(this);
        this.launcher = new Launcher(this);

        turret.init();
        launcher.init();
    }

    @Override
    public void loop() {
        turret.loop(follower);
        follower.setTeleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad2.right_stick_x);
        double dx = 144 - follower.getPose().getX();
        double dy = 144 - follower.getPose().getY();
        int dist = (int) Math.hypot(dx, dy);
        launcher.launch(dist);
    }
}
