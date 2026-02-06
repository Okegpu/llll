package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.Paths;
import org.firstinspires.ftc.teamcode.systems.Turret;

@Autonomous(name = "Auto Red")
public class AutoRed extends OpMode {
    private Turret turret;
    private Follower follower;
    private Paths paths;

    @Override
    public void init() {
        this.turret = new Turret(this);
        this.follower = Constants.createFollower(this.hardwareMap);
        this.paths = new Paths(follower);

        turret.init();
    }

    @Override
    public void start() {
        follower.followPath(paths.red_1);
        follower.followPath(paths.red_2);
    }

    @Override
    public void loop() {
        turret.loop(follower);
    }
}
