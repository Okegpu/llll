package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {
     public PathChain red_1;
     public PathChain red_2;

     public Paths(Follower follower) {
         red_1 = follower.pathBuilder().addPath(
                 new BezierLine(
                         new Pose(56.000, 8.000),
                         new Pose(46.382, 35.725)
                 ))
                 .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                 .build();

         red_2 = follower.pathBuilder().addPath(
                 new BezierLine(
                         new Pose(46.382, 35.725),
                         new Pose(18.000, 35.718)
                 ))
                 .setTangentHeadingInterpolation()
                 .build();
     }
}
