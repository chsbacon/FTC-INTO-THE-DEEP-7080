package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.*;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AutoRightStart {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)

                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(11, -62.5, Math.toRadians(-90.00)))
                        .addDisplacementMarker(() -> {
                            //Arm raise
                            //Forearm to vert
                        })
                        .setReversed(true)
                        .splineTo(new Vector2d(0, -33), Math.toRadians(90.00))
                        .lineTo(new Vector2d(0, -37.40))

                        .splineToConstantHeading(new Vector2d(28.31, -35.21), Math.toRadians(11.55))
                        .splineToConstantHeading(new Vector2d(36.36, -15.28), Math.toRadians(75.62))
                        .splineToConstantHeading(new Vector2d(46.5, -8), Math.toRadians(90))

                        .lineTo(new Vector2d(46.6, -55))
                        .back(5)
//                        // GRAB sample
                        .splineToConstantHeading(new Vector2d(-0, -33), Math.toRadians(89.01))
//                        //Place sample
                        .forward(5)
//
                        .splineToConstantHeading(new Vector2d(21.84, -40.10), Math.toRadians(-11.50))
                        .splineToConstantHeading(new Vector2d(35.93, -34.63), Math.toRadians(39.29))
                        .splineToConstantHeading(new Vector2d(46.85, -13.08), Math.toRadians(47.12))
                        .splineToConstantHeading(new Vector2d(56, -8), Math.toRadians(90))
                        .lineTo(new Vector2d(56, -55))
                        .back(5)
                        // pick up sample
                        .splineToConstantHeading(new Vector2d(0, -33), Math.toRadians(90))
                        //place sample
                        .setReversed(false)
                        .splineTo(new Vector2d(60, -60), Math.toRadians(-90))








                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}