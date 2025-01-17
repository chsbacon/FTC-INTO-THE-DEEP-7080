package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AutoLeftStart {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)

                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-24, -61, Math.toRadians(90)))



                        .splineTo(new Vector2d(-53, -53), Math.toRadians(225)) //bucket
                        .splineToLinearHeading(new Pose2d( -48.75, -53,Math.toRadians(-90)), Math.toRadians(-90)) //Pickup
                        //run action here
                        .splineToLinearHeading(new Pose2d(-53, -53, Math.toRadians(225)), Math.toRadians(225)) //bucket


                        .splineToLinearHeading(new Pose2d( -59.5, -50,Math.toRadians(-90)), Math.toRadians(-90)) //

                        .setReversed(true)
                        .splineTo(new Vector2d(-38.95, -25.15), Math.toRadians(75.96))
                        .splineTo(new Vector2d(-24.86, 0.29), Math.toRadians(0)) //park
                        //run park action here

                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}