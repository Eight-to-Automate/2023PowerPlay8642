package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        // positions for localization
        Pose2d startPos1 = new Pose2d(-35.7,-62.7, Math.toRadians(90));
        Pose2d conePush = new Pose2d(-35.7,-7.5, Math.toRadians(90));

        Pose2d pushToDrop1 = new Pose2d(-26+2,-7.5-4,Math.toRadians(90));
        Pose2d drop1 = new Pose2d(-24,-5.5,Math.toRadians(90));

        Pose2d stackPos = new Pose2d(-62.5, -12, Math.toRadians(180));


        Pose2d scorePos = new Pose2d(-26,-6.5, Math.toRadians(70));
        Pose2d scoreBack = new Pose2d(-26 - 4 * Math.cos(Math.toRadians(70)), -6 - 4 * Math.sin(Math.toRadians(70)), Math.toRadians(70));

        Pose2d prePark = new Pose2d(-28.55, -13, Math.toRadians(70));
        Pose2d zone1 = new Pose2d(-58, -14, Math.toRadians(90));
        Pose2d zone2 = new Pose2d(-35.7, -14, Math.toRadians(90));
        Pose2d zone3 = new Pose2d(-12, -14, Math.toRadians(90));
        // our auto
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(0.8 * 50.18793796530113, 0.67 * 50.18793796530113, 3.3, 3.0, 13.9)
                .setDimensions(16,16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPos1)
                                //score1
                                .splineToLinearHeading(conePush, Math.toRadians(90))
                                .setReversed(true)
                                .splineToLinearHeading(pushToDrop1, Math.toRadians(90))
                                .splineToLinearHeading(drop1, Math.toRadians(90))
//tostack1.
                                //.back(1.5)
                                .waitSeconds(0.1)
                                //.lineToLinearHeading(new Pose2d(-24,-5.5-1.5,Math.toRadians(91)))
                                .splineToConstantHeading(new Vector2d(-30, -12), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-39, -12, Math.toRadians(180)), Math.toRadians(180))
                                .splineToSplineHeading(stackPos, Math.toRadians(180))

                                .lineTo(new Vector2d(-40, -12 ))
                                .splineToSplineHeading(new Pose2d(-30, -12, Math.toRadians(70)), Math.toRadians(0))
                                .splineToLinearHeading(scorePos, Math.toRadians(70))
                                .waitSeconds(0.1)
//                                .splineToSplineHeading(new Pose2d(-36, -14, Math.toRadians(180)), Math.toRadians(180))
//                                .splineToSplineHeading(stackPos, Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-30, -12), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-39, -12, Math.toRadians(180)), Math.toRadians(180))
                                .splineToSplineHeading(stackPos, Math.toRadians(180))
                                .build()
                   );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}