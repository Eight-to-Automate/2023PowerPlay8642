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
        Pose2d startPos1 = new Pose2d(-35.7,-62.7 - 24, Math.toRadians(180-90));
        Pose2d conePush = new Pose2d(-35.7,-7.5, Math.toRadians(180-90));
        Pose2d drop1 = new Pose2d(-24.5,-5.5,Math.toRadians(180-90));//  was -6

        //    Pose2d drop2 = new Pose2d(-4.9, -20.11, Math.toRadians(-45));
        Pose2d drop2 = new Pose2d(-1, 18.5, Math.toRadians(-90));
        Pose2d stall = new Pose2d(-13,7,Math.toRadians(-90));

        Pose2d LeftTurnTransform = new Pose2d(24,-12.5, Math.toRadians(180));
        Pose2d TransformPosition = new Pose2d(-12, -12.5, Math.toRadians(180));
//    Vector2d TRANSFORMER = new Vector2d(-38.5, 0);

        Pose2d TRANSFORMER = new Pose2d(39, -1.5-1.8, Math.toRadians(0));
        Vector2d TRANSFORMERVECTOR = new Vector2d(39, -1.5-1.8);

        Pose2d zone1 = new Pose2d(12,-13, Math.toRadians(180+90));
        Pose2d zone2 = new Pose2d(-12,-13, Math.toRadians(180+90));
        Pose2d zone3 = new Pose2d(-36,-13, Math.toRadians(180+90));

        Pose2d laterDropsFirstHalf = new Pose2d(30, -12, Math.toRadians(110));
        Pose2d pushToDrop1 = new Pose2d(-24,-7.5-2,Math.toRadians(180-90));

        Pose2d cyclePose = new Pose2d(-35.75 + 24, -14, Math.toRadians(0));
        Pose2d transformerPos = new Pose2d(-62.5 + 12, 12, Math.toRadians(0));

        Pose2d CMLeftTurnTransform = new Pose2d(24,-12.5, Math.toRadians(180));
        Pose2d CMTransformPosition = new Pose2d(-12, -12.5, Math.toRadians(180));

        Vector2d laterScoresFirstLineTo = new Vector2d(40, -12 );
        //    Pose2d laterDropsSecondHalf = new Pose2d(-26, -6, Math.toRadians(70));
        Pose2d laterDropsSecondHalf = new Pose2d(26, -6.5, Math.toRadians(110));  //for testing 2/19/23
        Vector2d stack2LineTo1 = new Vector2d(30, -12);
        Pose2d stack2FirstSpline = new Pose2d(39, -12, Math.toRadians(0));
        // our auto
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(0.8 * 50.18793796530113, 0.67 * 50.18793796530113, 3.3, 3.0, 13.9)
                .setDimensions(16,16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(TRANSFORMER)
                                //.setReversed(true)
                                .lineTo(new Vector2d(30-1,0-1.5-1.8))
                                .splineToSplineHeading(new Pose2d(12-1.5,-12,Math.toRadians(-90)),Math.toRadians(180))
                                .lineTo(new Vector2d(-24-1.5, -12))
                                .splineToLinearHeading(new Pose2d(-35-1.5,-24,Math.toRadians(-90)),Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(-35-1.5,-58-24,Math.toRadians(-90)),Math.toRadians(-90))
                                .build()
                   );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}