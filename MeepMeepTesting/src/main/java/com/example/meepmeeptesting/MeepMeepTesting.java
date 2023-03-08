package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        // positions for localization
        Pose2d startPos1 = new Pose2d(-35.7,-62.7, Math.toRadians(90));
        Pose2d conePush = new Pose2d(-35.7,-7.5, Math.toRadians(90));
        Pose2d drop1 = new Pose2d(-25,-6.5,Math.toRadians(90));//  was -6
        Pose2d cyclePose = new Pose2d(-35.75, -14, Math.toRadians(180));
        Pose2d stackPos = new Pose2d(-62.5, -12, Math.toRadians(180));

        Pose2d pushToDrop1 = new Pose2d(-25,-9.5,Math.toRadians(90));

        Pose2d scorePos = new Pose2d(-26,-6, Math.toRadians(70));
        Pose2d scoreBack = new Pose2d(-26 - 4 * Math.cos(Math.toRadians(70)), -6 - 4 * Math.sin(Math.toRadians(70)), Math.toRadians(70));

        // our auto
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(0.8 * 50.18793796530113, 0.67 * 50.18793796530113, 3.3, 3.0, 13.9)
                .setDimensions(16,16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPos1)
//                                .back(6)
//                                .setReversed(true)
//                                .splineToLinearHeading(new Pose2d(-35, -12, Math.toRadians(90)), Math.toRadians(180)) //zone 2



                                    //.back(3)
                                //.setReversed(true)
                                //.splineToLinearHeading(new Pose2d(-57, -13, Math.toRadians(90)), Math.toRadians(210)) //zone 1


//                                .back(3)
//                                .setReversed(true)
//                                .splineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(90)), Math.toRadians(90)) //zone 3

//                                .splineToLinearHeading(conePush, Math.toRadians(90))//first forward movement, used to have 0.8 constraints
//                                //.setReversed(true)
//
//                                .splineToLinearHeading(drop1, Math.toRadians(90))  //-7.8
//                                .setReversed(false)


                                //.back(4)
                                //.lineToLinearHeading(scoreBack)
                                //.splineToSplineHeading(new Pose2d(-35, -12, Math.toRadians(180)), Math.toRadians(180))
                                //.splineToSplineHeading(stackPos, Math.toRadians(180))

                                .splineToLinearHeading(conePush, Math.toRadians(90))
                                //.setReversed(true)

                                .splineToLinearHeading(pushToDrop1, Math.toRadians(90))
                                .splineToLinearHeading(drop1, Math.toRadians(90))

                                .build()
                   );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}