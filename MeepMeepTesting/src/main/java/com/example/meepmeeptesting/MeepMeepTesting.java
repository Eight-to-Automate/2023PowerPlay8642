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

        Pose2d drop1 = new Pose2d(-24,-6.5,Math.toRadians(90));
        Pose2d cyclePose = new Pose2d(-35.75, -23, Math.toRadians(90));
        Pose2d stackPos = new Pose2d(-63, -12, Math.toRadians(180));
        Vector2d preturn = new Vector2d(-35.7, -12);
        Pose2d backpose = new Pose2d(-35.7, -12);

        Pose2d startPos1 = new Pose2d(-35.7,-62.7, Math.toRadians(90));//Splinedroptest4
        Pose2d conePush = new Pose2d(-35.7,-7.5, Math.toRadians(90));
        Pose2d highJunctionHeading = new Pose2d(-24, -6, Math.toRadians(90));
        Pose2d highJunctionHeading2 = new Pose2d(-28, -4, Math.toRadians(95));


        /*
        square
        .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
         */

        /*
        old program
        .lineTo(forward2) // added Max accel * 0.8 for championship Drive speed was 0.85
                                .lineTo(highJunction)
                                .forward(7)
                                .back(6.5)
                                .strafeLeft(12)
                                .turn(Math.toRadians(90))
                                .lineTo(stack)
                                .back(1)
                                .back(25)
                                .turn(Math.toRadians(-90))
                                .back(3)
                                .lineTo(highJunction)
                                .forward(6)

         */

        /*
        first approach time save
                        .splineToLinearHeading(cyclePose, Math.toRadians(90))
                        .splineToLinearHeading(highJunctionHeading, Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.7, DriveConstants.MAX_ANG_VEL*0.8, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL* 0.8))
         */


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(0.8 * 50.18793796530113, 0.67 * 50.18793796530113, 3.3, 3.0, 13.9)
                .setDimensions(16,16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(stackPos)
//                                .setReversed(true)
//                                .splineToSplineHeading(cyclePose, Math.toRadians(180))
//                                .splineToSplineHeading(stackPos, Math.toRadians(180))

//******************************************************************************************************

//                               .splineToSplineHeading(new Pose2d(-43, -12, Math.toRadians(180)), Math.toRadians(180))
//                              .splineToSplineHeading(new Pose2d(-35.7 + 7.5, -12 + 7.5, Math.toRadians(45)), Math.toRadians(60))

//                                .splineToSplineHeading(new Pose2d(-12, -12, Math.toRadians(180)), Math.toRadians(180))
//                                .splineToSplineHeading(new Pose2d(-19.8, -12, Math.toRadians(110)), Math.toRadians(180))

//___________________________________________start up to first cone
/*
                                .splineToLinearHeading(conePush, Math.toRadians(90))
                                        .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-24,-7.5-2,Math.toRadians(90)), Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(-24,-7.5-0.8,Math.toRadians(90)), Math.toRadians(90))
                                                .setReversed(true)
 */
//__________________________________________________
                                //.splineToSplineHeading(cyclePose, Math.toRadians(180))
                                //.splineToLinearHeading(stackPos, Math.toRadians(180))

//                                .splineToLinearHeading(conePush, Math.toRadians(90))
//                                .setReversed(true)
//                                .splineToLinearHeading(new Pose2d(-35.7+2,-7.5 - 5, Math.toRadians(90)), Math.toRadians(90))
//                                .splineToLinearHeading(drop1, Math.toRadians(120))
//                                .setReversed(false)
                               // .back(1.5)
                                .setReversed(true)
                                .lineTo(new Vector2d(-40, -12 ))
                                .splineToSplineHeading(new Pose2d(-30, -12, Math.toRadians(70)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(-26, -6, Math.toRadians(70)), Math.toRadians(70))

                                .setReversed(false)
                                .lineTo(new Vector2d(-30, -12))
                                .splineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(90)), Math.toRadians(180))

                                /*
                                .splineToSplineHeading(new Pose2d(-39, -12, Math.toRadians(180)), Math.toRadians(180))
                                .splineToLinearHeading(stackPos, Math.toRadians(180))*/



                              //  .splineToSplineHeading(cyclePose, Math.toRadians(270))
                                //.splineToLinearHeading(new Pose2d(-30,-5, Math.toRadians(45)), Math.toRadians(0))
                             //   .splineToLinearHeading(highJunctionHeading, Math.toRadians(0))
                                //.splineToLinearHeading(highJunctionHeading2, Math.toRadians(0))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}