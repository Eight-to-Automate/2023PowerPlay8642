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
        //Vector2d forward2 = new Vector2d(-35.7, -7.5);// was -35.75. -9.5
        Pose2d forward2 = new Pose2d(-35.7, -7.5, Math.toRadians(90));
        Vector2d highJunction = new Vector2d(-23.5, -14.5);     //was -23.5, -14.5 meet 3
        Pose2d highJunctionH = new Pose2d(-3.75, -15.5, Math.toRadians(90));
        //Vector2d getHighJunctionClose = new Vector2d(4.5, -25.1875);
        Vector2d stack = new Vector2d(-62.75, -10.5);  // was 62.75, 11.75
        Pose2d stackh = new Pose2d(-63, -12, Math.toRadians(180));

        Pose2d cyclePose = new Pose2d(-35.75, -24, Math.toRadians(90));
        //Pose2d cyclePose = new Pose2d(-35.75, -48, Math.toRadians(90));
        Pose2d prescore = new Pose2d(-35.75, -16, Math.toRadians(90));
        Pose2d highJunctionHeading = new Pose2d(-24, -6, Math.toRadians(90));


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
                        drive.trajectorySequenceBuilder(startPos1)
                                .splineToLinearHeading(forward2, Math.toRadians(90))
                                .splineToLinearHeading(prescore, Math.toRadians(90))
                                .splineToLinearHeading(highJunctionHeading, Math.toRadians(90))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}