package com.example.meepmeep_0_1_6;


import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeep_RedFarMultipleCyclesActions {
    static Pose2d startPose;
    static Pose2d stackPose;
    static Pose2d deliverToFloorPose;
    static Pose2d deliverToBoardPose;
    static Action FloorTraj;
    static Action DriveToStack;
    static Action BoardTraj2;
    static RoadRunnerBotEntity myBot;
    static int autoPosition = 1;
    static VelConstraint speedUpVelocityConstraint;
    static AccelConstraint speedUpAccelerationConstraint;
    static VelConstraint slowDownVelocityConstraint;
    static AccelConstraint slowDownAccelerationConstraint;
    static double stackY = -12.0;
    static double stackX = -59.0;
	
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        startPose = new Pose2d(-36,-62.5,Math.toRadians(90));
        stackPose = new Pose2d(stackX, stackY, Math.toRadians(180)); //-54.5,-11.5

        // Define some custom constraints to use when wanting to go faster than defaults
        speedUpVelocityConstraint = new TranslationalVelConstraint(75.0);
        speedUpAccelerationConstraint = new ProfileAccelConstraint(-40.0, 60.0);
        slowDownVelocityConstraint = new TranslationalVelConstraint(5); 
        slowDownAccelerationConstraint = new ProfileAccelConstraint(-20, 50);

        // Define the standard constraints to use for this robot
        myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.PI*.6, Math.PI*.8, 18.959)
                .setDimensions(18, 15)
                .build();

        // ******************************************
        /* Specify which Position will be run */
        // ******************************************
        autoPosition = 3;

        // Build up the floor delivery trajectory
        RedLeftPurplePixelDecision();

        // Create the floor to Stack trajectory
        DriveToStack = myBot.getDrive().actionBuilder(deliverToFloorPose)
                .splineToLinearHeading(new Pose2d(stackX+5, stackY, Math.toRadians(180)), Math.toRadians(180), null, slowDownAccelerationConstraint)
                .strafeToLinearHeading(new Vector2d(stackX, stackY), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(stackX-2, stackY), Math.toRadians(180), slowDownVelocityConstraint)
                .build();

        // Build up the Stack to Board Trajectory
        RedBoardDecision();

        // Create the backup trajectory
        Action Backup1 = myBot.getDrive().actionBuilder(deliverToBoardPose)
                .lineToX(46)
                .build();

        // Build up the Board back to Stack Trajectory
        Action DriveBackToStack = myBot.getDrive().actionBuilder(new Pose2d(46, deliverToBoardPose.position.y, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(30, stackY), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(12, stackY), Math.toRadians(180), speedUpVelocityConstraint)
                .strafeToLinearHeading(new Vector2d(-12, stackY), Math.toRadians(180), speedUpVelocityConstraint)
                .strafeToLinearHeading(new Vector2d(-36, stackY), Math.toRadians(180), speedUpVelocityConstraint)
                .strafeToLinearHeading(new Vector2d(-52, stackY), Math.toRadians(180), speedUpVelocityConstraint)
                .strafeToLinearHeading(new Vector2d(stackX+5, stackY), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(stackX, stackY), Math.toRadians(180))
                .build();

        // Build up the Stack to Board Position 1 Trajectory
        Action BoardTrajFinal = myBot.getDrive().actionBuilder(stackPose)
                .strafeToLinearHeading(new Vector2d(stackX + 1.0, stackY), Math.toRadians(180), slowDownVelocityConstraint)
                .strafeToConstantHeading(new Vector2d(-36, stackY), speedUpVelocityConstraint)
                .strafeToConstantHeading(new Vector2d(-12, stackY), speedUpVelocityConstraint)
                .strafeToConstantHeading(new Vector2d(12, stackY), speedUpVelocityConstraint)
                .strafeToConstantHeading(new Vector2d(30, stackY), speedUpVelocityConstraint)
                .strafeToLinearHeading(new Vector2d(deliverToBoardPose.position.x, -30), Math.toRadians(180))
                .build();

        // Create the backup trajectory
        Action Backup2 = myBot.getDrive().actionBuilder(new Pose2d(47, -33, Math.toRadians(180)))
                .lineToX(45)
                .build();

        // Build up the Board back to Stack Trajectory
        Action DriveBackToStack2 = myBot.getDrive().actionBuilder(new Pose2d(45, -33, Math.toRadians(180)))
                /* **** Curvy spline route out **** */
                .splineToLinearHeading(new Pose2d(26, stackY, Math.toRadians(180)), Math.toRadians(180))
                /* **** Pure strafe out trajectory **** */
                //.strafeToLinearHeading(new Vector2d(30, stackY), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(12, stackY), Math.toRadians(180), speedUpVelocityConstraint)
                .strafeToLinearHeading(new Vector2d(-36, stackY), Math.toRadians(180), speedUpVelocityConstraint)
                // Return to stack
                .strafeToLinearHeading(new Vector2d(-52, stackY), Math.toRadians(180), speedUpVelocityConstraint)
                .strafeToLinearHeading(new Vector2d(stackPose.position.x, stackY), Math.toRadians(180))
                .build();

        // Build up the Board position 1 to Parking Trajectory
        Action Park = myBot.getDrive().actionBuilder(new Pose2d(deliverToBoardPose.position.x, -33, Math.toRadians(180)))
                .lineToX(45, slowDownVelocityConstraint)
                .strafeToLinearHeading(new Vector2d(46, -24), Math.toRadians(90))
                .build();

        myBot.runAction(new SequentialAction(
                // Drive to Floor Position
                FloorTraj,
                // simulate waiting for purple pixel delivery & resetArm
                new SleepAction(1.0),
                // simulate waiting for slides down
                new SleepAction(0.6),
                DriveToStack,
                // simulate waiting to get to the white pixel pickup location
                new SleepAction(0.7),
                // simulate waiting to pick up the pixels
                new SleepAction(1.05),
                BoardTraj2,
                // simulate waiting for board delivery and april tag correction
                new SleepAction(0.6),
                Backup1,
                // Drive back to the stack - part 1
                DriveBackToStack,
                // simulate waiting to get to the white pixel pickup location
                new SleepAction(0.7),
                // simulate waiting to pick up the pixels
                new SleepAction(1.05),
                // Drive to Board Position 1
                BoardTrajFinal,
                // simulate waiting for board delivery
                new SleepAction(0.6),

                /* ************************************************************
                   ***** EXTRA EXTRA TRIP ATTEMPT *****************************
                   * **********************************************************
                 */
 /*               Backup2,
                // Drive back to the stack - part 1
                DriveBackToStack2,
                // simulate waiting to get to the white pixel pickup location
                new SleepAction(0.7),
                // simulate waiting to pick up the pixels
                new SleepAction(1.05),
                // Drive to Board Position 1
                BoardTrajFinal,
                // simulate waiting for board delivery
                new SleepAction(0.6), */
                // ************************************************************
                // ************************************************************

                // Drive to the parking spot
                Park
                // simulate waiting for slides down, and servo stop
                //, new SleepAction(0.8)
                ));

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    static public void RedBoardDecision() {
        // Look for potential errors
        //***POSITION 1***
        if (autoPosition == 1) {
            deliverToBoardPose = new Pose2d(46,-27,Math.toRadians(180));
        }
        //***POSITION 3***
        else if (autoPosition == 3) {
            deliverToBoardPose = new Pose2d(46,-39,Math.toRadians(180));
        }
        //***POSITION 2***
        else {
            deliverToBoardPose = new Pose2d(46,-33,Math.toRadians(180));
        }
        BoardTraj2 = myBot.getDrive().actionBuilder(new Pose2d(-57,stackY,Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(stackX + 1.0, stackY), Math.toRadians(180), slowDownVelocityConstraint)
                .strafeToConstantHeading(new Vector2d(-36, stackY), speedUpVelocityConstraint)
                .strafeToConstantHeading(new Vector2d(-12, stackY), speedUpVelocityConstraint)
                .strafeToConstantHeading(new Vector2d(12, stackY), speedUpVelocityConstraint)
                .strafeToConstantHeading(new Vector2d(30, stackY), speedUpVelocityConstraint)
                .strafeToLinearHeading(new Vector2d(deliverToBoardPose.position.x, deliverToBoardPose.position.y), Math.toRadians(180))
                .build();
    }

    static public void RedLeftPurplePixelDecision() {
        //***POSITION 1***
        if (autoPosition == 1) {
            deliverToFloorPose = new Pose2d(-41, -20, Math.toRadians(45));
            FloorTraj = myBot.getDrive().actionBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(-38.5, -35.5, Math.toRadians(90)), Math.toRadians(90))
                    .splineToLinearHeading (deliverToFloorPose, Math.toRadians(45))
                    .build();
        }
        //***POSITION 3***
        else if (autoPosition == 3) {
            deliverToFloorPose = new Pose2d(-36, -34.5, Math.toRadians(180));
            FloorTraj = myBot.getDrive().actionBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(-38.5, -35.5, Math.toRadians(90)), Math.toRadians(90))
                    .strafeToLinearHeading(new Vector2d(-27, -35.5), Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(deliverToFloorPose.position.x, deliverToFloorPose.position.y), Math.toRadians(180))
                    .build();
        }
        //***POSITION 2***
        else {
            deliverToFloorPose = new Pose2d(-34, -11.5, Math.toRadians(90));
            FloorTraj = myBot.getDrive().actionBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(-46, -33, Math.toRadians(90)), Math.toRadians(90))
                    .splineToLinearHeading(deliverToFloorPose, Math.toRadians(90))
                    .build();
        }
    }


}