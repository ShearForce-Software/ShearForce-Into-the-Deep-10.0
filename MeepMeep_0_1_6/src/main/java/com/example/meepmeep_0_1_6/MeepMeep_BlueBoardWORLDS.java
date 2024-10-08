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

public class MeepMeep_BlueBoardWORLDS {
    static Pose2d startPose;
    static Pose2d stackPose;
    static Pose2d deliverToFloorPose;
    static Pose2d deliverToBoardPose;
    static Action FloorTraj;
    static Action BoardTraj2;
    static Action DriveToStack;
    static RoadRunnerBotEntity myBot;
    static int autoPosition = 1;
    static VelConstraint speedUpVelocityConstraint;
    static AccelConstraint speedUpAccelerationConstraint;
    static VelConstraint slowDownVelocityConstraint;
    static AccelConstraint slowDownAccelerationConstraint;
    static double stackY = 36;
    static double stackX = -58;
	static double wallDriveY = 58.5;
	
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        startPose = new Pose2d(12, 62.5, Math.toRadians(270));
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

        // Build up the start to board delivery trajectory
        BlueBoardDecision();

        // Build up the Board to Floor Trajectory
        /* Determine path to Floor Position */
        BlueBoardPurplePixelDecision();

        // Build up the Floor to Stack Trajectory
        if (autoPosition == 3)
        {
            DriveToStack = myBot.getDrive().actionBuilder(deliverToFloorPose)
                    .strafeToLinearHeading(new Vector2d(12,wallDriveY), Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(-12,wallDriveY), Math.toRadians(180), speedUpVelocityConstraint, slowDownAccelerationConstraint)
                    .strafeToLinearHeading(new Vector2d(-36,wallDriveY), Math.toRadians(180), speedUpVelocityConstraint, slowDownAccelerationConstraint)
                    .strafeToLinearHeading(new Vector2d(stackX + 1.0,wallDriveY), Math.toRadians(180), speedUpVelocityConstraint, slowDownAccelerationConstraint)
                    .strafeToLinearHeading(new Vector2d(stackX, stackY), Math.toRadians(180), null, slowDownAccelerationConstraint)
                    .build();

        }
        else
        {
            DriveToStack = myBot.getDrive().actionBuilder(deliverToFloorPose)
                    .strafeToLinearHeading(new Vector2d(12,wallDriveY), Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(-12,wallDriveY), Math.toRadians(180), speedUpVelocityConstraint, slowDownAccelerationConstraint)
                    .strafeToLinearHeading(new Vector2d(-36,wallDriveY), Math.toRadians(180), speedUpVelocityConstraint, slowDownAccelerationConstraint)
                    .strafeToLinearHeading(new Vector2d(stackX + 1.0, wallDriveY), Math.toRadians(180), speedUpVelocityConstraint, slowDownAccelerationConstraint)
                    .strafeToLinearHeading(new Vector2d(stackX, stackY), Math.toRadians(180), null, slowDownAccelerationConstraint)
                    .build();
        }
        /////
       // Action DriveBackToStack2 = myBot.getDrive().actionBuilder(new Pose2d(48,11.5,Math.toRadians(180)))
        //        .strafeToLinearHeading(new Vector2d(stackPose.position.x, stackPose.position.y), Math.toRadians(180), speedUpVelocityConstraint)
         //       .build();

        // Build up the Stack to Board Position 1 Trajectory
        Action BoardTrajFinal = myBot.getDrive().actionBuilder(stackPose)
                .strafeToLinearHeading(new Vector2d(stackX + 1.0, stackY), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(stackX + 1.0, wallDriveY), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-46,wallDriveY), Math.toRadians(180), speedUpVelocityConstraint)
                .strafeToLinearHeading(new Vector2d(-12,wallDriveY), Math.toRadians(180), speedUpVelocityConstraint)
                .strafeToLinearHeading(new Vector2d(12,wallDriveY), Math.toRadians(180), speedUpVelocityConstraint)
                .strafeToLinearHeading(new Vector2d(46,wallDriveY), Math.toRadians(180), speedUpVelocityConstraint)
                .strafeToLinearHeading(new Vector2d(46,40), Math.toRadians(180), speedUpVelocityConstraint)
                .build();

        // Build up the Board position 3 to Parking Trajectory
        //Action Park = myBot.getDrive().actionBuilder(new Pose2d(46, deliverToBoardPose.position.y, Math.toRadians(180)))
        /* Park the Robot, and Reset the Arm and slides */
        Action Park = myBot.getDrive().actionBuilder(new Pose2d(47, 40, Math.toRadians(180)))
                .lineToX(45, slowDownVelocityConstraint)
                .strafeToLinearHeading(new Vector2d(48, 56), Math.toRadians(270))
                .build();

        myBot.runAction(new SequentialAction(
                // Drive to Board Position
                BoardTraj2,
                // simulate waiting for board delivery and april tag correction
                new SleepAction(0.6),
                FloorTraj,
                // simulate waiting for purple pixel delivery & resetArm
                new SleepAction(1.0),
                DriveToStack,
                // simulate waiting to get to the white pixel pickup location
                new SleepAction(0.7),
                // simulate waiting to pick up the pixels
                new SleepAction(1.05),
                // Drive to Board Position 3
                BoardTrajFinal,
                // simulate waiting for board delivery
                new SleepAction(0.6),
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

    static public void BlueBoardDecision() {
        // Look for potential errors
        //***POSITION 1***
        if (autoPosition == 1) {
            deliverToBoardPose = new Pose2d(46,42,Math.toRadians(180));
        }
        //***POSITION 3***
        else if (autoPosition == 3) {
            deliverToBoardPose = new Pose2d(46,30,Math.toRadians(180));
        }
        //***POSITION 2***
        else {
            deliverToBoardPose = new Pose2d(46,36,Math.toRadians(180));
        }
        BoardTraj2 = myBot.getDrive().actionBuilder(startPose)
                .splineToLinearHeading(deliverToBoardPose, Math.toRadians(0), speedUpVelocityConstraint)
                .build();
    }
    static public void BlueBoardPurplePixelDecision() {
        if (autoPosition == 1) {
            deliverToFloorPose = new Pose2d(10.5, 30, Math.toRadians(180));
            FloorTraj = myBot.getDrive().actionBuilder(deliverToBoardPose)
                    //.setTangent(Math.toRadians(180))
                    .splineToLinearHeading (deliverToFloorPose, Math.toRadians(180))
                    .build();
        }
        else if (autoPosition == 3) {
            deliverToFloorPose = new Pose2d(12, 30, Math.toRadians(0));
            FloorTraj = myBot.getDrive().actionBuilder(deliverToBoardPose)
                    .splineToLinearHeading(new Pose2d(12, deliverToFloorPose.position.y, Math.toRadians(0)), Math.toRadians(180), speedUpVelocityConstraint, slowDownAccelerationConstraint)
                    .strafeToLinearHeading(new Vector2d(deliverToFloorPose.position.x, deliverToFloorPose.position.y), Math.toRadians(0), speedUpVelocityConstraint, slowDownAccelerationConstraint)
                    .build();
        }
        else {
            deliverToFloorPose = new Pose2d(12, 36, Math.toRadians(90));
            FloorTraj = myBot.getDrive().actionBuilder(deliverToBoardPose)
                    //.splineToLinearHeading(new Pose2d(12, 36, Math.toRadians(90)), Math.toRadians(180))
                    .splineToLinearHeading(deliverToFloorPose, Math.toRadians(180), speedUpVelocityConstraint, slowDownAccelerationConstraint)
                    .build();
        }
    }

    }

