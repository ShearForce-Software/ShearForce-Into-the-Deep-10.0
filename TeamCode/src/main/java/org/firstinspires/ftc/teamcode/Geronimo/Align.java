package org.firstinspires.ftc.teamcode.Geronimo;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.acmerobotics.roadrunner.trajectory.TrajectorySequence;


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

import org.firstinspires.ftc.teamcode.Geronimo.MecanumDrive_Geronimo;
import org.firstinspires.ftc.teamcode.Gertrude.Gertrude;

@Autonomous(name = "Akshay Align")
public class Align extends LinearOpMode{
    Geronimo control = new Geronimo(true, false,this);
    MecanumDrive_Geronimo drive;
    Pose2d startPose;
    Pose2d deliverToFloorPose;
    Pose2d deliverToBoardPose;
    Pose2d stackPose;
    Action WallTraj;
    Action DriveToStack;
    Action BoxTraj;
    Action Park;
    Action DriveBackToStack;
    VelConstraint speedUpVelocityConstraint;
    AccelConstraint speedUpAccelerationConstraint;
    VelConstraint slowDownVelocityConstraint;
    AccelConstraint slowDownAccelerationConstraint;

    double stackY = 36;
    double stackX = -59;
    double wallDriveY = 58.5;

    double autoPosition = 3;
    Limelight3A limelight;
    VelConstraint velocityConstraint;
    AccelConstraint accelerationConstraint;


    @Override
    public void runOpMode(){
        //Initialization shit
        Pose2d startPose = new Pose2d(0,0,0);
        drive = new MecanumDrive_Geronimo(hardwareMap, startPose);
        control.Init(hardwareMap);
        control.InitLimelight(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            double num = control.GetStrafeOffsetInInches("block", 10.3);
            telemetry.addLine("current dist"+ num);

            if(gamepad1.dpad_up && Math.abs(num)>0.2){
                Action strafeRightSlowAction = drive.actionBuilder(startPose)
                        .strafeToConstantHeading(
                                new Vector2d(0, num)
                        )
                        .build();
                Actions.runBlocking(strafeRightSlowAction);
            }
            telemetry.update();
        }


    }





}
