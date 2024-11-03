package org.firstinspires.ftc.teamcode.Geronimo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.VelConstraint;
//import com.acmerobotics.roadrunner.trajectory.TrajectorySequence;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "LL AutoAim", group = "Linear OpMode")
public class AkshayLLAutoAim extends LinearOpMode{
    Geronimo control = new Geronimo(true, false,this);
    private Limelight3A limelight;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
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

    //LLConstants
    private static final double CAMERA_HEIGHT = 10.0; // inches
    private static final double TARGET_HEIGHT = 60.0; // inches
    private static final double CAMERA_ANGLE = 25.0;// degrees
    private static final double DESIRED_DISTANCE = 24.0; //desired stopping distance
    private static final double KpAim = 0.1; //Proportional control constant for aiming
    private static final double KpDrive = 0.05; //Proportional control constant for driving

    private DcMotorEx leftFront, leftBack, rightBack, rightFront;


    @Override
    public void runOpMode(){
        drive = new MecanumDrive_Geronimo(hardwareMap, startPose);
        control.Init(hardwareMap); //Init the hardwareMap
        control.InitLimelight(hardwareMap); // Init the limeLight

        waitForStart();

        while(opModeIsActive()){
            LLResult result = limelight.getLatestResult();

            if (result!=null && result.isValid()) {
                double tx = result.getTx(); // Horizontal offset from crosshair to target
                double ty = result.getTy(); // Vertical offset from crosshair to target

                //Calculate distance to target using trigonometry
                double distance = calculateDistance(ty);
                double drive = Range.clip((distance - DESIRED_DISTANCE) * KpDrive, -1.0, 1.0);
                double strafe = Range.clip(tx * KpAim, -1.0, 1.0);
                double rotation = 0.0; // not needed

                // Apply powers to mecanum wheels
                double leftFrontPower = drive + strafe - rotation;
                double leftRearPower = drive - strafe - rotation;
                double rightFrontPower = drive - strafe + rotation;
                double rightRearPower = drive + strafe + rotation;

                // Normalize power values to avoid exceeding max motor power
                double max = Math.max(1.0, Math.abs(leftFrontPower));
                max = Math.max(max, Math.abs(leftRearPower));
                max = Math.max(max, Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(rightRearPower));

                leftFront.setPower(leftFrontPower / max);
                leftBack.setPower(leftRearPower / max);
                rightFront.setPower(rightFrontPower / max);
                rightBack.setPower(rightRearPower / max);

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Distance (inches)", distance);
                telemetry.addData("Left Front Power", leftFrontPower / max);
                telemetry.addData("Left Rear Power", leftRearPower / max);
                telemetry.addData("Right Front Power", rightFrontPower / max);
                telemetry.addData("Right Rear Power", rightRearPower / max);
            } else {
                // No target detected; stop motors
                telemetry.addData("Limelight", "No Targets");
            }

            telemetry.update();
        }

    }
    private double calculateDistance(double ty){
        //Convert angles from degrees to radians
        double cameraAnglesRadians = Math.toRadians(CAMERA_ANGLE);
        double targetAngleRadians = Math.toRadians(ty);

        //Calculate total angle
        double totalAngle = cameraAnglesRadians + targetAngleRadians;

        //Calculate distance
        double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(totalAngle);

        return distance;
    }
}
