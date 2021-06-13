//Run code for the blue right position (1 wobble goal)

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Disabled
@Autonomous(name = "BlueRight2", group = "Taus")
public class BlueRight2 extends AutonomousMethods {

    @Override
    public void runOpMode() throws InterruptedException {

        //initializing robot
        initializeRobot();
        //detect number of rings
        int numberOfRings = findNumRings(bmp);
        telemetry.addData("rings", numberOfRings);
        telemetry.update();
        bmp.recycle();


        stopAndResetEncoders();
        controlIndexServo(1);
        controlBlocker(.375);

        //pick up wobble goal
        controlClawServo(.25);//close
        controlArmServo(.25);//down
        //shoot
        robot.shooter.setVelocityPIDFCoefficients(p,i,d,f);
        robot.shooter.setVelocity(powerShotPower);

        switch (numberOfRings){
            case 0:
                //code
                forward(.5, 2, 10);//16
                shoot(-26, (2195*28)/60.0);
                toAngle(0, 1);
                forward(.5, 0, 26);//20
                strafeLeft(.3, 0, 23);
                toAngle(-20, 1);
                dropWobbleGoal();
                toAngle(0, 1);
                strafeRight(.5, 1, 4);
                backward(.5, 0, 8);
                break;
            case 1:
                //code
                forward(.5, 2, 10);//16
                shoot(-26, (2195*28)/60.0);
                toAngle(0, 1);
                forward(.5, 1, 26);//20
                toAngle(-15,1);
                dropWobbleGoal();
                toAngle(0,1);
                strafeRight(.5, 0, 3);
                backward(.5, 2, 20);
                toAngle(90, 1);
                setIntakePower(1);
                backward(.5, 0, 11);
                forward(.5, 0, 11);
                toAngle(0, 1);
                forward(.5, 0, 20);
                shoot(-26, (2195*28)/60.0);
                toAngle(0, 1);
                forward(.5, 0, 12);
                break;
            case 4:
                //code
                forward(.5, 1, 15);
                shoot(-26, (2285*28)/60.0);
                toAngle(-90, 1);
                setIntakePower(1);
                backward(1, 0, 6, 6, 2, 6);
                forward(1, 0, 6);
                shoot(-26, (2285*28)/60.0);
                toAngle(-90, 1);
                setIntakePower(1);
                backward(.3, 0, 23);
                forward(.3, 0, 23);
                toAngle(0, 1);
                forward(.5, 0, 20);
                shoot(-26, (2150*28)/60.0);
                toAngle(0, 1);
                forward(.5, 2, 10);
                strafeLeft(.5, 0, 23);
                toAngle(45, 1);
                dropWobbleGoal();
                strafeRight(.5, 1, 23);
                toAngle(0, 1);
                backward(.5, 0, 12);
                break;
        }

    }
}
