package org.opencv.samples.colorblobdetect;


import org.opencv.core.Core;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Mat;

import java.util.Locale;

public class aiSystem {

    //DEBUG
    Boolean debug = true;

    //ROBOT STATICS
    int robot_y = 360; //1050 //720
    int robot_x = 0; //980 //720
    double robot_radius = 800.0/2.0; //this robot radius needs adjusting for repulsive field effect.
    double MaxRobotVel = 100.0;
    double MaxRobotRotate = 100.0;
    double robot_angle = 270.0;
    double goal_angle;
    double previous_goal_angle;
    int ball_reached = 0;
    int searching = 0;

    //FIELD MAP ARRAYS
    double [] attractive_field = new double [370]; //assigning 370 as room for error.
    double [] repulsive_field = new double [370];
    double [] residual_field = new double [370];
    int [] returnVel = new int [2];

    //GLOBAL COLORS
    private Scalar green = new Scalar(0,255,0);

    //PID VARIABLES
    private double integralaccum = 0;
    private double previousError = 0;
    private double GoalP = 10.0;
    private double GoalI = 0;
    private double GoalD = 10.0;



    //BALL LOST LOGIC ****************************************

    //when the ball gets lost behind an obstacle, store its last known LEGIT x and y coords. now
    //we can theoretically "keep the ball in sight" by using the last know x and y and minusing the amount the robot is moving.
    //this means we can virtually see the ball by tracking the bots movement and the last known ball x and y
    //so that we can keep aiming for the ball even when it gets lost behind an obstacle.
    //this logic can also apply for obstacles if need be.
    //we can find the amount the robot moves in the x and y position by initially setting the x_diff and y_diff to 0, then with each frame calculating
    //how much the bot is being told to move in the x direction and y direction
    //and then finally we grab the last know x and y of the ball and add on this x_diff and y_diff to make it out as if the ball is moving
    //even though we cant see it, we are virtually keeping track of it.

    //WHEEL VELOCITIES
    public int[] find_wheel_velocities(int ballX,int ballY,Mat rgbaImage,double [] obstacles){

        //calculate the angle to the goal
        double goal_rad = Math.atan2(ballY - robot_y, ballX - robot_x);
        goal_angle = containAngle360(Math.toDegrees(goal_rad)); //limit angle between 0 and 360 degrees

        //FIND BALL WHEN LOST
        if(ballX == 0 && ballY == 0 && previous_goal_angle < 360 && previous_goal_angle > 270){
            goal_angle = 200;
            searching = 1;
        }
        if(ballX == 0 && ballY == 0 && previous_goal_angle > 0 && previous_goal_angle < 90){
            goal_angle = 160;
            searching = 1;
        }

        if(ballX > 0 && ballY > 0){
            searching = 0;
        }

        //get the field maps
        find_attractive_field(goal_angle);
        find_repulsive_field(obstacles);

        //calculate the residual field as the difference between the two
        for(int j = 0; j < residual_field.length; j++){
            residual_field[j] = Math.max(0, attractive_field[j]-repulsive_field[j]); //- reuplsive_field[i]
        }

        //the heading angle is the max value in the residual array
        //find the heading
        double headingAngle = 0.0;
        double goalError = 0.0;
        for (int i = 0; i < residual_field.length; i++){
            double newnumber = residual_field[i];
            if ((newnumber > residual_field[(int)headingAngle])){
                headingAngle = i;
            }
        }

        //convert angles to radians
        double headingAngle_rad = Math.toRadians(headingAngle);
        double robot_angle_rad = Math.toRadians(robot_angle);

        //find the error and velocities
        if(ballX > 10 && ballY > 10) {
            goalError = signedDelta(robot_angle_rad, headingAngle_rad);
        }

        //START PID LOOP
        double pidOUT = (goalError*GoalP) + (integralaccum*GoalI) + (GoalD*(goalError - previousError));
        integralaccum += goalError;
//        if(integralaccum == 20){ //if we need to reset to integral accumulator
//            integralaccum = 0;
//        }
        previousError = goalError;


        //Find motor Velcoities
        double vel_rotate = (Math.min(MaxRobotRotate, Math.max((-MaxRobotRotate),(pidOUT))));
        double vel;
        if(ballX > 10 && ballY >10) {
            vel = MaxRobotVel * (1.0 - 0.8 * Math.abs(vel_rotate) / MaxRobotRotate);
        }
        else {
            vel = 0;
        }

        //left and right wheel velocities
        double motorVelLeft = moveLeft(vel,vel_rotate,0,0);
        double motorVelRight = moveRight(vel,vel_rotate,0,0);
        if(motorVelLeft < 0){
            motorVelLeft = 0;
        }
        if(motorVelRight < 0){
            motorVelRight = 0;
        }
        returnVel[0]= (int) motorVelLeft;
        returnVel[1]= (int) motorVelRight;

        //determine if ball has been reached
        //this is just test values atm, will need a margin for actual ball being reached
//        if(ballX < 500 && !(ballX == 0) && !(ballY == 0)){
//            ball_reached = 1;
//        }

        //setup the points for drawing the heading line direction.

        Point p1 = new Point(robot_x,robot_y);
        Point p2 = new Point();
        p2.x =  (int)Math.round(p1.x + 500 * Math.cos(headingAngle * Math.PI / 180.0));
        p2.y =  (int)Math.round(p1.y + 500 * Math.sin(headingAngle * Math.PI / 180.0));

        //DEBUG ***************************************************
        if(debug) {
            String steve = String.format(Locale.ENGLISH, "Heading Angle: %f", headingAngle); // Stringify x and y values
            String peteo = String.format(Locale.ENGLISH, "Velocity F: %f  " + "Velocity R: %f", vel, vel_rotate); // Stringify x and y values
            String steveo = String.format(Locale.ENGLISH, "Left Motor Velocity: %f  " + "Right Motor Velocity: %f", motorVelLeft, motorVelRight); // Stringify x and y values
            String toe = String.format(Locale.ENGLISH, "ID: %f  " + "x: %f  " + "y: %f", obstacles[0], obstacles[1], obstacles[2]);
            String rod = String.format(Locale.ENGLISH, "ID: %f  " + "x: %f  " + "y: %f", obstacles[3], obstacles[4], obstacles[5]);
            Imgproc.putText(rgbaImage, steve, new Point(40, 40), Core.FONT_HERSHEY_SIMPLEX, 0.8, green, 2);
            Imgproc.putText(rgbaImage, peteo, new Point(40, 70), Core.FONT_HERSHEY_SIMPLEX, 0.8, green, 2);
            Imgproc.putText(rgbaImage, steveo, new Point(40, 100), Core.FONT_HERSHEY_SIMPLEX, 0.8, green, 2);
            Imgproc.putText(rgbaImage, toe, new Point(40, 130), Core.FONT_HERSHEY_SIMPLEX, 0.8, green, 2);
            Imgproc.putText(rgbaImage, rod, new Point(40, 160), Core.FONT_HERSHEY_SIMPLEX, 0.8, green, 2);
            if(ballX > 0 && ballY > 0) {
                Imgproc.line(rgbaImage, p1, p2, new Scalar (0,0,255), 5);
            }
            //draw attractive field
            for(int i = 0; i <= 360; i++) {
                Imgproc.putText(rgbaImage, ".", new Point(i, ((attractive_field[i]*-100)+350)), Core.FONT_HERSHEY_PLAIN, 4, green, 1);
            }
//            //draw repulsive field
            for(int j = 0; j <= 360; j++) {
                Imgproc.putText(rgbaImage, ".", new Point(j, ((repulsive_field[j]*-100)+450)), Core.FONT_HERSHEY_PLAIN, 4, green, 1);
            }
            //draw residual field
            for(int k = 0; k <= 360; k++) {
                Imgproc.putText(rgbaImage, ".", new Point(k, ((residual_field[k]*-100)+650)), Core.FONT_HERSHEY_PLAIN, 4, green, 1);
            }
        }
        if(searching == 0) {
            previous_goal_angle = goal_angle;
        }
        return returnVel;
    }

    //ATTRACTIVE FIELD
    public void find_attractive_field(double goal_deg) {
        //make the goal_deg the maximum attraction
        attractive_field[(int) goal_deg] = 1;
        double gradient = 1.0/200.0;
        for (int i = 1; i <= 179; i++) {
            double posGoal = (containAngle360((goal_deg + i)));
            double negGoal = (containAngle360((goal_deg - i)));
            int posGoal_int = (int)(posGoal);
            int negGoal_int = (int)(negGoal);
//            System.out.println(pete);
            attractive_field[posGoal_int] = (1 - (i * gradient));
            attractive_field[negGoal_int] = (1 - (i * gradient));
        }

    }

    //REPULSIVE FIELD
    public void find_repulsive_field(double [] obstacles){
        double obs_width = 2*robot_radius+0.1; //obstacle width plus a margin
        //scan 0th, 3rd, 6th, 9th etc elements or IDS
        //reset the repulsive field to 0
        //it is important to do this so that we dont store old object repulsion effects
        for(int k = 1; k <= 361; k++){
            repulsive_field[k] = 0;
        }
        for(int i = 0;i < 10; i++){
            //grab the two next values for each id
            double obs_x = obstacles[(i*3)+1];
            double obs_y = obstacles[(i*3)+2];

//            System.out.println(obs_x);
//            System.out.println(obs_y);

            //find distance and angle to the obstacle from robot
            double obs_dist = Math.max(obs_width, Math.sqrt((Math.pow(obs_x-robot_x,2))+(Math.pow(obs_y-robot_y,2))));
            if(obs_x == 0 && obs_y == 0){
                obs_dist = 0;
            }
            double obs_rad = Math.atan2(obs_y-robot_y,obs_x-robot_x);
            double obs_deg = containAngle360(Math.toDegrees(obs_rad));

            //System.out.println(obs_dist);

            //convert the size of the obstacle to degrees in polar
            double obs_width_rad = Math.asin(obs_width/obs_dist);
            double obs_width_deg = containAngle360(Math.toDegrees(obs_width_rad));

            //System.out.println(obs_deg);

            //compute the first repulsive force WRT the obstacle distance
            //produces a ratio for the obstacle width with its distance. closer the object, larger the ratio or effect and vise versa.
            double obs_effect = Math.max(0, Math.min(1,(robot_radius*2/obs_dist)));
            repulsive_field[(int)obs_deg] = obs_effect;
//            System.out.println(obs_effect);

            //iterate through the size of the obstacle in polar
            for (int j = 1; j <= obs_width_deg; j++) {
                double posGoal = (containAngle360((obs_deg + j)));
                double negGoal = (containAngle360((obs_deg - j)));
                int posGoal_int = (int)(posGoal);
                int negGoal_int = (int)(negGoal);
                //System.out.println(repulsive_field[posGoal_int]);
                repulsive_field[posGoal_int] = Math.max(repulsive_field[posGoal_int], obs_effect);
                repulsive_field[negGoal_int] = Math.max(repulsive_field[negGoal_int], obs_effect);
            }
        }
    }

    //HELPER FUNCTIONS

    private double containAngle360(double angle){
        if(angle < 1){
            angle = angle + 360;
        }
        if(angle >= 361){
            angle = angle - 360;
        }
        return angle;
    }

    private double containRadians180(double radians) {
        if(radians > Math.PI) {
            radians = radians - 2*(Math.PI);
        }
        if(radians <= -(Math.PI)) {
            radians = radians + 2*(Math.PI);
        }
        return radians;
    }

    private double containRadians360(double radians){
        if(radians < 0) {
            radians = radians + 2 * (Math.PI);
        }
        if(radians >= 2*(Math.PI)) {
            radians = radians - 2 * (Math.PI);
        }
        return radians;
    }

    private double signedDelta(double angle1, double angle2){
        double direction = containRadians180(angle2 - angle1);
        double delta_angle = Math.abs(containRadians360(angle1)-containRadians360(angle2));
        double angleOut;

        if(delta_angle < ((2*(Math.PI)) - delta_angle)){
            if(direction > 0){
                angleOut = delta_angle;
            }
            else {
                angleOut = -(delta_angle);
            }
        }
        else {
            if (direction > 0){
                angleOut = (2*(Math.PI)) - delta_angle;
            }
            else {
                angleOut = -((2*(Math.PI)) - delta_angle);
            }
        }
        return angleOut;
    }

    //OUTPUT MOTOR FUNCTIONS

    private double moveLeft(double velocity, double velocity_rotate, double current_velocity, double current_rotate){
        double leftVel = 0.0;
        if(velocity_rotate < 0.0) {
            leftVel = (velocity_rotate)+velocity;
        }
        else {
            leftVel = MaxRobotVel; //when turning right, keep the left motor going at max and decrease speed of right motor.
        }
        return leftVel;
    }
    private double moveRight(double velocity,double velocity_rotate,double current_velocity,double current_rotate){
        double rightVel = 0.0;
        if(velocity_rotate < 0.0){
//            rightVel = (-(velocity_rotate));
            rightVel = MaxRobotVel; //when turning left, keep speed of right motor at max and decrease speed of left motor.
        }
        if(velocity_rotate > 0.0) {
            rightVel = (-(velocity_rotate))+velocity;
        }
        return rightVel;
    }

    //BALL REACHED FUNCTION

    public int ball_reached(){
        return ball_reached;
    }
}
