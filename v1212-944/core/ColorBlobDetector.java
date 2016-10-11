package org.opencv.samples.colorblobdetect;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;

public class ColorBlobDetector {
    // Lower and Upper bounds for range checking in HSV color space
    private Scalar ball_lower =  new Scalar(0,150,150,0);
    private Scalar ball_upper = new Scalar(15,255,255,0);
    //private Scalar ball_lower_second = new Scalar(150,140,140,0);
    //private Scalar ball_upper_second = new Scalar(179,255,255,0);
    private Scalar obstacle_lower = new Scalar(0,0,0,0);
    private Scalar obstacle_upper = new Scalar(179,255,70,0);
    private Scalar goal_lower = new Scalar(0,0,0,0);
    private Scalar goal_upper = new Scalar(0,0,0,0);

    // Cache
    Mat mPyrDownMat = new Mat();
    Mat ball_hierarchy = new Mat();
    Mat obstacle_hierarchy = new Mat();
    Mat hsv_image = new Mat();
    Mat ball_image = new Mat();
    Mat ball_image_second = new Mat();
    Mat ball_image_out = new Mat();
    Mat obstacle_image = new Mat();
    Mat goal_image = new Mat();
    Scalar Colour = new Scalar(0, 0, 255);
    Scalar obstacle_colour = new Scalar(0,255,0);
    Scalar Green = new Scalar(0, 255 ,0);
    int ball_y = 0;
    int ball_x = 0;
    int mX;
    int mY;
    double maxArea = 25;
    double obstacle_maxArea = 500;
    double [] obstacle_array = new double [1000];

    public void Configure(Mat Image,int ball_reached){
        Imgproc.pyrDown(Image, mPyrDownMat); //Downsizes the image twice
        Imgproc.pyrDown(mPyrDownMat, mPyrDownMat);
        //Imgproc.GaussianBlur(mPyrDownMat, mPyrDownMat, new Size(5,5), 0);    //Guassian Blur
        Imgproc.cvtColor(mPyrDownMat, hsv_image, Imgproc.COLOR_RGB2HSV); //Converts the image into HSV colour Space
        if(ball_reached > 0){
            find_goal(Image,hsv_image);
        }
        else{
            find_ball(Image, hsv_image);
        }
        find_obstacles(Image,hsv_image);
    }

    public void find_ball(Mat rgbaImage,Mat Timage) {
        int store = 0;
        Core.inRange(Timage, ball_lower, ball_upper, ball_image_out); //Threshold the image in the colour required ( in this case its Orange )
        //Core.inRange(Timage, ball_lower_second, ball_upper_second, ball_image_second); //Threshold the image in the colour required ( in this case its Orange )
        //Core.bitwise_or(ball_image,ball_image_second,ball_image_out);
        //Imgproc.erode(ball_image_out, ball_image_out, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2,2))); // Erode Thresholded Image
        Imgproc.dilate(ball_image_out, ball_image_out, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2,2))); // DIlate Thresholded Image
        List<MatOfPoint> ball_contours = new ArrayList<MatOfPoint>(); // Create array to put contours into
        Imgproc.findContours(ball_image_out, ball_contours, ball_hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE); // Find Existing Contours

        ball_x = 0;
        ball_y = 0;

        //create an array to store the contours larger than max area. also clears the previous contour array.
        List<MatOfPoint> draw_contours = new ArrayList<MatOfPoint>();
        //iterate on each contour in the obstacle_contours array
        for(int i = 0; i < ball_contours.size();i++){
            if (Imgproc.contourArea(ball_contours.get(i)) > obstacle_maxArea) {
                draw_contours.add(store,ball_contours.get(i));
                Core.multiply(ball_contours.get(i), new Scalar(4, 4), ball_contours.get(i)); //Rescale to fit original image
                Imgproc.drawContours(rgbaImage, draw_contours, -1, Colour, 5); //Draw criteria matching contours
                Moments p = Imgproc.moments(draw_contours.get(store), false);
                mX = (int) (p.get_m10() / p.get_m00());
                mY = (int) (p.get_m01() / p.get_m00());
                ball_x = mX;
                ball_y = mY;
                store++;
            }
        }
    }

    public void find_goal(Mat rgbaImage,Mat Timage) {
        int store = 0;
        Core.inRange(Timage, goal_lower, goal_upper, goal_image); //Threshold the image in the colour required ( in this case its Orange )
        Imgproc.erode(goal_image, goal_image, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2,2))); // Erode Thresholded Image
        Imgproc.dilate(goal_image, goal_image, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2,2))); // DIlate Thresholded Image
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>(); // Create array to put contours into
        Imgproc.findContours(goal_image, contours, ball_hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE); // Find Existing Contours

        //intialise ball_x and ball_y to 0
        ball_x = 0;
        ball_y = 0;

        //create an array to store the contours larger than max area. also clears the previous contour array.
        List<MatOfPoint> draw_contours = new ArrayList<MatOfPoint>();
        //iterate on each contour in the obstacle_contours array
        for(int i = 0; i < contours.size();i++){
            if (Imgproc.contourArea(contours.get(i)) > obstacle_maxArea) {
                draw_contours.add(store,contours.get(i));
                Core.multiply(contours.get(i), new Scalar(4, 4), contours.get(i)); //Rescale to fit original image
                Imgproc.drawContours(rgbaImage, draw_contours, -1, obstacle_colour, 5); //Draw criteria matching contours
                Moments p = Imgproc.moments(draw_contours.get(store), false);
                mX = (int) (p.get_m10() / p.get_m00());
                mY = (int) (p.get_m01() / p.get_m00());
                ball_x = mX;
                ball_y = mY;
                store++;
            }
        }
    }

    public void find_obstacles(Mat rgbaImage, Mat Timage) {
        //reset the obstacle array
        int store = 0;
        for(int j = 0; j < obstacle_array.length; j++){
            obstacle_array[j] = 0; //OBSTACLE ID
        }

        Core.inRange(Timage, obstacle_lower, obstacle_upper, obstacle_image); //Threshold the image in the colour required ( in this case its Orange )
        //Imgproc.erode(obstacle_image, obstacle_image, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2,2))); // Erode Thresholded Image
        Imgproc.dilate(obstacle_image, obstacle_image, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5,5))); // DIlate Thresholded Image
        List<MatOfPoint> obstacle_contours = new ArrayList<MatOfPoint>(); // Create array to put contours into
        Imgproc.findContours(obstacle_image, obstacle_contours, obstacle_hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE); // Find Existing Contours

        //create an array to store the contours larger than max area. also clears the previous contour array.
        List<MatOfPoint> draw_contours = new ArrayList<MatOfPoint>();
        //iterate on each contour in the obstacle_contours array
        for(int i = 0; i < obstacle_contours.size();i++){
            if (Imgproc.contourArea(obstacle_contours.get(i)) > obstacle_maxArea) {
                draw_contours.add(store,obstacle_contours.get(i));
                Core.multiply(obstacle_contours.get(i), new Scalar(4, 4), obstacle_contours.get(i)); //Rescale to fit original image
                Imgproc.drawContours(rgbaImage, draw_contours, -1, obstacle_colour, 5); //Draw criteria matching contours
                Moments p = Imgproc.moments(draw_contours.get(store), false);
                mX = (int) (p.get_m10() / p.get_m00());
                mY = (int) (p.get_m01() / p.get_m00());
                obstacle_array[(store * 3)] = store + 1; //OBSTACLE ID
                obstacle_array[(store * 3) + 1] = mX;
                obstacle_array[(store * 3) + 2] = mY;
                store++;
            }
        }
    }

    public int ball_coordsX() {
        return ball_x;
    }

    public int ball_coordsY() {
        return ball_y;
    }

    public double [] obstacles() {
        //make an array which finds obstacle blobs and stores there data under ID's
        //i.e (id,x,y,id,x,y) as the array format. return the array
        return obstacle_array;
    }
}