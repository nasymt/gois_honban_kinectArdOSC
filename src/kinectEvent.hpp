//
//  kinectEvent.hpp
//  gois_honban_kinectArdOSC
//
//  Created by 松岡正 on 2015/12/08.
//
//

#ifndef kinectEvent_hpp
#define kinectEvent_hpp

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofEvents.h"
#include "ofxKinect.h"
#define CHAIR_NUM 16



#endif /* kinectEvent_hpp */

class KinectEvent{
public:
    KinectEvent();
    void setup();
    void update(int _mouseX,int _mouseY);
    void draw();
    bool distance_check(int i);
    
    ofxKinect kinect;
    ofxCvColorImage colorImg;
    ofxCvGrayscaleImage grayImage; // grayscale depth image
    ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
    ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
    ofxCvContourFinder contourFinder;
    ofTrueTypeFont font;
    
    bool bThreshWithOpenCV;
    bool bDrawPointCloud;
    int nearThreshold;
    int farThreshold;
    int distance[CHAIR_NUM],temp_dis[CHAIR_NUM];
    int dis_x[CHAIR_NUM],dis_y[CHAIR_NUM];
    int temp_x[CHAIR_NUM],temp_y[CHAIR_NUM];
    int kinect_windowX,kinect_windowY;
    int kiMouseX,kiMouseY;
    int ikichi[CHAIR_NUM];
    int kinectAngle;
    int angle;
};
