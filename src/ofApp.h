#pragma once

#include "ofMain.h"
#include "ofEvents.h"
#include "ofxOsc.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "LedEvent.h"
#include "liveOscEvent.h"

//OSC
#define HOST "localhost"
#define S_PORT 12000
//#define R_PORT 9001
#define NUM_MSG_STRINGS 20
#define CHAIR_NUM 16
#define MAX_DISTANCECOUNT 150

class ofApp : public ofBaseApp{
    
public:
    void setup();
    void update();
    void draw();
    void exit();
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void keyPressed(int key);
    void oscEvent();
    
    void distance_check(int i);
    void sensor_check(int i);
    
    ofImage bgImage;
    ofTrueTypeFont font;
    ofxKinect kinect;
    
    ofxOscSender sender;
    
    //OSC Reciever
    int current_msg_string;
    string msg_strings[NUM_MSG_STRINGS];
    float timers[NUM_MSG_STRINGS];
    int beat,temp_beat;
    int chairCondition[CHAIR_NUM];
    int chair_index;
    
    //kinect
    ofxCvColorImage colorImg;
    ofxCvGrayscaleImage grayImage; // grayscale depth image
    ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
    ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
    ofxCvContourFinder contourFinder;
    
    bool bThreshWithOpenCV;
    bool bDrawPointCloud;
    int nearThreshold;
    int farThreshold;
    int angle;
    int distance[CHAIR_NUM],temp_dis[CHAIR_NUM],dis_sit[CHAIR_NUM],dis_stand[CHAIR_NUM];
    int distanceCount;
    int dis_x[CHAIR_NUM],dis_y[CHAIR_NUM];
    int temp_x[CHAIR_NUM],temp_y[CHAIR_NUM];
    int kinect_windowX,kinect_windowY;
    int kiMouseX,kiMouseY;
    int ikichi[CHAIR_NUM];
    int oscFlag[CHAIR_NUM];
    
    int play_flag[CHAIR_NUM];
    bool bManual,bDistanceEdit;
    int kinectAngle,distanceEditIndex;
    bool bSelectSensor;
    int selectSensorIndex;
    bool bInputArduino[CHAIR_NUM];
    /*int sensorCheck_num[CHAIR_NUM][10];
    bool bSensorCount[CHAIR_NUM];
    int sCountIndex[CHAIR_NUM];*/
    
    int sCount;
    int sensorCheck_num[CHAIR_NUM];
    
    /*--------LED----------*/
    LedEvent myLedEvent;
    bool ledCondition[CHAIR_NUM];
    bool bALL_LED_FIRE;
    bool bLEDBlink[CHAIR_NUM];
    int blinkCount,blinkFlag,bLedStop[CHAIR_NUM];
    int ledIndex;
    bool bSwitchSensor;
    
    
    /*----liveOsc---------*/
    LiveOscEvent liveOsc;
    int rythemFlag[4];
    
    
    
    /*-------kinect--------*/
    int kinectFlag[CHAIR_NUM];
    int average[CHAIR_NUM];
    bool bsetAverage;
    int temp_stand_ikichi[CHAIR_NUM];
    int dis_temp_stand[CHAIR_NUM];
    
    
    
    
};
