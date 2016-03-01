//
//  kinectEvent.cpp
//  gois_honban_kinectArdOSC
//
//  Created by 松岡正 on 2015/12/08.
//
//

#include "kinectEvent.hpp"

KinectEvent::KinectEvent(){
    ofSetLogLevel(OF_LOG_VERBOSE);
    kinect.setRegistration(true);
    kinect.init();
    kinect.open();
    
    if(kinect.isConnected()) {
        ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
        ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
        ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
        ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
    }
    
#ifdef USE_TWO_KINECTS
    kinect2.init();
    kinect2.open();
#endif
    colorImg.allocate(kinect.width, kinect.height);
    //    grayImage.allocate(kinect.width, kinect.height);
    //    grayThreshNear.allocate(kinect.width, kinect.height);
    //    grayThreshFar.allocate(kinect.width, kinect.height);
    
    nearThreshold = 230;
    farThreshold = 70;
    bThreshWithOpenCV = true;
    
    ofSetFrameRate(60);
    
    // zero the tilt on startup
    angle = 0;
    kinect.setCameraTiltAngle(angle);
    
    
    /*----------kinect distance checker setup -------------*/
    kinect_windowX=400;kinect_windowY=300;
    dis_x[0]=50; dis_y[0]=30;
    dis_x[1]=50; dis_y[1]=170;
    dis_x[2]=50; dis_y[2]=295;
    dis_x[3]=50; dis_y[3]=450;
    dis_x[4]=200; dis_y[4]=40;
    dis_x[5]=200; dis_y[5]=170;
    dis_x[6]=200; dis_y[6]=295;
    dis_x[7]=200; dis_y[7]=450;
    dis_x[8]=300; dis_y[8]=50;
    dis_x[9]=300; dis_y[9]=200;
    dis_x[10]=300; dis_y[10]=350;
    dis_x[11]=300; dis_y[11]=430;
    dis_x[12]=400; dis_y[12]=100;
    dis_x[13]=400; dis_y[13]=200;
    dis_x[14]=400; dis_y[14]=300;
    dis_x[15]=400; dis_y[15]=400;
    
    for(int i=0;i<CHAIR_NUM;i++){
        temp_x[i]=ofMap(dis_x[i],0,640,0,kinect_windowX);
        temp_y[i]=ofMap(dis_y[i],0,480,0,kinect_windowY);
    }
    
    for(int i=0;i<CHAIR_NUM;i++){
        ikichi[i]=500;
    }
    font.loadFont("Avenir.ttc", 20 );
}

void KinectEvent::update(int mouseX,int mouseY){
    kinect.update();
    if(kinect.isFrameNew()) {
        // we do two thresholds - one for the far plane and one for the near plane
        // we then do a cvAnd to get the pixels which are a union of the two thresholds
        if(bThreshWithOpenCV) {
            grayThreshNear = grayImage;
            grayThreshFar = grayImage;
            grayThreshNear.threshold(nearThreshold, true);
            grayThreshFar.threshold(farThreshold);
            cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
        } else {
            ofPixels & pix = grayImage.getPixels();
            int numPixels = pix.size();
            for(int i = 0; i < numPixels; i++) {
                if(pix[i] < nearThreshold && pix[i] > farThreshold) {
                    pix[i] = 255;
                } else {
                    pix[i] = 0;
                }
            }
        }
        // update the cv images
        grayImage.flagImageChanged();
        
        contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
        
        /*-----kinect GET Distance----------*/
        for(int i=0;i<16;i++)distance[i] = kinect.getDistanceAt(dis_x[i], dis_y[i]);
        kiMouseX=ofMap(mouseX, 500, 500+kinect_windowX, 0, 640);
        kiMouseY=ofMap(mouseY, 50, 50+kinect_windowY,0,480);
        if(kiMouseX<0)kiMouseX=0;
        else if(kiMouseX>640)kiMouseX=640;
        if(kiMouseY<0)kiMouseY=0;
        else if(kiMouseY>480)kiMouseY=480;
    }
    kinect.setCameraTiltAngle(kinectAngle);
}

void KinectEvent::draw(){
    ofSetColor(255, 255, 255);
    kinect.draw(500, 50, 400, 300);
    ofSetColor(255,0,0);
    ofRect(500+temp_x[0],temp_y[0]+50,5,5);
    ofSetColor(255,255,255);
    font.drawString("distance", 100, 450);
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            font.drawString(ofToString(i*4+j+1)+":"+ofToString(distance[i*4+j]), 100+j*120, 480+i*30);
        }
    }
    font.drawString("ikichi"+ofToString(ikichi[0]), 100, 400);
    font.drawString("X:"+ofToString(kiMouseX),600,400);
    font.drawString("Y:"+ofToString(kiMouseY),700,400);

}

bool KinectEvent::distance_check(int i){
    if(distance[i]>0)temp_dis[i]=distance[i];
    else distance[i]=temp_dis[i];
    if(distance[i]<ikichi[i])return true;
    if(distance[i]>ikichi[i]+500)return false;
}

