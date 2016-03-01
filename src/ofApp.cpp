#include "ofApp.h"

void ofApp::setup(){
    //kinect setup
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
    font.loadFont("Avenir.ttc", 20 );
    
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
    
    ofSetVerticalSync(true);
    ofSetFrameRate(60);
    //    ofSetBackgroundAuto(false);
    
    sender.setup(HOST,S_PORT);
    
    for(int i=0;i<16;i++){
        chairCondition[i]=false;
    }
    
    
    /*----------kinect distance checker setup -------------*/
    kinect_windowX=400;kinect_windowY=300;
    dis_x[0]=523; dis_y[0]=422;
    dis_x[1]=382; dis_y[1]=433;
    dis_x[2]=222; dis_y[2]=441;
    dis_x[3]=59; dis_y[3]=456;
    dis_x[4]=518; dis_y[4]=300;
    dis_x[5]=374; dis_y[5]=300;
    dis_x[6]=216; dis_y[6]=313;
    dis_x[7]=49; dis_y[7]=320;
    dis_x[8]=505; dis_y[8]=164;
    dis_x[9]=372; dis_y[9]=176;
    dis_x[10]=208; dis_y[10]=187;
    dis_x[11]=44; dis_y[11]=195;
    dis_x[12]=504; dis_y[12]=41;
    dis_x[13]=363; dis_y[13]=48;
    dis_x[14]=203; dis_y[14]=56;
    dis_x[15]=35; dis_y[15]=52;
    
    for(int i=0;i<CHAIR_NUM;i++){
        ikichi[i]=1950;
    }
    
    for(int i=0;i<CHAIR_NUM;i++){
        temp_x[i]=ofMap(dis_x[i],0,640,0,kinect_windowX);
        temp_y[i]=ofMap(dis_y[i],0,480,0,kinect_windowY);
        play_flag[i]=0;
    }
    bManual=true;
    kinectAngle=22;
}

void ofApp::update(){
    kinect.update();
    if(kinect.isFrameNew()) {
        grayImage.setFromPixels(kinect.getDepthPixels());
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
    for(int i=0;i<CHAIR_NUM;i++){
        if(!bInputArduino[i]){//kinect入力
            distance_check(i);
        }else {//Arduino入力
            sensor_check(i);
        }
    }
    if(distanceCount<MAX_DISTANCECOUNT)distanceCount++;
    else distanceCount=0;
    
    
    if(bsetAverage){
        bsetAverage=false;
        for(int i=0;i<CHAIR_NUM;i++){
            average[i]=distance[i];
            cout<<"ave:"<<i<<":"<<distance[i]<<endl;
        }
        cout<<"------------"<<endl;
    }
    
    
    
    /*-------------------LED----------------*/
    for(int i=0;i<CHAIR_NUM;i++){
        if(chairCondition[i]==1){
            myLedEvent.sendSerial(i, true);
            liveOsc.playTrack(i);
            liveOsc.sendToVisual(i,true);
            chairCondition[i]=2;
            cout<<"Send Play! >>"<<i<<endl;
        }
        if(chairCondition[i]==5){
            myLedEvent.sendSerial(i, false);
            liveOsc.stopTrack(i);
            chairCondition[i]=0;
            liveOsc.sendToVisual(i,false);
            cout<<"Send Stop! >>"<<i<<endl;
        }
        else if(chairCondition[i]==6){
            myLedEvent.sendSerial(i, false);
            liveOsc.stopTrack(i);
            chairCondition[i]=4;
            liveOsc.sendToVisual(i,false);
        }
        myLedEvent.beatBlink(i, liveOsc.beat, chairCondition[i]);
    }
    myLedEvent.readSerial();
    /*-------------OSC----------------*/
    liveOsc.receiveOsc();
    liveOsc.sendBeat(liveOsc.beat);
}

void ofApp::draw(){
    ofBackground(100,100,100);
    ofSetColor(255,255,255);
    kinect.draw(500,50,400,300);
    ofSetColor(255,0,0);
    ofRect(500+temp_x[0],temp_y[0]+50,5,5);
    
    /*--------distance表示画面---------*/
    ofSetColor(255,255,255);
    font.drawString("distance", 100, 450);
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            font.drawString(ofToString(i*4+j+1)+":"+ofToString(distance[i*4+j]), 100+j*120, 480+i*30);
        }
    }
    font.drawString("ikichi"+ofToString(ikichi[0]), 100, 400);
    
    
    /*--------着席位置モニター-----------*/
    font.drawString("gois monitor", 100, 70);
    ofSetColor(255, 0, 103);
    ofRect(280, 40, 140, 40);
    ofSetColor(255);
    if(!bManual)font.drawString("AUTO", 310, 70);
    else font.drawString("MANUAL", 290, 70);
    
    for(int i=0;i<4;i++){/*----GOIS MONITOR----*/
        for(int j=0;j<4;j++){
            if(chairCondition[i*4+j])ofSetColor(255,0,103);
            else ofSetColor(255, 255, 255);
            ofRect(120+j*70,100+i*70,40,40);
        }
    }
    
    ofSetColor(255);
    font.drawString("BEAT:"+ofToString(liveOsc.beat), 300, 400);
    font.drawString("X:"+ofToString(kiMouseX),600,400);
    font.drawString("Y:"+ofToString(kiMouseY),700,400);
    
    if(bDistanceEdit){//distanceエディット画面
        ofSetColor(255, 255, 0);
        ofRect(500+temp_x[distanceEditIndex],temp_y[distanceEditIndex]+50,5,5);
        ofSetColor(255, 0, 103);
        ofRect(250, 420, 250, 40);
        ofSetColor(255, 255, 255);
        font.drawString("DISTANCE EDIT:"+ofToString(distanceEditIndex+1), 250, 450);
    }
    if(bSwitchSensor){
        ofSetColor(255, 255, 255);
        font.drawString("SENSOR", 100, 600);
        for(int i=0;i<4;i++){
            for(int j=0;j<4;j++){
                font.drawString(ofToString(i*4+j)+":"+ofToString(myLedEvent.sensorVal[i*4+j]),100+j*80, 625+i*25);
            }
        }
    }
    if(bSelectSensor){//入力ソース一覧画面
        ofSetColor(255, 0, 103);
        ofRect(800, 420, 160, 30);
        ofSetColor(255, 255, 255);
        font.drawString("INPUT EDIT", 805, 450);
    }
    font.drawString("INPUT SOURCE",580, 450);
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            string str;
            if(bInputArduino[i*4+j])str="ard";
            else str="kin";
            font.drawString(ofToString(i*4+j)+":"+str, 600+j*100, 480+i*25);
            
            font.drawString(ofToString(i*4+j)+":"+ofToString(average[i*4+j]), 600+j*100, 600+i*25);
        }
    }
    
    /*-------beat表示--------*/
    ofSetColor(255, 0, 103);
    for(int i=0;i<4;i++){
        if(liveOsc.beat==i||liveOsc.beat==i+4)ofRect(120+i*70,350,10,10);
    }
    
    
}

void ofApp::mousePressed(int x, int y, int button){
    
    
    
}

void ofApp::mouseReleased(int x, int y, int button){
    dis_x[distanceEditIndex]=kiMouseX;
    dis_y[distanceEditIndex]=kiMouseY;
    temp_x[distanceEditIndex]=ofMap(dis_x[distanceEditIndex],0,640,0,kinect_windowX);
    temp_y[distanceEditIndex]=ofMap(dis_y[distanceEditIndex],0,480,0,kinect_windowY);
}

void ofApp::keyPressed(int key){
    if(bSelectSensor){//入力ソース調整用
        for(int i=1;i<10;i++){
            char c = ofToChar(ofToString(i));
            if(key==c)selectSensorIndex=i-1;
        }
        if(key=='0')selectSensorIndex=9;
        else if(key=='q')selectSensorIndex=10;
        else if(key=='w')selectSensorIndex=11;
        else if(key=='e')selectSensorIndex=12;
        else if(key=='r')selectSensorIndex=13;
        else if(key=='t')selectSensorIndex=14;
        else if(key=='y')selectSensorIndex=15;
        
        if(!bInputArduino[selectSensorIndex])bInputArduino[selectSensorIndex]=true;
        else bInputArduino[selectSensorIndex]=false;
        selectSensorIndex=0;
    }
    
    if(bDistanceEdit){//Distance調整用
        for(int i=1;i<10;i++){
            char c = ofToChar(ofToString(i));
            if(key==c)distanceEditIndex=i-1;
        }
        if(key=='0')distanceEditIndex=9;
        else if(key=='q')distanceEditIndex=10;
        else if(key=='w')distanceEditIndex=11;
        else if(key=='e')distanceEditIndex=12;
        else if(key=='r')distanceEditIndex=13;
        else if(key=='t')distanceEditIndex=14;
        else if(key=='y')distanceEditIndex=15;
    }
    else if(!bDistanceEdit&&!bSelectSensor){
        for(int i=4;i<8;i++){
            char c = ofToChar(ofToString(i));
            if(key==c){
                if(chairCondition[i-4]==0)chairCondition[i-4]=1;
                else if(chairCondition[i-4]==2)chairCondition[i-4]=5;
            }
        }
        if(key=='r'){
            if(chairCondition[4]==0)chairCondition[4]=1;
            else if(chairCondition[4]==2)chairCondition[4]=5;
        }
        if(key=='t'){
            if(chairCondition[5]==0)chairCondition[5]=1;
            else if(chairCondition[5]==2)chairCondition[5]=5;
        }
        if(key=='y'){
            if(chairCondition[6]==0)chairCondition[6]=1;
            else if(chairCondition[6]==2)chairCondition[6]=5;
        }
        if(key=='u'){
            if(chairCondition[7]==0)chairCondition[7]=1;
            else if(chairCondition[7]==2)chairCondition[7]=5;
        }
        if(key=='f'){//Rythem1-1
            if(chairCondition[8]==0){
                chairCondition[8]=1;
                rythemFlag[0]++;
            }
            if(chairCondition[8]==2){//離席
                rythemFlag[0]--;
                if(chairCondition[12]!=2){//もう一方未着席
                    chairCondition[8]=5;
                }
                else if(chairCondition[12]==2){
                    myLedEvent.sendSerial(8, false);
                    chairCondition[8]=0;
                    liveOsc.playTrack(12);
                }
            }
        }
        if(key=='g'){//Rythem1-2
            if(chairCondition[9]==0){
                chairCondition[9]=1;
                rythemFlag[1]++;
            }
            if(chairCondition[9]==2){//離席
                rythemFlag[1]--;
                if(chairCondition[13]!=2){//もう一方未着席
                    chairCondition[9]=5;
                }
                else if(chairCondition[13]==2){
                    myLedEvent.sendSerial(9, false);
                    chairCondition[9]=0;
                    liveOsc.playTrack(13);
                }
            }
        }
        if(key=='h'){//Rythem1-3
            if(chairCondition[10]==0){
                chairCondition[10]=1;
                rythemFlag[2]++;
            }
            if(chairCondition[10]==2){//離席
                rythemFlag[2]--;
                if(chairCondition[14]!=2){//もう一方未着席
                    chairCondition[10]=5;
                }
                else if(chairCondition[14]==2){
                    myLedEvent.sendSerial(10, false);
                    chairCondition[10]=0;
                    liveOsc.playTrack(14);
                }
            }
        }
        if(key=='j'){//Rythem1-4
            if(chairCondition[11]==0){
                chairCondition[11]=1;
                rythemFlag[3]++;
            }
            if(chairCondition[11]==2){//離席
                rythemFlag[3]--;
                if(chairCondition[15]!=2){//もう一方未着席
                    chairCondition[11]=5;
                }
                else if(chairCondition[15]==2){
                    myLedEvent.sendSerial(11, false);
                    chairCondition[11]=0;
                    liveOsc.playTrack(15);
                }
            }
        }
        if(key=='v'){//Rythem2-1
            if(chairCondition[12]==0){
                chairCondition[12]=1;
                rythemFlag[0]++;
            }
            else if(chairCondition[12]==2){
                
                rythemFlag[0]--;
                if(chairCondition[8]!=2){
                    chairCondition[12]=5;
                }
                if(chairCondition[8]==2){
                    chairCondition[12]=0;
                    myLedEvent.sendSerial(12, false);
                }
            }
        }
        if(key=='b'){//Rythem2-2
            if(chairCondition[13]==0){
                chairCondition[13]=1;
                rythemFlag[1]++;
            }
            else if(chairCondition[13]==2){
                rythemFlag[1]--;
                if(chairCondition[9]!=2){
                    chairCondition[13]=5;
                }
                if(chairCondition[9]==2){
                    chairCondition[13]=0;
                    myLedEvent.sendSerial(13, false);
                    liveOsc.playTrack(9);
                }
            }
        }
        if(key=='n'){//Rythem2-3
            if(chairCondition[14]==0){
                chairCondition[14]=1;
                rythemFlag[2]++;
            }
            else if(chairCondition[14]==2){
                
                rythemFlag[2]--;
                if(chairCondition[10]!=2){
                    chairCondition[14]=5;
                }
                if(chairCondition[10]==2){
                    chairCondition[14]=0;
                    myLedEvent.sendSerial(14, false);
                    liveOsc.playTrack(10);
                }
            }
        }
        if(key=='m'){//Rythem2-4
            if(chairCondition[15]==0){
                chairCondition[15]=1;
                rythemFlag[3]++;
            }
            else if(chairCondition[15]==2){
                
                rythemFlag[3]--;
                if(chairCondition[11]!=2){
                    chairCondition[15]=5;
                }
                if(chairCondition[11]==2){
                    chairCondition[15]=0;
                    myLedEvent.sendSerial(15, false);
                    liveOsc.playTrack(11);
                }
            }
        }
    }
    
    if(key==OF_KEY_RETURN&&!bSelectSensor){
        if(!bManual)bManual=true;
        else if(bManual)bManual=false;
    }
    if(key==OF_KEY_UP){
        kinectAngle+=2;
        cout<<kinectAngle<<endl;
    }
    else if(key==OF_KEY_DOWN){
        kinectAngle-=2;
        cout<<kinectAngle<<endl;
    }
    if(key=='c'){
        if(!bDistanceEdit)bDistanceEdit=true;
        else bDistanceEdit=false;
        cout<<"change!!"<<bDistanceEdit<<endl;
    }
    else if(key=='x'){
        if(!bSwitchSensor)bSwitchSensor=true;
        else bSwitchSensor=false;
    }
    else if(key=='z'){
        if(!bSelectSensor)bSelectSensor=true;
        else bSelectSensor=false;
    }
    else if(key==OF_KEY_RIGHT_SHIFT){
        if(!bsetAverage)bsetAverage=true;
    }
    else if(key==OF_KEY_LEFT_SHIFT){//リセット
//        cout<<"chairCondition"<<endl;
//        for(int i=0;i<CHAIR_NUM;i++){
//            cout<<i<<":"<<chairCondition[i]<<endl;
//        }
//        cout<<"-------------"<<endl;
        for(int i=0;i<CHAIR_NUM;i++){
            chairCondition[i]=5;
            oscFlag[i]=0;
        }
        
    }
}
void ofApp::exit() {
    kinect.setCameraTiltAngle(0); // zero the tilt on exit
    kinect.close();
    
#ifdef USE_TWO_KINECTS
    kinect2.close();
#endif
}

void ofApp::distance_check(int i){
    if(distance[i]>0)temp_dis[i]=distance[i];
    else distance[i]=temp_dis[i];
    
    if(!bManual){
        if(distance[i]<temp_stand_ikichi[i]){
            dis_temp_stand[i]++;
        }
//        if(distance[i]<ikichi[i]){//着席
        if(distance[i]<average[i]-300){//着席
            dis_sit[i]++;
            //cout<<"dis:"<<i<<":"<<dis_sit[i]<<":"<<distanceCount<<":"<<chairCondition[i]<<endl;
        }
       // else if(distance[i]>ikichi[i]+500&&(chairCondition[i]!=0)){
        else if(distance[i]>average[i]-100&&(chairCondition[i]!=0)){
            dis_stand[i]++;
        }
//        cout<<"disCount"+distanceCount<<endl;
        if(distanceCount>=MAX_DISTANCECOUNT) {
//            cout<<"dis_sit:"<<dis_sit[i]<<" i:"<<i<<endl;
            
            if(dis_temp_stand[i]>110){
                if(chairCondition[i]==0){
                    kinectFlag[i]=1;
                }
            }
            
            if(dis_sit[i]>110){//座ったとみなす
                cout<<"ok"<<endl;
                if(chairCondition[i]==0){
                    chairCondition[i]=1;
                    cout<<"kinect 1st Sitted! : "<<ofToString(i)<<endl;
                }
                else if(chairCondition[i]==3){
                    chairCondition[i]=4;
                   // chairCondition[i]=6;
                    cout<<"kinect 2nd Sitted! : "<<ofToString(i)<<endl;
                }
            }
            else if(dis_stand[i]>110&&(chairCondition[i]==2||chairCondition[i]==4)){//立ったとみなす
                if(chairCondition[i]==2){
                    chairCondition[i]=3;
                    cout<<"kinect 1st Standed! : "<<ofToString(i)<<endl;
                }
                else if(chairCondition[i]==4){
                    chairCondition[i]=5;
                    //chairCondition[i]=0;
                    oscFlag[i]=0;
                    cout<<"kinect 2nd Standed! : "<<ofToString(i)<<endl;
                }
            }
            dis_sit[i]=0;
            dis_stand[i]=0;
        }
    }
}

void ofApp::oscEvent(){
    
}

void ofApp::sensor_check(int i){
    if(!bManual){
        int sensorVal = myLedEvent.sensorVal[i];
        sensorCheck_num[i]+=sensorVal;
        
        if(sCount<10)sCount++;
        else {
            sCount=0;
            int num=sensorCheck_num[i]/10;
            if(num>=120){//着席
                if(chairCondition[i]==0){
                    chairCondition[i]=1;
                    cout<<"SwitchSensor 1st Sitted :"<<ofToString(i)<<endl;
                }
                else if(chairCondition[i]==3){
                    chairCondition[i]=4;
                    cout<<"SwitchSensor 2nd Sitted :"<<ofToString(i)<<endl;
                }
            }
            else if(num<120){//離席
                if(chairCondition[i]==2){
                    chairCondition[i]=3;
                    cout<<"SwitchSensor 1st Standed :"<<ofToString(i)<<endl;
                }
                else if(chairCondition[i]==4){
                    chairCondition[i]=5;
                    cout<<"SwitchSensor 2nd Standed :"<<ofToString(i)<<endl;
                }
            }
            sensorCheck_num[i]=0;
        }
    }
}
