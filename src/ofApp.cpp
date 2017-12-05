#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    cout << "--- INIT KINECT SENDER ---" << endl;
    ofBackground(10, 10, 10);

    kinect.open(0);

    camSetup();
    bgCamSetup();

    //origin
    originMarker.set(100, 100, 100);

    //tcp
    imageBytesSize = imageWidth * imageHeight;  //*3

    tcpCreateConnection();

    _frameId = 0;

    pointCloudFbo.allocate(imageWidth, imageHeight, GL_LUMINANCE);
    backgroundFbo.allocate(imageWidth, imageHeight, GL_RGBA);

    loadBackground();  //if the background mesh already exists

    loadGui();
    sphereSubtractor.setRadius(100);

    createFloorCube();
}

//--------------------------------------------------------------
void ofApp::update(){
    kinect.update();
    updateCam();
    sphereCentroid.set(sphereX, sphereY, sphereZ);

    if(kinect.isFrameNew()){        
        rawPCL = constructPointCloud(kinect.getRawDepthPixels());

        if(useBackgroundSubtraction){
//            finalPCL = removeBackground(rawPCL);
//            finalPCL = removeBackgroundDist(rawPCL);
//            finalPCL = sphericalSubtraction(rawPCL);
            finalPCL = floorPlaneSubtraction(rawPCL);
        }else{
            finalPCL = rawPCL;
        }


        pointCloudFbo = pclToNetworkFbo(finalPCL);
        tcpSendImage(pointCloudFbo);

    }


}

//--------------------------------------------------------------
void ofApp::draw(){
//    ofBackground(0, 0, 0);
    if(drawFbo){
        pointCloudFbo.draw(0, 0);
    }

    gui.draw();
    drawBackgroundPCL();



}


//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------

void ofApp::saveBackgroundAvg(){
    float startTime = ofGetElapsedTimeMillis();
    float endTime = bgTime;  // loop for 1 second
    float nextInterval = 3;
    float intervalTime = 3;
    bool timeReached = false;
    int frameCounter = 1;
    vector<ofMesh> pcl;

    while(!timeReached){
        float timer = ofGetElapsedTimeMillis() - startTime;

        if(timer >= endTime && !timeReached){
            cout << "time reached" << endl;
            timeReached = true;
        }else{
            cout << "Grabbing depth frame: " << frameCounter << endl;
            frameCounter++;
            ofMesh pointCloud = constructPointCloud(kinect.getRawDepthPixels());
            pcl.push_back(pointCloud);
            nextInterval += intervalTime;
        }
    }

    cout << "Calculating Average depth from " << pcl.size() << " pointclouds" << endl;

    ofMesh pclAvg;
    pclAvg.clear();
    pclAvg.setMode(OF_PRIMITIVE_POINTS);
    pointColor = ofColor(255, 255, 255);

    cout << "Fill with zeros" << endl;
    for(int y = 0; y < kinectHeight; y += sampleSize){
        for(int x = 0; x < kinectWidth; x += sampleSize){
            ofVec3f point = depthToPointCloudPos(x, y, 0);
            int index = (y * kinectWidth) + x;
            pclAvg.addVertex(point);
//            pclAvg.addIndex(index);
        }
    }

    cout << "blank cloud has points: " << pclAvg.getNumVertices() << endl;

    for(int y = 0; y < kinectHeight; y += sampleSize){
        for(int x = 0; x < kinectWidth; x += sampleSize){
//            cout << "got here" << endl;
            int index = (y * kinectWidth) + x;
            int totalDepth = 0;
            int emptyPoints = 0;

            for(int i = 0; i < pcl.size(); i++){
                ofMesh *mesh = &pcl[i];
                ofVec3f rawPoint = mesh->getVertex(index);

                if(rawPoint.z == 0){
                    emptyPoints++;
                }else{
                    totalDepth += rawPoint.z;
                }
            }

            int remain = pcl.size() - emptyPoints;
            if(remain <= 0){
                //skip
            }else{

                float avgDepth = totalDepth / (pcl.size() - emptyPoints);

                ofVec3f avgPoint = pcl[0].getVertex(index);
                avgPoint.z = avgDepth;

                pclAvg.setVertex(index, avgPoint);
            }
        }
    }

    pclAvg.save(backgroundMeshName);

    cout << "Background mesh saved as: '" << backgroundMeshName << ".ply'" << endl;
    loadBackground();
}


void ofApp::saveBackground(){
    cout << "Save Background!" << endl;
    ofMesh pcl = constructPointCloud(kinect.getRawDepthPixels());
    pcl.save(backgroundMeshName);
    cout << "Background data has been saved to: '" << backgroundMeshName << ".ply'" << endl;
    loadBackground();
}

void ofApp::loadBackground(){
    ofFile file;
    bool fileExists = file.doesFileExist(backgroundMeshName, true);

    if(fileExists){
        backgroundPCL.load(backgroundMeshName);
        cout << "Background mesh has been loaded" << endl;
        backgroundLoaded = true;
    }else{
        cout << "Background file does not exist yet!" << endl;
        backgroundLoaded = false;
    }

}

void ofApp::saveSettings(){
    cout << "Saving Settings..." << endl;

    ofxXmlSettings settings;

    settings.setValue("settings:matrixTransformationX", mTranX);
    settings.setValue("settings:matrixTransformationY", mTranY);
    settings.setValue("settings:matrixTransformationZ", mTranZ);

    settings.setValue("settings:matrixRotationX", mRotX);
    settings.setValue("settings:matrixRotationY", mRotY);
    settings.setValue("settings:matrixRotationZ", mRotZ);

    settings.setValue("settings:cameraRoll", camRoll);
    settings.setValue("settings:floorThreshold", floorThreshold);

    settings.saveFile("settings.xml");

    cout << "Settings saved to 'settings.xml'" << endl;
}

void ofApp::loadSettings(){
    cout << "Load Saved Settings..." << endl;
    ofxXmlSettings settings;

    settings.loadFile("settings.xml");
    mTranX = settings.getValue("settings:matrixTransformationX", 0);
    mTranY = settings.getValue("settings:matrixTransformationY", 0);
    mTranZ = settings.getValue("settings:matrixTransformationZ", 0);
    mRotX = settings.getValue("settings:matrixRotationX", 0);
    mRotY = settings.getValue("settings:matrixRotationY", 0);
    mRotZ = settings.getValue("settings:matrixRotationZ", 0);
    camRoll = settings.getValue("settings:cameraRoll", 0);
    floorThreshold = settings.getValue("settings:floorThreshold", 0);
    cout << "Settings have been loaded" << endl;
}

void ofApp::loadGui(){
    gui.setup();

//    saveBackgroundBtn.addListener(this, &ofApp::saveBackground);
    saveBackgroundBtn.addListener(this, &ofApp::saveBackgroundAvg);
//    saveBackgroundBtn.addListener(this, &ofApp::saveBackgroundFill);

    gui.add(saveBackgroundBtn.setup("Save Background"));

    loadBackgroundBtn.addListener(this, &ofApp::loadBackground);
    gui.add(loadBackgroundBtn.setup("Load Background Mesh"));

    gui.add(useBackgroundSubtraction.setup("Background Subtraction", true));

    gui.add(bgDepthRange.setup("BG Range", 0.0, 0.0, bgDepthRangeMax));

    gui.add(sphereX.setup("Sphere X", 0, spherePosMin, spherePosMax));
    gui.add(sphereY.setup("Sphere Y", 0, spherePosMin, spherePosMax));
    gui.add(sphereZ.setup("Sphere Z", 0, spherePosMin, spherePosMax));

    gui.add(mTranX.setup("mTranX", tX, spherePosMin, spherePosMax));
    gui.add(mTranY.setup("mTranY", tY, spherePosMin, spherePosMax));
    gui.add(mTranZ.setup("mTranZ", tZ, spherePosMin, spherePosMax));
    gui.add(mRotX.setup("mRotX", rX, 0, rotMax));
    gui.add(mRotY.setup("mRotY", rY, 0, rotMax));
    gui.add(mRotZ.setup("mRotZ", rZ, 0, rotMax));

    gui.add(floorThreshold.setup("FloorThresh", floorVal, floorMin, floorMax));
    gui.add(camRoll.setup("camRoll", rollVal, rollMin, rollMax));

    saveSettingsBtn.addListener(this, &ofApp::saveSettings);
    gui.add(saveSettingsBtn.setup("Save Settings"));
    loadSettingsBtn.addListener(this, &ofApp::loadSettings);
    gui.add(loadSettingsBtn.setup("Load Saved Settings"));
}


void ofApp::drawBackgroundPCL(){
    backgroundFbo.begin();
    ofClear(255, 255, 255);
    ofBackground(25, 25, 0);
    bgCam.begin();
    ofScale(1, 1, 1);
    glPointSize(pointSize);
    backgroundPCL.drawVertices();
    bgCam.end();
    backgroundFbo.end();

    backgroundFbo.draw(600, 150);
}

void ofApp::camSetup(){
    //camera settings
    cam.setAutoDistance(false);
    cam.setFov(fov);
    cam.setNearClip(nearClip);
    cam.setFarClip(farClip);
    camPosition = ofVec3f(camX, camY, camZ);
    cam.setPosition(camPosition);
    cam.tilt(camTilt);
    cam.pan(camPan);
    cam.roll(rollVal);
    cam.setVFlip(true);
}

void ofApp::updateCam(){
    cam.roll(camRoll);
}

void ofApp::bgCamSetup(){
    //camera settings
    bgCam.setAutoDistance(false);
    bgCam.setFov(fov);
    bgCam.setNearClip(nearClip);
    bgCam.setFarClip(farClip);
    ofVec3f bgCamPosition = ofVec3f(camX, camY, camZ);
    bgCam.setPosition(camPosition);
    bgCam.tilt(camTilt);
    bgCam.pan(camPan);
    bgCam.roll(camRoll);
    bgCam.setVFlip(true);
}

string ofApp::getHostname(){
    FILE* stream = popen("hostname", "r");
    ostringstream output;

    while(!feof(stream) && !ferror(stream)){
        char buf[128];
        int bytesRead = fread(buf, 1, 128, stream);
        output.write(buf, bytesRead);
    }

    string resultRaw = output.str();
    vector<string> resultSplit = ofSplitString(resultRaw, "\n");
    string result = resultSplit[0];

    cout << "Current Hostname: " << result << endl;

    return result;
}

void ofApp::tcpCreateConnection(){
    string address;
    int port;

    string hostname = getHostname();

    if(hostname == "node0"){
        address = tcpAddress_localhost;
        port = tcpPort_0;
    }else{
        address = tcpAddress;
    }

    if(hostname == "node1"){
        port = tcpPort_1;
    }
    if(hostname == "node2"){
        port = tcpPort_2;
    }
    if(hostname == "node3"){
        port = tcpPort_3;
    }
    if(hostname == "node4"){
        port = tcpPort_4;
    }

    cout << hostname << " connecting to: " << address << " on port: " << port << endl;

    tcpSender.setup(address, port, packetSize);
}


void ofApp::tcpSendImage(const ofFbo &fbo){
    ofPixels _pixels;
    fbo.readToPixels(_pixels);
    unsigned char * pixels = _pixels.getPixels();
    tcpSender.send(pixels, imageBytesSize, _frameId++);
}

//-------------------------------------------------------
ofFbo ofApp::pclToNetworkFbo(const ofMesh & pcl){    

    ofFbo fbo;

    fbo.allocate(imageWidth, imageHeight, GL_LUMINANCE); //this could be optimized

    fbo.begin();
    ofClear(255, 255, 255);
    ofBackground(25, 0, 0);
    cam.begin();
    ofScale(1, 1, 1);
    glPointSize(pointSize);
    pcl.drawVertices();

//    sphereSubtractor.setPosition(sphereX, sphereY, sphereZ);
//    sphereSubtractor.draw();

//    floorCube.draw();

    cam.end();
    fbo.end();

    return fbo;
}


ofMesh ofApp::removeBackground(const ofMesh & pcl){
    ofMesh newPCL;
    newPCL.clear();
    newPCL.setMode(OF_PRIMITIVE_POINTS);

    //if the background mesh doesn't exist, return
    if(!backgroundLoaded){
        cout << "cant load background" << endl;
        useBackgroundSubtraction = false;
        return pcl;
    }

//    cout << "Fill with zeros" << endl;
    for(int y = 0; y < kinectHeight; y += sampleSize){
        for(int x = 0; x < kinectWidth; x += sampleSize){
            ofVec3f point = depthToPointCloudPos(x, y, 0);
            newPCL.addVertex(point);
        }
    }

    for(int y = 0; y < kinectHeight; y += sampleSize){
        for(int x = 0; x < kinectWidth; x += sampleSize){
            int index = (y * kinectWidth) + x;
            ofVec3f rawPoint = pcl.getVertex(index);
            ofVec3f bgPoint = backgroundPCL.getVertex(index);

//            if(rawPoint.z > bgPoint.z - bgDepthRange && rawPoint.z < bgPoint.z + bgDepthRange
//                || rawPoint.z == 0 ){


            if(rawPoint.z > bgPoint.z - bgDepthRange && rawPoint.z < bgPoint.z + bgDepthRange){
                //do nothing
            }else{
//                cout << "kept: " << endl;
//                cout << "rawPoint.z: " << rawPoint.z << endl;
//                cout << "bgPoint.z: " << bgPoint.z << endl;
//                newPCL.addColor(pointColor);
//                newPCL.addVertex(rawPoint);
                newPCL.setVertex(index, rawPoint);
            }

        }
    }

    return newPCL;
}



void ofApp::createFloorCube(){
    ofVec3f vertices[] = {
        ofVec3f(2700, 960, 4500),
        ofVec3f(-2200, 400, 4100),
        ofVec3f(1700, 2000, 2475),
        ofVec3f(-2200, 1300, 2500),
//        ofVec3f(1600, 2000, 2400),
//        ofVec3f(-100, 100, 100),
//        ofVec3f(-100, -100, 100),
//        ofVec3f(-100, -100, -100)
    };

    floorCube.addVertices(vertices, 4);

    static ofIndexType indices[] = {
        0, 1, 2,
        0, 2, 3,
        4, 5, 6,
        4, 6, 7,

    };

    floorCube.addIndices(indices, 12);

}

ofMesh ofApp::floorPlaneSubtraction(const ofMesh & pcl){
    ofMesh newPCL;
    newPCL.clear();
    newPCL.setMode(OF_PRIMITIVE_POINTS);

    for(int y = 0; y < kinectHeight; y += sampleSize){
        for(int x = 0; x < kinectWidth; x += sampleSize){
            ofVec3f point = depthToPointCloudPos(x, y, 0);
            newPCL.addVertex(point);
        }
    }

    ofMatrix4x4 matrix;
    matrix.glTranslate(mTranX, mTranY, mTranZ);
    ofQuaternion yRot(mRotY, ofVec3f(0, 1, 0));
    ofQuaternion xRot(mRotX, ofVec3f(-1, 0, 0));
    ofQuaternion zRot(mRotZ, ofVec3f(0, 0, 1));
    matrix.glRotate(yRot);
    matrix.glRotate(xRot);
    matrix.glRotate(zRot);

    for(int y = 0; y < kinectHeight; y += sampleSize){
        for(int x = 0; x < kinectWidth; x += sampleSize){
            int index = (y * kinectWidth) + x;
            ofVec3f rawPoint = pcl.getVertex(index);
//            float dist = rawPoint.distance(sphereCentroid);
            rawPoint = rawPoint * matrix;
            if(rawPoint.z <= floorThreshold){
                //omit
            }else{
//                cout << rawPoint.z << endl;
                newPCL.setVertex(index, rawPoint);
            }


        }
    }

    return newPCL;
}

ofMesh ofApp::sphericalSubtraction(const ofMesh & pcl){
    ofMesh newPCL;
    newPCL.clear();
    newPCL.setMode(OF_PRIMITIVE_POINTS);

//    sphereCentroid.set(sphereX, sphereY, sphereZ);

//    floorCube.setVertex(1, sphereCentroid);

    for(int y = 0; y < kinectHeight; y += sampleSize){
        for(int x = 0; x < kinectWidth; x += sampleSize){
            ofVec3f point = depthToPointCloudPos(x, y, 0);
            newPCL.addVertex(point);
        }
    }

    ofMatrix4x4 matrix;
    matrix.glTranslate(mTranX, mTranY, mTranZ);
    ofQuaternion yRot(mRotY, ofVec3f(0, 1, 0));
    ofQuaternion xRot(mRotX, ofVec3f(-1, 0, 0));
    ofQuaternion zRot(mRotZ, ofVec3f(0, 0, 1));
    matrix.glRotate(yRot);
    matrix.glRotate(xRot);
    matrix.glRotate(zRot);

    for(int y = 0; y < kinectHeight; y += sampleSize){
        for(int x = 0; x < kinectWidth; x += sampleSize){
            int index = (y * kinectWidth) + x;
            ofVec3f rawPoint = pcl.getVertex(index);
//            float dist = rawPoint.distance(sphereCentroid);
            bool omit = false;
            for(int k = 0; k < floorCube.getNumVertices(); k++){
                ofVec3f floorVertex = floorCube.getVertex(k);

                float dist = rawPoint.distance(floorVertex);

                if(dist <= bgDepthRange){
                    omit = true;
                }
            }

            if(!omit){
                rawPoint = rawPoint * matrix;
                newPCL.setVertex(index, rawPoint);
            }

        }
    }

    return newPCL;
}

ofMesh ofApp::removeBackgroundDist(const ofMesh & pcl){
    ofMesh newPCL;
    newPCL.clear();
    newPCL.setMode(OF_PRIMITIVE_POINTS);

    //if the background mesh doesn't exist, return
    if(!backgroundLoaded){
        cout << "cant load background" << endl;
        useBackgroundSubtraction = false;
        return pcl;
    }

    for(int y = 0; y < kinectHeight; y += sampleSize){
        for(int x = 0; x < kinectWidth; x += sampleSize){
            ofVec3f point = depthToPointCloudPos(x, y, 0);
            newPCL.addVertex(point);
        }
    }

    for(int y = 0; y < kinectHeight; y += sampleSize){
        for(int x = 0; x < kinectWidth; x += sampleSize){
            int index = (y * kinectWidth) + x;
            ofVec3f rawPoint = pcl.getVertex(index);
            ofVec3f bgPoint = backgroundPCL.getVertex(index);

            ofVec3f distVec = rawPoint - bgPoint;

            if(abs(distVec.x) < bgDepthRange || abs(distVec.y) < bgDepthRange || abs(distVec.z) < bgDepthRange){
                //skip it
            }else{
//                if(distVec.z > 10000 || distVec.z < 10){
//                    //skip
//                }else{
//                    newPCL.setVertex(index, rawPoint);
//                }
                newPCL.setVertex(index, rawPoint);
            }
//            if(rawPoint.z == 0){
//                //skip dont add it to our cloud
//            }else{

//                float dist = rawPoint.distance(bgPoint);
//                if(dist < bgDepthRange){
//                    //skip dont add it to our cloud
//                }else{
//                    cout << dist << endl;
//                    newPCL.setVertex(index, rawPoint);
//                }
//            }
        }
    }

    return newPCL;
}

//------------------------------------------------------
ofMesh ofApp::constructPointCloud(const ofFloatPixels & depth){
    ofMesh pcl;
    if(depth.size() > 0){
        pcl.clear();
        pcl.setMode(OF_PRIMITIVE_POINTS);

        for(int y = 0; y < kinectHeight; y += sampleSize){
            for(int x = 0; x < kinectWidth; x += sampleSize){
                int index = (y * kinectWidth) + x;
                int depthMeasurement= depth[index];

                ofVec3f point = depthToPointCloudPos(x, y, depthMeasurement);
//                pointColor = ofColor(255, 255, 255);
                pcl.addVertex(point);
            }
        }
    }

    return pcl;
}





//------------------------------------------------------
ofVec3f ofApp::depthToPointCloudPos(int x, int y, float depthValue){
    static float cx = 254.878f;
    static float cy = 205.395f;
    static float fx = 365.456f;
    static float fy = 365.456f;
    static float k1 = 0.0905474;
    static float k2 = -0.26819;
    static float k3 = 0.0950862;
    static float p1 = 0.0;
    static float p2 = 0.0;

    ofVec3f point;

    point.z = (depthValue);// / (1.0f); // Convert from mm to meters
    point.x = (x - cx) * point.z / fx;
    point.y = (y - cy) * point.z / fy;
    return point;
}



//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    saveBackground();
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){

}
