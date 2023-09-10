#ifndef FRAME_H
#define FRAME_H

#include "Settings.h"
#include "Camera.h"
#include "Map.h"
#include "pangolin/pangolin.h"
#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>
#include <pangolin/scene/renderable.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/geometry/glgeometry.h>
#include <Eigen/Core>


namespace DC_VSLAM
{

class ViewFrame
{
    private:

    public:
        // if cntrl+c is pressed request stop of the thread
        bool stopRequested {false};
    
        // constructor
        ViewFrame();
        // visualization function
        void pangoQuit(Zed_Camera* zedPtr, const Map* _map);                    
        // visualization function for Dual Camera Setup
        void pangoQuitMulti(Zed_Camera* zedPtr, Zed_Camera* zedPtrB, const Map* _map);                    


};

struct CameraFrame
{
    const float cameraWidth = 0.0575f;

    Zed_Camera* zedCamera;
    Zed_Camera* zedCameraB = nullptr;

    const Map* map;

    void drawCameraFrame();
    void lineFromKeyFrameToCamera();
    void drawCamera();
    void drawBackCamera();
    void drawPoints();
    void drawKeyFrames();
    void getOpenGLMatrix(pangolin::OpenGlMatrix &MOw);
};


} //namespace DC_VSLAM

#endif // FRAME_H