#include "simpleViewer.h"
#include <QtGUI/QMenu>
#include <QtGUI/QKeyEvent>
#include <QtGUI/QMouseEvent>
#include <QTCore/QMap>
#include <QtGUI/QCursor>

using namespace std;

void Viewer::draw()
{
    
    // Draw the first and fixed point cloud
    glBegin(GL_POINTS);
    for (int i=0; i<C.points.size(); i++){
        glColor3f(1.0, 0.0, 0.0);
        glVertex3f(C.points[i].x()*resizeFactor, C.points[i].y()*resizeFactor, C.points[i].z() *resizeFactor);

    }
    glEnd();

    // Draw the second point cloud, the one that moves
    C.tpoints.transform(C.drawPoints,PSolver::v2t(C.vector[C.counter]));
    
    glBegin(GL_POINTS);
    for (int i=0; i<C.drawPoints.size(); i++){
        glColor3f(0.0, 0.0, 1.0);
        glVertex3f(C.drawPoints[i].x()*resizeFactor, C.drawPoints[i].y()*resizeFactor, C.drawPoints[i].z()*resizeFactor);

    }
    glEnd();
   
    
    // Print text on screen 
    displayText();
}

void Viewer::init()
{
    // Disable Lighting, not needed with points
    glDisable(GL_LIGHTING);
    
    // Restore previous viewer state.
    restoreStateFromFile();
    
    // Set the point size
    glPointSize(3);
    

    // CREATE THE SCENE AND DOES ICP, DEMO CODE
    C.it = 100;
    C.counter = 0;
    Eigen::Vector3f v;
    v << 0.4,0.6,0.2;
    v.normalize();
    Eigen::Quaternionf q(Eigen::AngleAxisf(M_PI/4,v));
    q.normalize();
    Eigen::Quaternionf q2(Eigen::AngleAxisf(M_PI/4,Eigen::Vector3f::UnitY()));
    q2.normalize();
    
    drawCircle(C.points, Eigen::Vector3f(0,0,0), 10, q2, 100);
    drawSpiral(C.points,200);
    C.T.setIdentity();
    C.T.linear() = q.toRotationMatrix();
    Eigen::Vector3f V;
    V << 10 , 10 , 10;
    C.T.translation() = V;
    C.points.transform(C.tpoints, C.T.inverse());
    C.Tguess.setIdentity();
    icp(C.Tguess,C.points,C.tpoints,C.vector,C.errorVector,C.it);
    
    // DEMO CODE ENDS HERE
    
    
    
    // Press 'Esc' to exit application
    setShortcut(EXIT_VIEWER, Qt::Key_Escape);
    
    // Opens help window
    help();
}

// Draw the text in the 2D coordinates of the screen
void Viewer::displayText()
{
    qglColor(foregroundColor());

    
    Vector6f v = t2v(C.T);
    
    QString error = "ERROR: " + QString::number(C.errorVector[C.counter]);
    QString iter = "iteration: " + QString::number(C.counter+1);
    
    QString original =    "Original Transformation : tx:" +
    QString::number(v(0)) + "; ty: " +
    QString::number(v(1)) + "; tz: " +
    QString::number(v(2)) + "; Roll: " +
    QString::number(v(3)) + "; Pitch: " +
    QString::number(v(4)) + "; Yaw: " +
    QString::number(v(5)) + " " ;
    
    QString transformation =    "Current Transformation : tx:" +
    QString::number(C.vector[C.counter](0)) + "; ty: " +
    QString::number(C.vector[C.counter](1)) + "; tz: " +
    QString::number(C.vector[C.counter](2)) + "; Roll: " +
    QString::number(C.vector[C.counter](3)) + "; Pitch: " +
    QString::number(C.vector[C.counter](4)) + "; Yaw: " +
    QString::number(C.vector[C.counter](5)) + " " ;
    
    drawText(10,height()-30, transformation);
    drawText(10,height()-15, original);
    drawText(width()-220,height()-30, error);
    drawText(width()-220,height()-15, iter);


}


// Define the help screen, called once
QString Viewer::helpString() const
{
    QString text("<h2>I C P  3 D</h2>");
    text += "Press <b>left</b> and <b>right</b> arrow key to navigate through the iterations of the ICP<br><br>";
    text += "On the bottom left corner there is the original and current transformations in terms of ";
    text += "translation and rotation angles, while on the bottom right corner the euclidean error is shown <br><br>";
    
    text += "Press <b>left</b> and <b>right</b> arrow key to navigate through the iterations of the ICP";
    
    text += "Use the mouse to move the camera around the object. ";
    text += "You can respectively revolve around, zoom and translate with the three mouse buttons. ";
    text += "Left and middle buttons pressed together rotate around the camera view direction axis";
    
    text += "<b>Alt+Return</b> for full screen mode and <b>Control+S</b> to save a snapshot. ";
    text += "See the Keyboard tab in this window for a complete shortcut list.";
    text += "Double clicks automates single click actions: A left button double click aligns the closer axis with the camera (if close enough). ";
    text += "A middle button double click fits the zoom of the camera and the right button re-centers the scene.";
    text += "A left button double click while holding right button pressed defines the camera <i>Revolve Around Point</i>. <br><br>";
    
    text += "Press <b>Escape</b> to exit the viewer.";
    return text;
}


// Function which handles the user defined keyboard events
void Viewer::keyPressEvent(QKeyEvent *e)
{
    bool handled = false;
    if ((e->key()==Qt::Key_Right) )
    {
       
            if(C.counter+1<C.it)
                C.counter++;
            updateGL();
    }
    else
        if ((e->key()==Qt::Key_Left))
        {

            if(C.counter>0)
                C.counter--;
            updateGL();
        }

    if (!handled)
        QGLViewer::keyPressEvent(e);
}
