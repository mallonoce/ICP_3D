/**********************************************************/
// ICP 3D by Antonio Tammaro                              //
//                                                        //
// based on the simple_matcher of                         //
// Giorgio Grisetti                                       //
//                                                        //
// Dependencies:                                          //
// FLANN: http://www.cs.ubc.ca/research/flann/            //
// libqglviewer: http://libqglviewer.com/                 //
// Eigen: http://eigen.tuxfamily.org                      //
// openGL: https://www.opengl.org                         //
//                                                        //
// Note that the demo is created into the init() function //
// defined in simpleViewer. If you want to change the     //
// scene be sure to use the container defined in icp.h    //
/**********************************************************/

// TODOs:
// use quaternion to handle rotations
// rewrite the displayText() function to improve the quality of text

#include "icp.h"
#include <iostream>
#include <fstream>
#include "simpleViewer.h"
#include <QtGui/qapplication.h>


using namespace PSolver;


int main(int argc, char** argv){
   
     // Read command lines arguments.
     QApplication application(argc,argv);
     
     // Instantiate the viewer.
     Viewer viewer;
     viewer.setWindowTitle("ICP 3D");
     
     // Make the viewer window visible on screen.
     viewer.show();
     
     // Run main loop.
     return application.exec();

  return 0;
}

