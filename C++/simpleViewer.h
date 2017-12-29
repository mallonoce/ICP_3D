#include <QGLViewer/qglviewer.h>
#include "point3f_vector.h"
#include "icp.h"
#include "defs.h"

using namespace PSolver;

class Viewer : public QGLViewer
{
    

    
    protected :
    virtual void draw();
    virtual void init();
    virtual void keyPressEvent(QKeyEvent *e);
    void displayText();
    virtual QString helpString() const;
    
    
    private:
    container C; // struct to handle all the ICP variables
    float resizeFactor = 0.1; // resize factor if the clouds are too big

    
};

