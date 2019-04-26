#!/usr/bin/env python

## BEGIN_TUTORIAL
##
## Imports
## ^^^^^^^
##
## First we start with the standard ros Python import line:
import roslib; roslib.load_manifest('rviz_python_tutorial')
from PyQt5.QtWidgets import QApplication, QWidget,QVBoxLayout,QSlider,QHBoxLayout,QPushButton,QLabel,QLineEdit

## Then load sys to get sys.argv.
import sys

## Next import all the Qt bindings into the current namespace, for
## convenience.  This uses the "python_qt_binding" package which hides
## differences between PyQt and PySide, and works if at least one of
## the two is installed.  The RViz Python bindings use
## python_qt_binding internally, so you should use it here as well.
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

## Finally import the RViz bindings themselves.
import rviz

## The MyViz class is the main container widget.
class MyViz( QWidget ):

    ## MyViz Constructor
    ## ^^^^^^^^^^^^^^^^^
    ##
    ## Its constructor creates and configures all the component widgets:
    ## frame, thickness_slider, top_button, and side_button, and adds them
    ## to layouts.
    def __init__(self):
        QWidget.__init__(self)

        ## rviz.VisualizationFrame is the main container widget of the
        ## regular RViz application, with menus, a toolbar, a status
        ## bar, and many docked subpanels.  In this example, we
        ## disable everything so that the only thing visible is the 3D
        ## render window.
        self.frame = rviz.VisualizationFrame()

        ## The "splash path" is the full path of an image file which
        ## gets shown during loading.  Setting it to the empty string
        ## suppresses that behavior.
        self.frame.setSplashPath( "" )

        ## VisualizationFrame.initialize() must be called before
        ## VisualizationFrame.load().  In fact it must be called
        ## before most interactions with RViz classes because it
        ## instantiates and initializes the VisualizationManager,
        ## which is the central class of RViz.
        self.frame.initialize()

        ## The reader reads config file data into the config object.
        ## VisualizationFrame reads its data from the config object.
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile( config, "rviz_navigation.rviz" )
        self.frame.load( config )

        ## You can also store any other application data you like in
        ## the config object.  Here we read the window title from the
        ## map key called "Title", which has been added by hand to the
        ## config file.
        self.setWindowTitle( config.mapGetChild( "Title" ).getValue() )

        ## Here we disable the menu bar (from the top), status bar
        ## (from the bottom), and the "hide-docks" buttons, which are
        ## the tall skinny buttons on the left and right sides of the
        ## main render window.
        self.frame.setMenuBar( None )
        self.frame.setStatusBar( None )
        self.frame.setHideButtonVisibility( False )

        ## frame.getManager() returns the VisualizationManager
        ## instance, which is a very central class.  It has pointers
        ## to other manager objects and is generally required to make
        ## any changes in an rviz instance.
        self.manager = self.frame.getManager()

        ## Since the config file is part of the source code for this
        ## example, we know that the first display in the list is the
        ## grid we want to control.  Here we just save a reference to
        ## it for later.
        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt( 0 )
        
        ## Here we create the layout and other widgets in the usual Qt way.
        layout = QVBoxLayout()
        layout.addWidget( self.frame )
        l1 = QLabel()
	l1.setText("Set Aco Variables ")
	l1.setAlignment(Qt.AlignLeft)
	layout.addWidget(l1)
	layout.addStretch()
	l2 = QLabel()
	l2.setText("Number Of Iterations") #3000
 	l2.setAlignment(Qt.AlignLeft)
	layout.addWidget(l2)
	layout.addStretch()
	txt2 = QLineEdit()
	txt2.setAlignment(Qt.AlignLeft)
	txt2.setValidator(QIntValidator())
	txt2.setText("3000")
	layout.addWidget(txt2)
	layout.addStretch()
	l3 = QLabel()
	l3.setText(" m ")   #20
	l3.setAlignment(Qt.AlignLeft)
	layout.addWidget(l3)
	layout.addStretch()
	txt3 = QLineEdit()
	txt3.setAlignment(Qt.AlignLeft)
	txt3.setValidator(QDoubleValidator())
	txt3.setText("20")
	layout.addWidget(txt3)
	layout.addStretch()
	l4 = QLabel()
	l4.setText(" q0 ") #0.9
	l4.setAlignment(Qt.AlignLeft)
	layout.addWidget(l4)
	layout.addStretch()
	txt4 = QLineEdit()
	txt4.setAlignment(Qt.AlignLeft)
	txt4.setValidator(QDoubleValidator())
	txt4.setText("0.9")
	layout.addWidget(txt4)
	layout.addStretch()
	l5 = QLabel()
	l5.setText(" b ")  #2
	l5.setAlignment(Qt.AlignLeft)
	layout.addWidget(l5)
	layout.addStretch()
	txt5 = QLineEdit()
	txt5.setAlignment(Qt.AlignLeft)
	txt5.setValidator(QDoubleValidator())
	txt5.setText("2")
	layout.addWidget(txt5)
	layout.addStretch()
	l6 = QLabel()
	l6.setText(" r ") #0.1
	l6.setAlignment(Qt.AlignLeft)
	layout.addWidget(l6)
	layout.addStretch()
	txt6 = QLineEdit()
	txt6.setAlignment(Qt.AlignLeft)
	txt6.setValidator(QDoubleValidator())
	txt6.setText("0.1")
	layout.addWidget(txt6)
	layout.addStretch()
	l7 = QLabel()
	l7.setText(" x ") #0.1
	l7.setAlignment(Qt.AlignLeft)
	layout.addWidget(l7)
	layout.addStretch()
	txt7 = QLineEdit()
	txt7.setAlignment(Qt.AlignLeft)
	txt7.setValidator(QDoubleValidator())
	txt7.setText("0.1")
	layout.addWidget(txt7)
	layout.addStretch()
	l8 = QLabel()
	l8.setText(" a ") #1
	l8.setAlignment(Qt.AlignLeft)
	layout.addWidget(l8)
	layout.addStretch()
	txt8 = QLineEdit()
	txt8.setAlignment(Qt.AlignLeft)
	txt8.setValidator(QDoubleValidator())
	txt8.setText("1")
	layout.addWidget(txt8)
	layout.addStretch()	
	#txt8.editingFinished.connect(enterPress)
        thickness_slider = QSlider( Qt.Horizontal )
        thickness_slider.setTracking( True )
        thickness_slider.setMinimum( 1 )
        thickness_slider.setMaximum( 1000 )
        thickness_slider.valueChanged.connect( self.onThicknessSliderChanged )
        layout.addWidget( thickness_slider )
        
        h_layout = QHBoxLayout()
        
        top_button = QPushButton( "Top View" )
        top_button.clicked.connect( self.onTopButtonClick )
        h_layout.addWidget( top_button )
        
        side_button = QPushButton( "Side View" )
        side_button.clicked.connect( self.onSideButtonClick )
        h_layout.addWidget( side_button )
        
        layout.addLayout( h_layout )
        
        self.setLayout( layout )

    ## Handle GUI events
    ## ^^^^^^^^^^^^^^^^^
    ##
    ## After the constructor, for this example the class just needs to
    ## respond to GUI events.  Here is the slider callback.
    ## rviz.Display is a subclass of rviz.Property.  Each Property can
    ## have sub-properties, forming a tree.  To change a Property of a
    ## Display, use the subProp() function to walk down the tree to
    ## find the child you need.
    def onThicknessSliderChanged( self, new_value ):
        if self.grid_display != None:
            self.grid_display.subProp( "Line Style" ).subProp( "Line Width" ).setValue( new_value / 1000.0 )

    ## The view buttons just call switchToView() with the name of a saved view.
    def onTopButtonClick( self ):
        self.switchToView( "Top View" );
        
    def onSideButtonClick( self ):
        self.switchToView( "Side View" );
        
    ## switchToView() works by looping over the views saved in the
    ## ViewManager and looking for one with a matching name.
    ##
    ## view_man.setCurrentFrom() takes the saved view
    ## instance and copies it to set the current view
    ## controller.
    def switchToView( self, view_name ):
        view_man = self.manager.getViewManager()
        for i in range( view_man.getNumViews() ):
            if view_man.getViewAt( i ).getName() == view_name:
                view_man.setCurrentFrom( view_man.getViewAt( i ))
                return
        print( "Did not find view named %s." % view_name )

## Start the Application
## ^^^^^^^^^^^^^^^^^^^^^
##
## That is just about it.  All that's left is the standard Qt
## top-level application code: create a QApplication, instantiate our
## class, and start Qt's main event loop (app.exec_()).
if __name__ == '__main__':
    app = QApplication( sys.argv )

    myviz = MyViz()
    myviz.resize( 500, 500 )
    myviz.show()

app.exec_()
