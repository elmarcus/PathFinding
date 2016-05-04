#!/usr/bin/python

import sys, rospy, math
from PyQt4 import QtGui, QtCore
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import ColorRGBA, Float32, Bool
from apriltags_intrude_detector.srv import apriltags_intrude
from apriltags_intrude_detector.srv import apriltags_info

class PotentialField:
    
	def __init__(self, points):
		self.angle=0.0
		self.defaultAngle=0.0
		self.points = points
		xSum = 0.0
		ySum = 0.0		
		for point in points:
			xSum += point.x	
			ySum += point.y
		xSum /= 4.0
		ySum /= 4.0
		self.centerPoint = {'x': xSum, 'y': ySum}
		self.power = 0
		self.radius = math.sqrt(math.pow(points[0].x - self.centerPoint['x'], 2) + math.pow(points[0].y - self.centerPoint['y'], 2) )

	def getDistanceTo(self, robotLocation):
		return math.sqrt( math.pow(robotLocation.x - self.centerPoint['x'], 2) + math.pow(robotLocation.y - self.centerPoint['y'], 2) )

	def getAngleTo(self, robotLocation):
		return math.atan2(float(-1*(self.centerPoint['y'] - robotLocation.y)), float(self.centerPoint['x'] - robotLocation.x))
        
class AttractiveField(PotentialField):

	def __init__(self, points):
		print "in the constructor"
		
		PotentialField.__init__(self, points)
		print " called base constructor"
		self.alpha = 1.5
		
    
	def getVelocityChange(self, robotLocation):
		d = PotentialField.getDistanceTo(self, robotLocation)
		if (d < self.radius):
			print "inside goal"
			print self.radius
			
			return {'x': 0, 'y': 0}
		else:
			self.angle += PotentialField.getAngleTo(self, robotLocation)
			deltaX = self.alpha*(d - self.radius)*math.cos(self.angle)
			deltaY = self.alpha*(d - self.radius)*math.sin(self.angle)
			self.angle=self.defaultAngle
			return {'x': deltaX, 'y': deltaY}
            
    
class RepulsiveField(PotentialField):	
    	 	
	def __init__(self, points):
		PotentialField.__init__(self,points)
		self.beta = 1.0
		self.s = 60.0
		self.bigNum = 9001.0
		self.radius*=1.5
        
	def getVelocityChange(self, robotLocation):
		d = PotentialField.getDistanceTo(self, robotLocation)
		self.angle += PotentialField.getAngleTo(self, robotLocation)
		deltaX = 0
		deltaY = 0
		if (d < self.radius):
			print "inside repulsive field"
			print self.radius
			deltaX = -1*(math.cos(self.angle)/math.fabs(math.cos(self.angle)))*self.bigNum
			deltaY = -1*(math.sin(self.angle)/math.fabs(math.sin(self.angle)))*self.bigNum
		elif (self.radius <= d and d <= self.s + self.radius):
			deltaX = -1*self.beta*(self.s + self.radius - d)*math.cos(self.angle)
			deltaX = -1*self.beta*(self.s + self.radius - d)*math.sin(self.angle)
		self.angle = self.defaultAngle
		return {'x': deltaX, 'y': deltaY}

class TangentalField(RepulsiveField):
	def __init__(self, points):
		RepulsiveField.__init__(self, points)
		self.defaultAngle = math.pi/2
		self.angle+=self.defaultAngle

# You implement this class
class Controller:
    stop = True # This is set to true when the stop button is pressed
    fields = []
	
    def __init__(self):
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.trackposSub = rospy.Subscriber("tracked_pos", Pose2D, self.trackposCallback)

    def getVelocityChange(self, robotLocation):
        deltaX = 0
        deltaY = 0
        for field in self.fields:
            delta = field.getVelocityChange(robotLocation)
            #print "Individual field return: " + str(delta)
            deltaX += delta['x']
            deltaY += delta['y']
        return {'x': deltaX, 'y': deltaY}

    def trackposCallback(self, msg):
        # This function is continuously called
        if not self.stop:
            twist = Twist()
            vel = self.getVelocityChange(msg)
            #print "Velocity: " + str(vel)
            # Change twist.linear.x to be your desired x velocity
            twist.linear.x = vel['x']
            # Change twist.linear.y to be your desired y velocity
            twist.linear.y = vel['y']
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            self.cmdVelPub.publish(twist)

    def start(self):
        rospy.wait_for_service("apriltags_info")
        try:
            info_query = rospy.ServiceProxy("apriltags_info", apriltags_info)
            resp = info_query()

            for i in range(len(resp.polygons)):
                # A polygon (access points using poly.points)
                poly = resp.polygons[i]
                # The polygon's id (just an integer, 0 is goal, all else is bad)
                t_id = resp.ids[i]
                print "ind of polygons" + str(i)
                if (int(t_id) == 0): 
                    print "calling attr field constructor"   
                    self.fields.append(AttractiveField(poly.points))
                elif (int(t_id) == 2):
                    self.fields.append(TangentalField(poly.points))
                else:
                    self.fields.append(RepulsiveField(poly.points))
        except Exception, e:
            print "Exception: " + str(e)
        finally:
            self.stop = False

    def stop(self):
        self.stop = True


class SpheroIntrudeForm(QtGui.QWidget):
    controller = Controller()
    
    def __init__(self):
        super(QtGui.QWidget, self).__init__()
        self.resize(600, 480) 
        self.initUI()

        rospy.init_node('sphero_intrude', anonymous=True)
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.cmdVelSub = rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback) 

        self.trackposSub = rospy.Subscriber("tracked_pos", Pose2D, self.trackposCallback) 
       
    def initUI(self):

        self.stateLabel = QtGui.QLabel("Position")
        self.stateTextbox = QtGui.QTextEdit()
        self.stateTextbox.setReadOnly(True)
        self.connect(self, QtCore.SIGNAL("sendPosIDText(PyQt_PyObject)"), self.updateStateTextbot)     
        
        key_instruct_label = """
	Control Your Sphero!
	---------------------------
	Moving around:
	   u    i    o
	   j    k    l
	   m    ,    .
	"""
        self.keyInstructLabel = QtGui.QLabel(key_instruct_label)
        self.cmdVelLabel = QtGui.QLabel("cmd_vel")
        self.cmdVelTextbox = QtGui.QTextEdit()
        self.cmdVelTextbox.setReadOnly(True)  
        self.connect(self, QtCore.SIGNAL("sendCmdVelText(PyQt_PyObject)"), self.updateCmdVelTextbox)

        self.aprilTagsInfoLabel = QtGui.QLabel("april tags info")
        self.aprilTagsInfoBtn = QtGui.QPushButton("Query")
        self.aprilTagsInfoBtn.clicked.connect(self.queryAprilTagsInfo)
        self.aprilTagsTextbox = QtGui.QTextEdit()
        self.aprilTagsTextbox.setReadOnly(True)
        self.connect(self, QtCore.SIGNAL("sendTagInfoText(PyQt_PyObject)"), self.updateAprilTagsTextbot)

        self.aprilTagsStartBtn = QtGui.QPushButton("Start")
        self.aprilTagsStartBtn.clicked.connect(self.controller.start)

        self.aprilTagsStopBtn = QtGui.QPushButton("Stop")
        self.aprilTagsStopBtn.clicked.connect(self.controller.stop)


        self.layout =  QtGui.QVBoxLayout()
        self.layout.addWidget(self.stateLabel)
        self.layout.addWidget(self.stateTextbox)
        self.layout.addWidget(self.keyInstructLabel)
        self.layout.addWidget(self.cmdVelLabel)
        self.layout.addWidget(self.cmdVelTextbox)
        hlayout = QtGui.QHBoxLayout()
        hlayout.addWidget(self.aprilTagsInfoLabel)
        hlayout.addWidget(self.aprilTagsInfoBtn)
        hlayout.addWidget(self.aprilTagsStartBtn)
        hlayout.addWidget(self.aprilTagsStopBtn)
        self.layout.addLayout(hlayout)
        self.layout.addWidget(self.aprilTagsTextbox)
        self.setLayout(self.layout)

        self.setWindowTitle("Sphero Intrude")
        self.show()

    def keyPressEvent(self, e): 
        twist = None       
        if e.key() == QtCore.Qt.Key_U:
            twist = Twist()
            twist.linear.x = -80; twist.linear.y = 80; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_I:
            twist = Twist()  
            twist.linear.x = 0; twist.linear.y = 80; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0     
        elif e.key() == QtCore.Qt.Key_O:
            twist = Twist()
            twist.linear.x = 80; twist.linear.y = 80; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_J:
            twist = Twist()
            twist.linear.x = -80; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_K:
            twist = Twist()
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_L:
            twist = Twist()
            twist.linear.x = 80; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_M:
            twist = Twist()
            twist.linear.x = -80; twist.linear.y = -80; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_Comma:
            twist = Twist()
            twist.linear.x = 0; twist.linear.y = -80; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_Period:
            twist = Twist()
            twist.linear.x = 80; twist.linear.y = -80; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0 
        if twist != None:
            self.cmdVelPub.publish(twist)

    def cmdVelCallback(self, msg):
        cmd_vel_text = "x=" + str(msg.linear.x) + " y=" + str(msg.linear.y)
        self.emit(QtCore.SIGNAL("sendCmdVelText(PyQt_PyObject)"), cmd_vel_text) 

    def updateCmdVelTextbox(self, value):
        self.cmdVelTextbox.moveCursor(QtGui.QTextCursor.End)
        self.cmdVelTextbox.ensureCursorVisible()
        self.cmdVelTextbox.append(str(value))
        self.cmdVelTextbox.update()

    def trackposCallback(self, msg):
        rospy.wait_for_service("apriltags_intrude")
        try:
            intrude_query = rospy.ServiceProxy("apriltags_intrude", apriltags_intrude)
            resp = intrude_query(int(msg.x), int(msg.y))
            pos_id_text = "["+str(int(msg.x))+"," +str(int(msg.y))+"]" + "(" + str(resp.id) + ")"
            self.emit(QtCore.SIGNAL("sendPosIDText(PyQt_PyObject)"), pos_id_text)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def updateStateTextbot(self, value):
        self.stateTextbox.moveCursor(QtGui.QTextCursor.End)
        self.stateTextbox.ensureCursorVisible()
        self.stateTextbox.append(str(value))
        self.stateTextbox.update()

    def queryAprilTagsInfo(self):
        #print "clicked"
        rospy.wait_for_service("apriltags_info")
        try:
            info_query = rospy.ServiceProxy("apriltags_info", apriltags_info)
            resp = info_query()
               
            #print str(resp)

            info_text = "" 
            for i in range(len(resp.polygons)):
                poly = resp.polygons[i]
                t_id = resp.ids[i]

                #print(str(poly))
                #print(str(t_id))
                info_text += "["+str(t_id)+"] "
                for p in poly.points:
                    info_text += "(" + str(int(p.x)) + "," + str(int(p.y)) + ")"
                info_text += "\n" 

            self.emit(QtCore.SIGNAL("sendTagInfoText(PyQt_PyObject)"), info_text)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
    def updateAprilTagsTextbot(self, value):
        self.aprilTagsTextbox.clear()
        self.aprilTagsTextbox.moveCursor(QtGui.QTextCursor.End)
        self.aprilTagsTextbox.ensureCursorVisible()
        self.aprilTagsTextbox.append(str(value))
        self.aprilTagsTextbox.update()        


if __name__ == '__main__':

    app = QtGui.QApplication(sys.argv)
    w = SpheroIntrudeForm()
    w.show()
    sys.exit(app.exec_())
  
        
