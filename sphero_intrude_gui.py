#!/usr/bin/python
import polygonsToGraph, RRT, AStar
import sys, rospy, math, datetime 
from PyQt4 import QtGui, QtCore
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import ColorRGBA, Float32, Bool
from apriltags_intrude_detector.srv import apriltags_intrude
from apriltags_intrude_detector.srv import apriltags_info

# You implement this class
class Controller:
    stop = True # This is set to true when the stop button is pressed

	
    def __init__(self):
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.trackposSub = rospy.Subscriber("tracked_pos", Pose2D, self.trackposCallback)
        self.close_distance = 5.0
        self.last_node = None
        self.next_node = None    
        self.current_node_index = 0
        self.current_vx = 20.0
        self.current_vy = 20.0   
        self.path = []
        self.goal = -1

    def isCloseTo(self, point1, point2):
        return (math.fabs(point1.x - point2.x) < self.close_distance and  math.fabs(point1.y - point2.y) < self.close_distance)       
        
    def getVelocityChange(self, robotLocation):
        if(not self.next_node or not self.last_node):
            print "NULL NODES"
            return
        if (self.isCloseTo(robotLocation, self.next_node.center)):
            print "IN IF STATEMENT"
            if (self.next_node.n_id == self.goal):
                self.current_vx = 0
                self.current_vy = 0
            angle = math.atan2(float(-1*(self.next_node.center.y - robotLocation.y)), float(self.next_node.center.x - robotLocation.x))
            self.current_vx = math.cos(angle)
            self.current_vy = math.sin(angle)
            self.last_node = self.next_node
            self.current_node_index += 1
            self.next_node = self.path[self.current_node_index]

    def trackposCallback(self, msg):
        # This function is continuously called
        if not self.stop:
            twist = Twist()
            self.getVelocityChange(msg)
            #print "Velocity: " + str(vel)
            # Change twist.linear.x to be your desired x velocity
            twist.linear.x = self.current_vx
            # Change twist.linear.y to be your desired y velocity
            twist.linear.y = self.current_vy
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
            time = str(datetime.datetime.now()).replace(' ', '').replace('.','').replace(':','').strip()
            print time
            f = open('/home/mu/polygons/polygons' + time + '.txt', 'w')
            print "created f"
            for i in range(len(resp.polygons)):
                # A polygon (access points using poly.points)
                poly = resp.polygons[i]
                # The polygon's id (just an integer, 0 is self.goal, all else is bad)
                t_id = resp.ids[i]


        except Exception, e:
            print "Exception: " + str(e)
        finally:
            self.stop = False
        print "CONTROLLER"
        graph = polygonsToGraph.translateToGraph(resp.polygons)
        self.goal = graph.goal
       # polygonsToGraph.visualizeGraph(graph, resp.polygons)
        rrt = RRT.RRT(graph)
        self.path = rrt.run()
        #astar = AStar.AStar(graph)
        #self.path = astar.run()
        #polygonsToGraph.visualizeResultFromNodes(graph, self.path, resp.polygons)
        self.last_node = self.path[0]
        self.next_node = self.path[1]

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
  
        
