import time
import pybullet as p
import numpy as np

quitApp = False
guiDisp = True

guiClock = time.clock()
guiMinTime = .5

dataFolder = "dataThingies\\"

pi = 3.141592

speed = 0.05
jointSpeed = 0.01

p.connect(p.GUI)

p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)

p.setGravity(0, 0, -10)

p.loadURDF(dataFolder + "plane.urdf")

# load the MuJoCo MJCF hand
objects = p.loadMJCF(dataFolder + "hand/hand.xml")

hand = objects[0]

anchorPos = [0, 0, 1]

cameraData = p.getDebugVisualizerCamera()
p.resetDebugVisualizerCamera(cameraData[10], cameraData[8], cameraData[9], anchorPos)

anchor = p.createConstraint(hand, -1, -1, -1, p.JOINT_FIXED, [0, 1, 0], [0, 0, 0], anchorPos)

numJoints = p.getNumJoints(hand)

jointIndices = [4, 6, 8, 10, 12, 14, 16, 18, 21, 23, 25, 27, 29, 31, 33, 35, 37, 39, 41]
jointKeys = ['1', 'q', 'a', 'z', '2', 'w', 's', 'x', 'e', 'd', 'c', '4', 'r', 'f', 'v', '5', 't', 'g', 'b']
jointCurPos = [0]*19
paramDict = {}
for joint in jointIndices:
	paramDict[joint] = p.addUserDebugParameter(p.getJointInfo(hand, joint)[1].decode("utf-8"), -pi/2., pi/2., 0.)

p.setRealTimeSimulation(1)


def checkkeyboardinput():  # Put keyboard control here
	global quitApp, anchorPos, guiDisp, guiClock, jointCurPos

	keys = p.getKeyboardEvents()
	if p.B3G_BACKSPACE in keys:
		quitApp = True
	if ord('m') in keys and time.clock()-guiClock > guiMinTime:
		p.configureDebugVisualizer(p.COV_ENABLE_GUI, guiDisp)
		guiDisp = not guiDisp
		guiClock = time.clock()
	if p.B3G_LEFT_ARROW in keys:
		anchorPos = list(map(sum, zip(anchorPos,
		[-x*speed for x in np.cross(list(p.getDebugVisualizerCamera()[5]), list(p.getDebugVisualizerCamera()[4]))])))
		p.changeConstraint(anchor, anchorPos)
	if p.B3G_RIGHT_ARROW in keys:
		anchorPos = list(map(sum, zip(anchorPos, [x*speed for x in np.cross(list(p.getDebugVisualizerCamera()[5]), 
																			list(p.getDebugVisualizerCamera()[4]))])))
		p.changeConstraint(anchor, anchorPos)
	if p.B3G_DOWN_ARROW in keys:
		anchorPos = list(map(sum, zip(anchorPos, [-x*speed for x in list(p.getDebugVisualizerCamera()[5])])))
		p.changeConstraint(anchor, anchorPos)
	if p.B3G_UP_ARROW in keys:
		anchorPos = list(map(sum, zip(anchorPos, [x*speed for x in list(p.getDebugVisualizerCamera()[5])])))
		p.changeConstraint(anchor, anchorPos)
	if ord('p') in keys:
		jointCurPos = [0]*19
	for i, ji in enumerate(jointIndices):
		if ord(jointKeys[i]) in keys and p.B3G_SHIFT not in keys:
			jointCurPos[i] += jointSpeed * pi
		elif ord(jointKeys[i]) in keys and p.B3G_SHIFT in keys:
			jointCurPos[i] -= jointSpeed * pi
		p.setJointMotorControl2(hand, ji, p.POSITION_CONTROL, jointCurPos[i])


def modifyhandjoints():
	for j in jointIndices:
		p.setJointMotorControl2(hand, j, p.POSITION_CONTROL, p.readUserDebugParameter(paramDict[j]))


while not quitApp:
	checkkeyboardinput()
	cameraData = p.getDebugVisualizerCamera()
	p.resetDebugVisualizerCamera(cameraData[10], cameraData[8], cameraData[9], anchorPos)
	# modifyhandjoints()
	time.sleep(0.01)

p.disconnect()
