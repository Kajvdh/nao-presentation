#!flask/bin/python
from flask import Flask, abort, jsonify, request
from naoqi import ALProxy
import argparse
import motion
import time
import almath
import logger

app = Flask(__name__)

nao_host = "localhost"
nao_port = 53417
logger = logger.Logger(4) # Initialize logger with level "debug"

@app.route('/behaviors', methods=['GET'])
def get_behaviors():
    logger.debug("get_behaviors() called")
    managerProxy = ALProxy("ALBehaviorManager", nao_host, nao_port)
    behaviors = managerProxy.getInstalledBehaviors()
    return jsonify({"behaviors": behaviors}), 200

@app.route('/behaviors/start', methods=['POST'])
def start_behavior():
    logger.debug("start_behavior() called")
    if not request.json or not 'behavior' in request.json:
        abort(400)
    behavior = str(request.json['behavior'])
    managerProxy = ALProxy("ALBehaviorManager", nao_host, nao_port)

    if (managerProxy.isBehaviorInstalled(behavior)):
        logger.debug("Behavior "+behavior+" is present on the robot, starting behavior...")
        managerProxy.post.runBehavior(behavior)
        return jsonify({"started": behavior}), 200
    else:
        logger.debug("Behavior "+behavior+" is NOT present on the robot")
        return jsonify({"error": "Behavior not found"}), 404

@app.route('/behaviors/stop', methods=['POST'])
def stop_behavior():
    logger.debug("stop_behavior() called")
    if not request.json or not 'behavior' in request.json:
        abort(400)
    behavior = str(request.json['behavior'])
    managerProxy = ALProxy("ALBehaviorManager", nao_host, nao_port)

    if (managerProxy.isBehaviorRunning(behavior)):
        logger.debug("Behavior "+behavior+" is running on the robot, stopping behavior...")
        managerProxy.stopBehavior(behavior)
        return jsonify({"stopped": behavior}), 200
    else:
        logger.debug("Behavior "+behavior+" is NOT running on the robot")
        return jsonify({"error": "Behavior not running"}), 404

@app.route('/behaviors/stop/all', methods=['GET'])
def stop_behaviors():
    logger.debug("stop_behaviors() called")
    managerProxy = ALProxy("ALBehaviorManager", nao_host, nao_port)
    behaviors = managerProxy.getRunningBehaviors()

    if (len(behaviors) > 0):
        managerProxy.stopAllBehaviors()
        return jsonify({"stopped": behaviors}), 200
    else:
        return jsonify({"error": "No running behaviors"}), 400


@app.route('/')
def index():
    return "Hello, To The Point!"

@app.route('/greet/<string:name>')
def greet(name):
    tts = ALProxy("ALTextToSpeech", nao_host, nao_port)
    tts.say("Hello "+ str(name) +"!")
    return "I just greeted "+ str(name) +"!", 200

@app.route('/move')
def move():
    motion = ALProxy("ALMotion", nao_host, nao_port)
    motion.moveInit()
    motion.post.moveTo(1.5, 0, 0)
    return "Better start running!", 200

@app.route('/kick')
def kick():
    motionProxy  = ALProxy("ALMotion", nao_host, nao_port)
    postureProxy = ALProxy("ALRobotPosture", nao_host, nao_port)
    # Wake up robot
    motionProxy.wakeUp()

    # Send robot to Stand Init
    postureProxy.goToPosture("StandInit", 0.5)

    # Activate Whole Body Balancer
    isEnabled  = True
    motionProxy.wbEnable(isEnabled)

    # Legs are constrained fixed
    stateName  = "Fixed"
    supportLeg = "Legs"
    motionProxy.wbFootState(stateName, supportLeg)

    # Constraint Balance Motion
    isEnable   = True
    supportLeg = "Legs"
    motionProxy.wbEnableBalanceConstraint(isEnable, supportLeg)

    # Com go to LLeg
    supportLeg = "LLeg"
    duration   = 2.0
    motionProxy.wbGoToBalance(supportLeg, duration)

    # RLeg is free
    stateName  = "Free"
    supportLeg = "RLeg"
    motionProxy.wbFootState(stateName, supportLeg)

    # RLeg is optimized
    effector = "RLeg"
    axisMask = 63
    frame    = motion.FRAME_WORLD

    # Motion of the RLeg
    times   = [2.0, 2.7, 4.5]

    path = computePath(motionProxy, effector, frame)

    motionProxy.transformInterpolations(effector, frame, path, axisMask, times)

    # Example showing how to Enable Effector Control as an Optimization
    isActive     = False
    motionProxy.wbEnableEffectorOptimization(effector, isActive)

    # Com go to LLeg
    supportLeg = "RLeg"
    duration   = 2.0
    motionProxy.wbGoToBalance(supportLeg, duration)

    # RLeg is free
    stateName  = "Free"
    supportLeg = "LLeg"
    motionProxy.wbFootState(stateName, supportLeg)

    effector = "LLeg"
    path = computePath(motionProxy, effector, frame)
    motionProxy.transformInterpolations(effector, frame, path, axisMask, times)

    time.sleep(1.0)

    # Deactivate Head tracking
    isEnabled = False
    motionProxy.wbEnable(isEnabled)

    # send robot to Pose Init
    postureProxy.goToPosture("StandInit", 0.3)

    # Go to rest position
    motionProxy.rest()
    return "ZzzZzzZzzZ", 200



def computePath(proxy, effector, frame):
    dx      = 0.05                 # translation axis X (meters)
    dz      = 0.05                 # translation axis Z (meters)
    dwy     = 5.0*almath.TO_RAD    # rotation axis Y (radian)

    useSensorValues = False

    path = []
    currentTf = []
    try:
        currentTf = proxy.getTransform(effector, frame, useSensorValues)
    except Exception, errorMsg:
        print str(errorMsg)
        print "This example is not allowed on this robot."
        exit()

    # 1
    targetTf  = almath.Transform(currentTf)
    targetTf *= almath.Transform(-dx, 0.0, dz)
    targetTf *= almath.Transform().fromRotY(dwy)
    path.append(list(targetTf.toVector()))

    # 2
    targetTf  = almath.Transform(currentTf)
    targetTf *= almath.Transform(dx, 0.0, dz)
    path.append(list(targetTf.toVector()))

    # 3
    path.append(currentTf)

    return path


robots = [
    {
        'id': 1,
        'title': u'Nao',
        'description': u'The default Nao robot'
    },
    {
        'id': 2,
        'title': u'Zora',
        'description': u'An advanced Nao robot that takes care of your health'
    }
]

@app.route('/robots', methods=['GET'])
def get_robots():
    return jsonify({'robots': robots}), 200

@app.route('/robots/<int:robot_id>', methods=['GET'])
def get_robot(robot_id):
    robot = [robot for robot in robots if robot['id'] == robot_id]
    if len(robot) == 0:
        abort(404)
    return jsonify({'robot': robot[0]}), 200

if __name__ == '__main__':
    app.run(debug=True)
