#!flask/bin/python
from flask import Flask, abort, jsonify, request
from naoqi import ALProxy

app = Flask(__name__)

nao_host = "localhost"
nao_port = 53417

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
