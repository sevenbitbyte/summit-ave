const readline = require('readline');
const spawn = require('child_process').spawn;
const extend = require('util')._extend;
const exec = require('child_process').exec;
const ROSLIB = require('roslib')
var ros = new ROSLIB.Ros();

var uri = "ws://vega01.local:4000"
const DEVICE_ID = '54:AB:3A:F3:E1:18';
const gattProcess = spawn('gatttool', ['-b', DEVICE_ID, '--char-write-req', '--handle=0x002d', '--value=0300', '--listen']);

function exitHandler(err) {
  gattProcess.kill('SIGTERM')
  if (err) console.log(err);
  process.exit();
}

process.on('exit', exitHandler.bind());
process.on('SIGINT', exitHandler.bind());
process.on('uncaughtException', exitHandler.bind());

var poseTopic;
var yawAcc = 0
var lastMs = 0

var connectionError = function(err){ console.log("Error" + JSON.stringify(err)); }
var connectionClosed = function(){ console.log("Closed");}

ros.on('error', function(error) {
  connectionError(error);
  setTimeout(()=>{
    console.log('retry')
    ros.connect(uri)
  }, 5000)
});


ros.on('close', function() {
  poseTopic = undefined
  connectionClosed();
  ros.connect(uri);
});


ros.on('connection', function() {
  console.log(`ROS client connected: ${ros.socket.url}`)

  poseTopic = new ROSLIB.Topic({
     ros: ros,
     name: '/dreammouse/pose',
     messageType: 'geometry_msgs/PoseStamped'
   })

  return
});

ros.connect(uri);

var mouseState = {
  touch: {},
  touch_start: {},
  touch_end: {},
  buttons: {
    left: false,
    right: false,
    super: false,
    volume_up: false,
    volune_down: false
  },
  imu: {}
}

function toDeg(a){
  let val;
  if(a > 127){
    val = ((255 - a) / 127) * 90
  }
  else{
    val = ((0 - a) / 127) * 90
  }

  return val
}

function degToRad(a){
  return a * (Math.PI / 180)
}



function updateMouse(newState){
  var cmd = ''

  if(mouseState.imu.rpy){
    let yawDelta = newState.imu.rpy.yaw - mouseState.imu.rpy.yaw
    if(yawDelta < -60 || yawDelta > 60){ yawDelta=0 }
    yawAcc += -(yawDelta)

    console.log(yawDelta)

    var phi = degToRad(newState.imu.rpy.roll) / 2.0;
    var the = degToRad(newState.imu.rpy.pitch) / 2.0;
    var psi = degToRad(yawAcc) ;
    var x = Math.sin(phi) * Math.cos(the) * Math.cos(psi) - Math.cos(phi) * Math.sin(the)
        * Math.sin(psi);
    var y = Math.cos(phi) * Math.sin(the) * Math.cos(psi) + Math.sin(phi) * Math.cos(the)
        * Math.sin(psi);
    var z = Math.cos(phi) * Math.cos(the) * Math.sin(psi) - Math.sin(phi) * Math.sin(the)
        * Math.cos(psi);
    var w = Math.cos(phi) * Math.cos(the) * Math.cos(psi) + Math.sin(phi) * Math.sin(the)
        * Math.sin(psi);

    let pose = {
      orientation: {
        x: x,
        y: y,
        z: z,
        w: w
      }
    }

    console.log(newState.imu.rpy);
    if(poseTopic){
      let poseMsg = new ROSLIB.Message({pose:pose, header: {frame_id: 'base_link'}})
      poseTopic.publish(poseMsg)
    }
  }

  if(mouseState.buttons.left != newState.buttons.left){
    cmd += ' ' + ((newState.buttons.left) ? 'mousedown' : 'mouseup') + ' ' + 1
  }

  if(mouseState.buttons.right != newState.buttons.right){
    cmd += ' ' + ((newState.buttons.right) ? 'mousedown' : 'mouseup') + ' ' + 3
  }

  if(mouseState.buttons.super != newState.buttons.super){
    cmd += ' ' + ((newState.buttons.super) ? 'keydown' : 'keyup')
    cmd += ' super'
  }

  if(mouseState.buttons.volume_up != newState.buttons.volume_up){
    cmd += ' ' + ((newState.buttons.volume_up) ? 'keydown' : 'keyup')
    cmd += ' XF86AudioRaiseVolume'
  }

  if(mouseState.buttons.volume_down != newState.buttons.volume_down ){
    cmd += ' ' + ((newState.buttons.volume_down) ? 'keydown' : 'keyup')
    cmd += ' XF86AudioLowerVolume'
  }

  if((mouseState.touch.x != newState.touch.x || mouseState.touch.y != newState.touch.y || newState.touch.x != 0 || newState.touch.y != 0)/* && !newState.buttons.left*/){
    //cmd += ' mousemove_relative -- ' + (newState.touch.x/2) + ' ' + (newState.touch.y/2)
    if(mouseState.touch.x == 0 && mouseState.touch.y == 0){
      newState.touch_start.x = newState.touch.x
      newState.touch_start.y = newState.touch.y
    }
    else if(mouseState.touch_start.x != 0 && mouseState.touch_start.y != 0){
      newState.touch_end.x = newState.touch.x
      newState.touch_end.y = newState.touch.y

      let delta = {
        x: newState.touch_end.x - mouseState.touch.x,
        y: newState.touch_end.y - mouseState.touch.y,
      }

      if(newState.touch.x == 0 && newState.touch.y == 0){
        newState.touch_start.x = 0
        newState.touch_start.y = 0
        console.log('end')
      }
      else{
        cmd += ' mousemove_relative -- ' + delta.x*2 + ' ' + delta.y*2

        console.log('delta')
        console.log(delta)
      }
    }

  }
  else if(mouseState.imu.rpy && newState.imu.rpy){
    let delta = {
      x: -degToRad(yawAcc),
      y: newState.imu.rpy.pitch - mouseState.imu.rpy.pitch,
    }
    cmd += ' mousemove_relative -- ' + delta.x*5 + ' ' + delta.y*5
  }

  var nowMs = (new Date).getTime()
  var deltaMs = nowMs - lastMs
  lastMs = nowMs;

  if(cmd.length < 1){


    mouseState = newState
    return
  }

  mouseState = newState
  console.log(newState)
  exec('xdotool' + cmd, (error, stdout, stderr) => {
    if (error) {
      console.error(`exec error: ${error}`);
      child = undefined
      return;
    }
  });

}

function sendCommand(cmd){

}

gattProcess.stderr.on('data', (data) => {
  console.log(`stderr: ${data}`);
});

gattProcess.on('close', (code) => {
  console.log(`child process exited with code ${code}`);
});

const rl = readline.createInterface({
  input: gattProcess.stdout,
});

function mkShort(a,b){
  return ((0xff&a)<<8 | (0xff&b))
}

rl.on('line', (input) => {

  var idx = input.indexOf('value:');

  input=input.substring(idx+7)

  let tokens = input.split(' ')

  if(tokens.length!=21){return;}


  let buf = Buffer.from(tokens.join(''), "hex")

  let buttonState = buf[18] & 0x1f
  let touchX = buf[18] & 0xe0

  let byte1 = ((buf[16] & 0x1f) << 3) |  ((buf[17] & 0xe0) >>> 5)
  let byte2 = ((buf[17] & 0x1f) << 3) |  ((buf[18] & 0xe0) >>> 5)

  var newMouseState = Object.assign({}, mouseState)

  if(byte1 == 0 || byte2 == 0){
    newMouseState.touch = {
      x: 0,
      y: 0
    }
  }
  else{
    newMouseState.touch = {
      x: byte1,
      y: byte2
    }
  }


/**

0: pitch,
1: pitch L
2: yaw L
4: roll L
5: roll ?
**/


  let roll = toDeg(buf[7])
  let pitch = toDeg(buf[2])
  let yaw = toDeg(buf[4])

  var i=2
  var ffff = 0xffff
  newMouseState.imu = {
    t: mkShort(buf[0], buf[1]),
    rpy: {
      roll: roll,
      pitch: pitch,
      yaw: yaw
    }
  }

  //console.log(newMouseState.imu)

  newMouseState.buttons = {
    left: (buttonState & 0x1) && true,
    right: (buttonState & 0x4) && true,
    super: (buttonState & 0x2) && true,
    volume_down: (buttonState & 0x8) && true,
    volume_up: (buttonState & 0x10) && true
  }

  if(JSON.stringify(newMouseState) != JSON.stringify(mouseState)){
    updateMouse(newMouseState)
  }
});
