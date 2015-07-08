var tmp = require('tmp')
var net = require('net');
var events = require('events');

var MouseControl = function(){
  this._server = new net.Server;
  this._serverPath = "";
  this._displays = {};

  tmp.setGracefulCleanup();
  tmp.tmpName(
    function nameGenerated(err, path) {
      if (err){
        console.log("Got an error!");
        throw err;
      }

      console.log("Created temporary filename: ", path);
      this._server.listen(path, this.socketReady.bind(this));
      this._serverPath = path;

      this._server.on('error',
        function (e) {
          if (e.code == 'EADDRINUSE') {
            console.log('ERROR: Mouse unix socket in use!');
          }
        }
      );


    }.bind(this)
  );


  events.EventEmitter.call(this);

  return this;
}

MouseControl.prototype.__proto__ = events.EventEmitter.prototype;

MouseControl.prototype.socketReady = function(){
  console.log("INFO: Mouse server socker listening...");

  process.on('SIGINT', function() {
    console.log('Mouse: exit detected, removing unix socket: ' + this._serverPath);
    this._server.close();
  }.bind(this));

  this._server.on('connection',
    this.onConnection.bind(this)
  );
}

MouseControl.prototype.onConnection = function(socket){
  console.log("Mouse: New connection on unix socket");

  socket.on('close', function(){
    console.log('Mouse: Socket connection closed');
  });


  socket.on('data', function(chunk){
    /*if(chunk.)*/

    console.log('Mouse: data - ' + chunk);

    var data = JSON.parse(chunk);

    var size = data.screens[0].size;

    var cmd = {
      action : 'move',
      x : Math.random() * (size.width - 0),
      y : Math.random() * (size.height - 0)
    }

    cmd = JSON.stringify(cmd);
    cmd = cmd + '\n' + cmd + '\n';

    console.log("Mouse: Sending command " + cmd);

    socket.write(cmd,
      function(){
        console.log("Mouse: Send complete");
      }
    );
  })
}

MouseControl.prototype.setPosition = function(display){

}

module.exports = MouseControl;


var MouseSession = function(socket){
  //
}
