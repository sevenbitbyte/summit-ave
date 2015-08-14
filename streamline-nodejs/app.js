var os = require('os');
var config = require('config');

console.log(config);

var express = require('express');
var session = require('express-session')
var compression = require('compression')
var http = require('http');
var path = require('path');
var favicon = require('static-favicon');
var logger = require('morgan');
var cookieParser = require('cookie-parser');
var bodyParser = require('body-parser');

var routes = require('./routes');
//var users = require('./routes/user');
var nimbus = require('./routes/nimbus/');

var async = require('async');
var app = express();

// view engine setup
app.set('views', path.join(__dirname, 'views'));
app.set('view engine', 'ejs');

app.use(favicon());
app.use(logger('dev'));


app.use(bodyParser.json({limit: '50mb'}));
app.use(bodyParser.urlencoded({limit: '50mb'}));

app.use(cookieParser());
app.use(require('stylus').middleware(path.join(__dirname, 'public')));
app.use(express.static(path.join(__dirname, 'public')));

//Setup session
app.use(
  session({
    secret: 'toboldlygo',
    resave: true,
    saveUninitialized: false
  })
);


app.use(compression());
app.use(app.router);

app.get('/', routes.index);
app.post('/nimbus', nimbus.dispatch);


var Canvas = require('canvas');

app.get('/img',
  function(req, res){
    Image = Canvas.Image
    , canvas = new Canvas(200,200)
    , ctx = canvas.getContext('2d');

    ctx.font = '30px Impact';
    ctx.rotate(.1);
    ctx.fillText("Awesome!", 50, 100);

    var te = ctx.measureText('Awesome!');
    ctx.strokeStyle = 'rgba(0,0,0,0.5)';
    ctx.beginPath();
    ctx.lineTo(50, 102);
    ctx.lineTo(50 + te.width, 102);
    ctx.stroke();

    res.send(canvas.toBuffer());
  }
);


var TerrainParser = require('./routes/nimbus/parsers/terrain_arcascii_crawler');
var TerrainRenderer = require('./routes/nimbus/terrain_renderer.js');

var dataSource = new TerrainParser.TerrainDataSource({});
var terrainRenderer = undefined;

dataSource.on('ready', function(){
  terrainRenderer = new TerrainRenderer(dataSource);
});

app.get('/terrain/tile/info/:lat,:long.json',
  function(req, res){
    var model = dataSource.getModel();

    console.log('TileInfo');

    var query = model.TileInfo.find({ $and: [
      {'lowerleft.x' : { $lt : req.params.long}},
      {'lowerleft.y' : { $lt : req.params.lat}},
      {'topright.x' : { $gt : req.params.long}},
      {'topright.y' : { $gt : req.params.lat}},
      ]}).limit(1);
    var promise = query.exec();
    promise.then(
      function(infoList){
        console.log('Found ' + infoList.length + ' tiles');
        res.send(JSON.stringify(infoList));
      }
    );
  }
);

app.get('/terrain/tile/elev/:lat,:long.png',
  function(req, res){
    var model = dataSource.getModel();


    var query = model.TileInfo.find({ $and: [
      {'lowerleft.x' : { $lt : req.params.long}},
      {'lowerleft.y' : { $lt : req.params.lat}},
      {'topright.x' : { $gt : req.params.long}},
      {'topright.y' : { $gt : req.params.lat}},
      ]}).limit(1);
    var promise = query.exec();
    promise.then(
      function(infoList){
        console.log('Found ' + infoList.length + ' tile');


        var rendered = 0;
        for(infoIdx in infoList){
          info = infoList[infoIdx];

          console.log(info.lowerleft)
          console.log(info.topright)

          var dataQuery = model.TileData.find(
            { origin: info.origin,
              'lowerleft.x': info.lowerleft.x,
              'lowerleft.y': info.lowerleft.y}).limit(1);

          var dataPromise = dataQuery.exec();
          dataPromise.then(
            function(thatInfo){

              return function(dataList){
                console.log('Found tile data: ' + dataList.length);
                var buffer = terrainRenderer.getPng(thatInfo, dataList[0], parseFloat(req.query.scale));
                console.log('Rendered tile: ' + thatInfo.id);
                res.setHeader('Content-Type', 'image/x-png'); //Solution!
                res.writeHead(200);
                res.end(buffer);
                console.log('Send complete');
              };
            }(info)
          );
          return;
        }
      }
    );
  }
);


//app.post('/api/:user', users.info);

app.get('/landing', function(req, res){
  var view = 'remote_landing';
  if(req.ip == '127.0.0.1'){
    view = 'local_landing';
  }

  res.render(view, {
      title: 'streamline',
      info: req.ip + os.hostname()
  });
});

/// catch 404 and forwarding to error handler
app.use(function(req, res, next) {
    var err = new Error('Not Found');
    err.status = 404;
    next(err);
});

/// error handlers

// development error handler
// will print stacktrace
if (app.get('env') === 'development') {
    app.use(function(err, req, res, next) {
        res.render('error', {
            message: err.message,
            error: err
        });

        console.log(err.stack);
    });
}

// production error handler
// no stacktraces leaked to user
app.use(function(err, req, res, next) {
    res.render('error', {
        message: err.message,
        error: {}
    });
});

app.set('port', process.env.PORT || 3000);



var server = http.createServer(app);

server.on('connection',
  function(s){
    var conn = {
      remote : {
        port : s.remotePort,
        address : s.remoteAddress
      },
      local : {
        port : s.localPort,
        address : s.localAddress
      }
    }
    console.log(conn);
  }
);

var listenPort = config.has('port') ? config.get('port') : 3000;

server.listen(listenPort);


module.exports = app;
