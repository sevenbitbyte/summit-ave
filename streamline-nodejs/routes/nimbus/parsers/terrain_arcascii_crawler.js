var fs = require('fs');
var Path = require('path');
var byline = require('byline');
var events = require('events');
var Promise = require('mpromise');
var mongoose = require('mongoose');
var Histogram = require('../histogram.js');
var db_connection = mongoose.createConnection('mongodb://localhost/streamline-terrain');

var model = require('../models/terrain_model');
var TerrainModel = undefined;

var reportError = function(value){
  console.log('ERROR: Value = ' + value);
}



db_connection.on('error', reportError);

db_connection.on('connecting', function(){ console.log('mongoose connecting'); });
db_connection.on('connected', function(){ console.log('mongoose connected'); });


db_connection.on('disconnecting', function(){ console.log('mongoose disconnecting'); });
db_connection.on('disconnected', function(){ console.log('mongoose disconnected'); });
db_connection.on('close', function(){ console.log('mongoose close'); });
db_connection.on('reconnected', function(){ console.log('mongoose reconnected'); });



var TerrainDataSource = function(options){
  this.crawlPath = (options.path !== undefined) ? options.path : '/home/sevenbit/Documents/Datasets/Topo/data';
  this.files = [];
  this._parseQueue = [];
  events.EventEmitter.call(this);

  this.tiler = new TerrainTiler({});
  this.on('parsed', this.tiler.onData.bind(this.tiler));

  db_connection.on('connected',
    function(){
      TerrainModel = new model.TerrainModel(mongoose, db_connection);
      this.emit('ready');
    }.bind(this)
  );

  return this;
}

TerrainDataSource.prototype.__proto__ = events.EventEmitter.prototype;

TerrainDataSource.prototype.getModel = function(){
  return TerrainModel;
}

TerrainDataSource.prototype.crawl = function(){
  fs.readdir(this.crawlPath, this._dirCrawlCb.bind(this));
}

TerrainDataSource.prototype._dirCrawlCb = function(error, files){
  if(error){
    this.error = error;
    return;
  }
  var that = this;


  files.forEach(function(f){
    var stat = fs.statSync(this.crawlPath + '/' + f)
    if(stat.isDirectory()){
      var dirFiles = fs.readdirSync(this.crawlPath + '/' + f);
      dirFiles.forEach(function(subF){
        files.push(f + '/' + subF);
      }.bind(this));
    }
  }.bind(this));

  var queryParams = [];

  for(idx in files){
    var filePath = this.crawlPath + '/' + files[idx++];
    var typeIdx = filePath.indexOf('.asc');

    if(Path.extname(filePath) === '.asc'){

      queryParams.push( {path: filePath} );


      this.files.push(filePath);
    }
  }

  var query = TerrainModel.Crawl.find({$or: queryParams});
  var promise = query.exec();
  promise.then(
    function(crawls){

      for(idx in crawls){
        var crawl = crawls[idx];

        var fileListIdx = this.files.indexOf(crawl.path);
        if (fileListIdx > -1) {
          this.files.splice(fileListIdx, 1);
        }
        this.emit('skip', {path: crawl.path, date: new Date(crawl.time)} );
      }

      //this.emit('files', this.files);

      this._parseOne();
    }.bind(that)
  );
}


TerrainDataSource.prototype._parseOne = function(){

  var path = this.files.pop();

  if(!path){
    return;
  }

  this._parser = new ArcAsciiParser(path);
  this._parser.on('end',
    function(p){
      this.emit('parsed', p);
      this._parseOne();
    }.bind(this)
  );

  this.emit('parsing', path);
  this._parser.parse();
}


var ArcAsciiParser = function(path){
  this.path = path;
  this.header = {
    time: undefined,
    ncols: undefined,
    nrows: undefined,
    cellsize: undefined,
    lowerleft: {x: undefined, y: undefined},
    nodata: undefined
  }

  this.tiledata = undefined;

  events.EventEmitter.call(this);
  //this.parse();

  return this;
}

ArcAsciiParser.prototype.__proto__ = events.EventEmitter.prototype;

ArcAsciiParser.prototype.toXY = function(latlon){
  var xoffset = (latlon.x - this.header.lowerleft.x) / this.header.cellsize;
  var yoffset = (latlon.y - this.header.lowerleft.y) / this.header.cellsize;

  if(xoffset < 0 || xoffset >= this.header.ncols){
    xoffset = undefined;
  }

  if(yoffset < 0 || yoffset >= this.header.nrows){
    yoffset = undefined;
  }

  return { x: Math.round(xoffset), y: Math.floor(yoffset) };
}

ArcAsciiParser.prototype.offset = function(pt){
  return ((this.header.ncols * ((this.header.nrows -1 )-pt.y)) + pt.x) * 4;
}

/*ArcAsciiParser.prototype.get = function(x,y){
  return this.tiledata [ this.ncols * y ];
}*/

ArcAsciiParser.prototype.parse = function(){
  var fileStream = byline.createStream( fs.createReadStream(this.path) );
  /*var fileReader = readline.createInterface({
    input: fileStream,
    output: process.stdout,
    terminal: false
  });*/


  fileStream.on('end', function(){
    var pixelCount = this.header.nrows * this.header.ncols * 4;
    if(this.tiledata.length != pixelCount){
      var err = 'invalid pixel count, expected ' + pixelCount;
      err += ' values but read ' + this.tiledata.length;
      this.emit('error', err, this);
      return;
    }

   this.emit('end', this);
 }.bind(this));

  this.header.time = (new Date()).getTime();
  this.dataEnd = new Number(0);


  fileStream.on('data', function(line){
    var tokens = line.toString().trim().split(/[ ]+/);

    if(tokens[0] == 'ncols'){
      this.header.ncols = parseInt(tokens[1], 10);
    }
    else if(tokens[0] == 'nrows'){
      this.header.nrows = parseInt(tokens[1], 10);
    }
    else if(tokens[0] == 'cellsize'){
      this.header.cellsize = parseFloat(tokens[1], 10);
    }
    else if(tokens[0] == 'xllcorner'){
      this.header.lowerleft.x = parseFloat(tokens[1], 10);
    }
    else if(tokens[0] == 'yllcorner'){
      this.header.lowerleft.y = parseFloat(tokens[1], 10);
    }
    else if(tokens[0] == 'NODATA_value'){
      this.header.nodata = parseInt(tokens[1], 10);
    }
    else{
      if(this.tiledata === undefined){
        var bufSize = this.header.ncols * this.header.nrows * 4;
        this.tiledata = new Buffer( bufSize );
      }

      if(tokens.length != this.header.ncols){
        throw 'Tokens.length expected to be: ' + this.header.ncols + ' but read ' + tokens.length + 'instead';
      }

      //Store data
      for(i in tokens){
        var value = value = Math.floor(parseInt(tokens[i], 10));;
        if(value == this.header.nodata){
          value = null;
        }
        else{
          if(isNaN(value)){
            throw value;
          }
        }
        //console
        this.tiledata.writeInt32LE(value, this.dataEnd);
        this.dataEnd+=4;
      }
    }

  }.bind(this));
}


var TerrainTiler = function(options){
  this.tilesize = (options.tilesize !== undefined) ? options.tilesize : 0.25;  //size in degrees

  events.EventEmitter.call(this);

  return this;
}

TerrainTiler.prototype.__proto__ = events.EventEmitter.prototype;

TerrainTiler.prototype.onData = function(parsed){
  var crawl = new TerrainModel.Crawl({path: parsed.path, time: parsed.header.time, header: parsed.header});

  this.emit('tiling', parsed.path);

  var totalWidthLL = parsed.header.ncols * parsed.header.cellsize;
  var totalHeightLL = parsed.header.nrows * parsed.header.cellsize;

  var tilesX = Math.ceil( totalWidthLL / this.tilesize );
  var tilesY = Math.ceil( totalHeightLL / this.tilesize );

  var tileSizeLL = {x: this.tilesize, y: this.tilesize};
  var tileSizePx = {x: Math.floor(parsed.header.ncols / (totalWidthLL / this.tilesize)),
    y: Math.floor(parsed.header.nrows/( totalHeightLL / this.tilesize ))};

  for(var xIter=0; xIter < tilesX; xIter++){
    for(var yIter=0; yIter < tilesY; yIter++){
      var lowerleft = {x : (this.tilesize * xIter) + parsed.header.lowerleft.x,
                  y : (this.tilesize * yIter) + parsed.header.lowerleft.y}
      var topright = {x : lowerleft.x + tileSizeLL.x, y : lowerleft.y + tileSizeLL.y}

      var info = new TerrainModel.TileInfo({
        origin : parsed.path,
        lowerleft: {x : lowerleft.x,
                    y : lowerleft.y},
        topright : {x : topright.x, y : topright.y},
        sizeLL : tileSizeLL,
        elevHist : new Histogram({})
      });

      var startPx = parsed.toXY(info.lowerleft);
      var endPx = parsed.toXY(info.topright);

      //console.log(startPx);
      //console.log(endPx)
      //console.log(('allocing ' + (tileSizePx.x * tileSizePx.y * 4) + 'bytes').grey);

      var data = new TerrainModel.TileData({
          origin : parsed.path,
          lowerleft: {x : lowerleft.x,
                      y : lowerleft.y},
          topright : {x : topright.x, y : topright.y},
          sizeLL : tileSizeLL,
          sizePx : tileSizePx,
          elev: new Buffer(tileSizePx.x * tileSizePx.y * 4)
      });

      for(var y = startPx.y; y < endPx.y; y++){
        var startOffset = parsed.offset( {x: startPx.x, y: y} );
        var endOffset = parsed.offset( {x: endPx.x, y: y} );

        //Select source data from parsed tile
        var elevSlice = parsed.tiledata.slice(startOffset, endOffset);

        //TODO: CLEANUP DEBUG CODE
        try{
          //Copy selected source data into data model
          elevSlice.copy(data.elev, tileSizePx.x * (y - startPx.y) * 4, 0, elevSlice.length);
        }
        catch(event){
          console.log('y : ' + y);
          console.log(tileSizePx);
          console.log(startPx)
          console.log(endPx)

          throw event;
        }
      }

      data.markModified('elev');

      //Update histogram
      for(var i=0; i < data.elev.length; i+=4){
        info.elevHist.push(data.elev.readInt32LE(i));
      }

      info.markModified('elevHist');

      data.save(function(err){
        if(!err){return};
        console.warn(err);
        console.warn('failed to save data {originObj:'
        + data.originObj + ' lowerleft:' + data.lowerleft + '}');
      });

      //! TODO: Update info.elevHist

      info.save(function(err){
        if(!err){return};
        console.warn(err);
        console.warn('failed to save info {originObj:'
        + info.originObj + ' lowerleft:' + info.lowerleft + '}');
      });
    }
  }

  crawl.save(function(err){
    if(!err){return};
    console.warn(err);
    console.warn('failed to save crawl {path:'
    + crawl.path + ' time:' + crawl.time + '}');
  });
}



exports.TerrainDataSource = TerrainDataSource;
exports.ArcAsciiParser = ArcAsciiParser;
exports.TerrainTiler = TerrainTiler;
