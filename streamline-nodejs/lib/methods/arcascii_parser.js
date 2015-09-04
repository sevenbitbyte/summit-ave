
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
