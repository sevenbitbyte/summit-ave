var fs = require('fs')
var events = require('events');
var Canvas = require('canvas');
var Histogram = require('./histogram.js');

var TerrainRenderer = function(datasource, formats){
  this.datasource = datasource;
  this.path = './media/terrain/';

  if(formats === undefined){
    this.formats = {json: true, jpeg: true, png: true};
  }
  else{
    this.formats = formats;
  }


  events.EventEmitter.call(this);

  return this;
}

TerrainRenderer.prototype.__proto__ = events.EventEmitter.prototype;

TerrainRenderer.prototype.getPng = function(info, data){
  var canvas = new Canvas(data.sizePx.x, data.sizePx.y)
  var ctx = canvas.getContext('2d');
  var hist = new Histogram(info.elevHist);

  for(var y = 0; y < data.sizePx.y; y++){
    for(var x = 0; x < data.sizePx.x; x++){

      //var value = Math.floor(255 * hist.value(data.elev.readInt32LE(((data.sizePx.x * y) + x) * 4)));
      var value = Math.round(255 * (data.elev.readInt32LE(((data.sizePx.x * y) + x) * 4) / (info.elevHist.max) ));

      var a = 1;
      var r = value;
      var g = value;
      var b = value;

      ctx.fillStyle = 'rgba(' + r + ',' + g + ',' + b + ',' + a + ')';

      ctx.fillRect(x, (data.sizePx.y -1 )- y, 1, 1);
    }
  }

  return canvas.toBuffer();
}

TerrainRenderer.prototype.drawTile = function(info, data){
  //Support JSON
  if(this.formats.json){
    var obj={
      lowerleft : data.lowerleft,
      sizePx : data.sizePx,
      sizeLL : data.sizeLL,
      histogram : info.elevHist
    };

    var jsonPath = this.path + '/json/' + info.id + '.'+ (new Date()).getTime() + '.json';
    outJson = fs.createWriteStream(jsonPath);

    outJson.on('open', function(){
      outJson.end(JSON.stringify(obj));
      this.emit('json', jsonPath);
    });


  }

  //Image formats
  if(this.formats.png || this.formats.jpeg){
    var canvas = new Canvas(data.sizePx.x, data.sizePx.y)
    var ctx = canvas.getContext('2d');
    var hist = new Histogram(info.elevHist);

    for(var y = 0; y < data.sizePx.y; y++){
      for(var x = 0; x < data.sizePx.x; x++){

        var value = Math.floor(255 * hist.value(data.elev.readInt32LE(((data.sizePx.x * y) + x) * 4)));

        var a = 1;
        var r = value;
        var g = value;
        var b = value;

        ctx.fillStyle = 'rgba(' + r + ',' + g + ',' + b + ',' + a + ')';

        ctx.fillRect(x, data.sizePx.y-y, 1, 1);
      }
    }

    //Jpeg output
    if(this.formats.jpeg){
      var jpegPath = this.path + '/jpeg/' + info.id + '.'+ (new Date()).getTime() + '.jpg';
      outJpg = fs.createWriteStream(jpegPath);
      streamJpg = canvas.jpegStream();

      streamJpg.on('data', function(chunk){
        outJpg.write(chunk);
      });


      streamJpg.on('end', function(){
        this.emit('jpeg', jpegPath);
      });
    }

    //Png output
    if(this.formats.png){
      var pngPath = this.path + '/png/' + info.id + '.'+ (new Date()).getTime() + '.png';
      outPng = fs.createWriteStream(pngPath);
      streamPng = canvas.pngStream();

      streamPng.on('data', function(chunk){
        outPng.write(chunk);
      });


      streamPng.on('end', function(){
        this.emit('png', pngPath);
      });
    }
  }



}

module.exports = TerrainRenderer;
