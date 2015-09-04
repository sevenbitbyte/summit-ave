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

TerrainRenderer.prototype.getPng = function(info, data, scale){

  if(!scale){
    scale = 1.0;
  }

  var canvas = new Canvas(data.sizePx.x*scale, data.sizePx.y*scale);
  var ctx = canvas.getContext('2d');
  var hist = new Histogram(info.elevHist);

  for(var y = 0; y < data.sizePx.y; y++){
    for(var x = 0; x < data.sizePx.x; x++){

      var value = Math.floor(255 * hist.value(data.elev.readInt32LE(((data.sizePx.x * y) + x) * 4)));
      //var value = Math.round(255 * (data.elev.readInt32LE(((data.sizePx.x * y) + x) * 4) / (info.elevHist.max) ));

      var a = 1;
      var r = value;
      var g = value;
      var b = value;

      ctx.fillStyle = 'rgba(' + r + ',' + g + ',' + b + ',' + a + ')';

      ctx.fillRect(scale * x, scale * (data.sizePx.y-y), scale, scale);
    }
  }

  console.log('scale='+scale)

  return canvas.toBuffer();
}


var fileExists = function(path) {
  try {
    var stats = fs.lstatSync(path);

    return stats.isFile();
  }
  catch(e){

  }
  return false;
}

TerrainRenderer.prototype.drawTile = function(options, callback){
  var info = options.info
  var data = options.data
  var scale = options.scale

  //Support JSON
  if(this.formats.json){
    var obj={
      lowerleft : data.lowerleft,
      sizePx : data.sizePx,
      sizeLL : data.sizeLL,
      histogram : info.elevHist
    };

    var jsonPath = this.path + '/json/' + info.id + '.json';

    if(!fileExists(jsonPath)){
      console.log('Writing: ' + jsonPath);
      var outJson = fs.createWriteStream(jsonPath);

      outJson.on('open', function(){
        outJson.end(JSON.stringify(obj));
        this.emit('json', jsonPath);
      });
    }
    else {
      //console.log('Skipping: ' + jsonPath);
    }
  }

  //Image formats
  if(this.formats.png || this.formats.jpeg){


    if(!scale){
      scale = 1.0;
    }

    var jpegPath = this.path + '/jpeg/' + info.id + ':' + scale + '.jpg';
    var pngPath = this.path + '/png/' + info.id + ':'+ scale + '.png';

    var jpegPathExists = fileExists(jpegPath);
    var pngPathExists = fileExists(pngPath);


    if(!jpegPathExists || !pngPathExists){

      console.log('Drawing: ' + info.id)

      var canvas = new Canvas(data.sizePx.x*scale, data.sizePx.y*scale);
      var ctx = canvas.getContext('2d');
      var hist = new Histogram(info.elevHist);

      for(var y = 0; y < data.sizePx.y; y++){
        for(var x = 0; x < data.sizePx.x; x++){

          var value = Math.floor(255 * hist.value(data.elev.readInt32LE(((data.sizePx.x * y) + x) * 4)));
          //var value = Math.round(255 * (data.elev.readInt32LE(((data.sizePx.x * y) + x) * 4) / (info.elevHist.max) ));

          var a = 1;
          var r = value;
          var g = value;
          var b = value;

          ctx.fillStyle = 'rgba(' + r + ',' + g + ',' + b + ',' + a + ')';

          ctx.fillRect(scale * x, scale * (data.sizePx.y-y), scale, scale);
        }
      }
    }
    else{
      console.log('Skipping: ' + info.id + ':' + scale);
    }


    //Jpeg output
    if(this.formats.jpeg){


      if(!jpegPathExists){
        console.log('Writing: ' + jpegPath);
        var outJpg = fs.createWriteStream(jpegPath);
        var streamJpg = canvas.jpegStream();

        streamJpg.on('data', function(chunk){
          outJpg.write(chunk);
        });


        streamJpg.on('end', function(){
          this.emit('jpeg', jpegPath);
        });
      }
      else {
        console.log('Skipping: ' + jpegPath);
      }
    }

    //Png output
    if(this.formats.png){

      if(!pngPathExists){
        console.log('Writing: ' + pngPath);
        var outPng = fs.createWriteStream(pngPath);
        var streamPng = canvas.pngStream();

        streamPng.on('data', function(chunk){
          outPng.write(chunk);
        });


        streamPng.on('end', function(){
          this.emit('png', pngPath);
          console.log('Wrote: ' + pngPath);
        });
      }
      else {
        console.log('Skipping: ' + pngPath);
      }
    }
  }



  callback();

}

module.exports = TerrainRenderer;
