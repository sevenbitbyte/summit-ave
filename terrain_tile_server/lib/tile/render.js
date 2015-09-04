var fs = require('fs')
var events = require('events');
var Canvas = require('canvas');


var Histogram = require('../histogram.js');
var TerrainDataSource = require('../db/terrain_datasource');

var TileRenderer = function(datasource){
  this.datasource = datasource;

  return this;
}

TileRenderer.prototype.getImage = function(info, data, scale){

  var 
   this.getCanvas(info, data)
}

TileRenderer.prototype.getCanvas = function(info, data){

  var canvas = new Canvas(data.sizePx.x, data.sizePx.y);
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

      ctx.fillRect(x, (data.sizePx.y-y), 1, 1);
    }
  }

  console.log('scale='+scale)

  return canvas;
}

exports.render = function(bounds, scale, callback){
  //
}
