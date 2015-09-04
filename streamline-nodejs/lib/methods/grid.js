var Bounds = require('./bounds')

var LLGrid = function(options){
  this.sizeLL = options.sizeLL
  this.sizePx = options.sizePx

  this.grids = [];



}

LLGrid.prototype.tileCount = function(){
  return Math.pow( Math.ceil(360.0/this.sizeLL), 2 );
}


LLGrid.prototype.gridIndexToBounds = function(idx){
  var tileWidth = Math.ceil(360.0 / this.sizeLL)

  var tileX = idx % tileWidth;
  var tileY = Math.floor(idx / tileWidth)

  var lowerleft = {
    x : tileX * this.sizeLL,
    y : tileY * this.sizeLL
  }

  var topRight = {
    x : (tileX+1) * this.sizeLL,
    y : (tileY+1) * this.sizeLL
  }

  return new Bounds({
    lower: lowerleft,
    upper: topRight
  });
}


module.exports = LLGrid;
