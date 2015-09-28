var Hoek = require('hoek')
var Bounds = require('./bounds')

var LLGrid = function(options){
  this.sizeLL = Hoek.reach(options, 'sizeLL', undefined, true)
  this.sizePx = Hoek.reach(options, 'sizePx', undefined, true)

  this.grids = {};

  return this;
}

LLGrid.prototype.count = function(){
  return Math.pow( Math.ceil(360.0/this.sizeLL), 2 );
}

LLGrid.prototype.getGrid = function(idx){
  if(this.grids[idx]){
    return this.grids[idx]
  }

  this.grids[idx] = this.gridIndexToBounds(idx)
  return this.grids[idx];
}

LLGrid.prototype.pointToGridIndex = function(point){
  var tileWidth = Math.ceil(360.0 / this.sizeLL)

  var shiftedX = point.x + 360
  var shiftedY = point.y + 360

  var tileX = Math.floor(shiftedX / this.sizeLL)
  var tileY = Math.floor(shiftedY / this.sizeLL)

  return tileX + (tileY * tileWidth)
}

LLGrid.prototype.gridIndexToBounds = function(idx){
  var tileWidth = Math.ceil(360.0 / this.sizeLL)

  var tileX = idx % tileWidth;
  var tileY = Math.floor(idx / tileWidth)

  var lowerleft = {
    x : - (180 - (tileX * this.sizeLL)),
    y : - (180 - (tileY * this.sizeLL))
  }

  var topRight = {
    x : -(180 - ((tileX+1) * this.sizeLL)),
    y : -(180 - ((tileY+1) * this.sizeLL))
  }

  return new Bounds({
    lower: lowerleft,
    upper: topRight
  });
}

LLGrid.prototype.gridsWithinBounds = function(bounds){

  var indexes = [];

  var tileWidth = Math.ceil(360.0 / this.sizeLL)
  var bottomLeft = this.pointToGridIndex(bounds.bottomLeft())
  var bottomRight = this.pointToGridIndex(bounds.bottomRight())
  var topLeft = this.pointToGridIndex(bounds.topLeft())
  var topRight = this.pointToGridIndex(bounds.topRight())

  var deltaY = Math.round((topLeft - bottomLeft) / tileWidth)
  var deltaX = Math.round(bottomRight - bottomLeft)


  for (var y = 0; y < deltaY; y++){
    for(var x = 0; x < deltaX; x++){

      var idx = Math.round((bottomLeft + x) + (tileWidth * y))
      indexes.push(idx)
    }
  }

  return indexes;
}


module.exports = LLGrid;
