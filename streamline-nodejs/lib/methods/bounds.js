var Hoek = require('hoek')
var Point = require('./point')

var Bounds = function(options){
  this.lower = new Point(Hoek.reach(options, 'lower', undefined, true))
  this.upper = new Point(Hoek.reach(options, 'upper', undefined, true))

  return this;
}

Bounds.prototype.left = function(){
  return {
    x: this.lower.x
  }
}

Bounds.prototype.right = function(){
  return {
    x: this.upper.x
  }
}

Bounds.prototype.bottom = function(){
  return {
    y: this.lower.y
  }
}

Bounds.prototype.top = function(){
  return {
    y: this.upper.y
  }
}

Bounds.prototype.topLeft = function(){
  return {
    x: this.lower.x,
    y: this.upper.y
  }
}

Bounds.prototype.topRight = function(){
  return {
    x: this.upper.x,
    y: this.upper.y
  }
}

Bounds.prototype.bottomLeft = function(){
  return {
    x: this.lower.x,
    y: this.lower.y
  }
}

Bounds.prototype.bottomRight = function(){
  return {
    x: this.upper.x,
    y: this.lower.y
  }
}

Bounds.prototype.within = function(x, y){

  var pt = {
    x: x,
    y: y
  }

  if(typeof x === 'object'){
    pt = new Point(x)
  }
  else{
    pt = new Point(pt)
  }

  var withinX = (pt.x <= this.upper.x && pt.x > this.lower.x);
  var withinY = (pt.x <= this.upper.y && pt.x > this.lower.y);

  return (withinX && withinY);
}

Bounds.prototype.intersect = function(other, recurse){

  if( this.within(other.topLeft()) || this.within(other.topRight()) ||
      this.within(other.bottomLeft()) || this.within(other.bottomRight()) )
  {
    return true;
  }

  var topLeftHit = this.within(other.topLeft())
  var topRightHit = this.within(other.topRight())
  var bottomLeftHit = this.within(other.bottomLeft())
  var bottomRightHit = this.within(other.bottomRight())

  var xContained = other.right() >= this.right() && other.left() < this.left()
  var yContained = other.top() >= this.top() && other.bottom() < this.bottom()

  if(xContained && yContained){
    return true;
  }

  var leftHit = (other.left() <= this.right() && other.left() > this.left())
  var rightHit = (other.right() <= this.right() && other.right() > this.left())
  var bottomHit = (other.bottom() <= this.top() && other.bottom() > this.bottom())
  var topHit = (other.top() <= this.top() && other.top() > this.bottom())

  return (yContained && (leftHit || rightHit)) || (xContained && (bottomHit || topHit))
}

module.exports = Bounds;
