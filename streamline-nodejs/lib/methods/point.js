var Hoek = require('hoek')

var Point = function(options){
  this.x = Hoek.reach(options, 'x', undefined, true)
  this.y = Hoek.reach(options, 'y', undefined, true)

  return this;
}

module.exports = Point;
