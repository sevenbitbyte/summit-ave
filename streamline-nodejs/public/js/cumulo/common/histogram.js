var Histogram = function(bucktSize){
  this.scale = (bucktSize === "undefined") ? new Number(10) : new Number(bucktSize);
  this.values = {};
  this.min = undefined;
  this.max = undefined;
  return this;
}

Histogram.prototype.push = function(value){
  if(value !== 'undefined' && !isNaN(value)){
    var bucket = Math.round(value / this.scale) * this.scale;

    if(this.values[bucket] === undefined){
      this.values[bucket] = new Number(0);
    }

    this.values[bucket]++;

    if(this.min === undefined || this.min > value){
      this.min = value;
    }

    if(this.max === undefined || this.max < value){
      this.max = value;
    }
  }
}

Histogram.prototype.getBuckets = function(){
  var buckets = [];
  for(var idx in this.values){
    buckets.push(idx);
  }
  return buckets;
}
