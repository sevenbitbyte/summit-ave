var Histogram = function(options){
  this.scale = (options.scale === undefined) ? 10 : options.scale;
  this.values = (options.values === undefined) ? {} : options.values;
  this.min = (options.min === undefined) ? undefined : options.min;
  this.max = (options.max === undefined) ? undefined : options.max;;
  return this;
}

Histogram.prototype.push = function(value){
  if(value !== undefined && !isNaN(value)){
    var bucket = Math.round(value / this.scale) * this.scale;

    if(this.values[bucket] === undefined){
      this.values[bucket] = 0;
    }

    this.values[bucket]++;

    if(this.min === undefined || this.min > value){
      this.min = value;
    }

    if(this.max === undefined || this.max < value){
      this.max = value;
    }
  }
  else{
    console.log('Histogram.push');
    throw value;
  }
}

Histogram.prototype.getBuckets = function(){
  var buckets = [];
  for(var idx in this.values){
    buckets.push(idx);
  }
  return buckets;
}

Histogram.prototype.value = function(n){
  var value = (n - this.min) / (this.max - this.min);

  if(value > 1 || value < 0){
    console.log("Strange value " + value);
    console.log(n)
    console.log(this.min)
    console.log(this.max)
    console.log(this.scale)
    console.log(this.values)
    throw value;
  }


  return value;
}

module.exports = Histogram;
