var RunningStats = function(options){
  this.compute_stddev = (options.stddev !== undefined) ? options.stddev : false;
  this.compute_mean = ((options.mean !== undefined) ? options.mean : true) || this.compute_stddev;
  this.compute_median = (options.median !== undefined) ? true : false;

  this.size = options.size;
  this._dataType = (options.type !== undefined) ? options.type : Uint8Array;

  this._n = 0;
  if(this.compute_mean){
    this._oldM = new this._dataType(this.size);
    this._newM = new this._dataType(this.size);
  }

  if(this.compute_stddev){
    this._oldS = new this._dataType(this.size);
    this._newS = new this._dataType(this.size);
  }

  this._max = new this._dataType(this.size);
  this._min = new this._dataType(this.size);

  if(this.compute_median){
    this._filterSize = options.median;
    this._median = [];
    for(i=0; i<this.size; i++){
      this._median[i] = [];
    }
  }

  return this;
}

/*
 *  Add array of data points into running statistics calulation
 *
 *  @param  data  An array of data points
 */
RunningStats.prototype.push = function(data){
  this._n++

  if(this._n == 1){
    for(i=0; i < this.size; i++){
      this._oldM[i] = this._newM[i] = data[i]
      this._oldS[i] = 0.0;

      this._min[i] = data[i];
      this._max[i] = data[i];

      if(this.compute_median){
        if(this._median[i].length >= this._filterSize){
          this._median[i].pop();
        }

        this._median[i].push(data[i]);
      }
    }
  }
  else{
    for(i=0; i < this.size; i++){
      this._newM[i] = this._oldM[i] + (data[i] - this._oldM[i])/this._n
      this._newS[i] = this._oldS[i] + (data[i] - this._oldM[i]) * (data[i] - this._newM[i])

      this._oldM[i] = this._newM[i]
      this._oldS[i] = this._newS[i]

      if(this._min[i] > data[i]){ this._min[i] = data[i]; }
      if(this._max[i] < data[i]){ this._max[i] = data[i]; }

      if(this.compute_median){
        if(this._median[i].length >= this._filterSize){
          this._median[i].pop();
        }

        this._median[i].push(data[i]);
      }
    }
  }
}

RunningStats.prototype.count = function(){
  return this._n;
}

RunningStats.prototype.median = function(idx){
  if(idx === undefined){
    var arr = [];

    for(i=0; i<this.size; i++){
      arr[i] = this.median(i);
    }

    return arr;
  }
  else{
    return getMedian( this._median[i], this._filterSize );
  }
}

RunningStats.prototype.range = function(idx){
  if(idx === undefined){
    var range = {
      max : [],
      min : []
    };

    for(i=0; i<this.size; i++){
      var r = this.range(i);

      range.max[i] = r.max;
      range.min[i] = r.min;
    }

    return range;
  }
  else{
    return {
      max : this._max[i],
      min : this._min[i]
    }
  }
}

RunningStats.prototype.mean = function(idx){
  if(idx === undefined){
    var arr = [];

    for(i=0; i<this.size; i++){
      arr[i] = this.mean(i);
    }

    return arr;
  }
  else{
    return (this._n > 0) ? this._newM[idx] : 0.0;
  }
}

RunningStats.prototype.variance = function(idx){
  if(idx === undefined){
    var arr = [];

    for(i=0; i<this.size; i++){
      arr[i] = this.variance(i);
    }

    return arr;
  }
  else{
    return (this._n > 1) ? this._newS[idx] / (this._n - 1) : 0.0;
  }
}

RunningStats.prototype.standardDeviation = function(idx){
  if(idx === undefined){
    var arr = [];

    for(i=0; i<this.size; i++){
      arr[i] = this.standardDeviation(i);
    }

    return arr;
  }
  else{
    return Math.sqrt( this.variance(idx) );
  }
}
