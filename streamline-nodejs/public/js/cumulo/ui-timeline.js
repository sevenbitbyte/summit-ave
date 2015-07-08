CanvasRenderingContext2D.prototype.dashedLine = function (x1, y1, x2, y2, dashLen) {
    if (dashLen == undefined) dashLen = 2;
    this.moveTo(x1, y1);

    var dX = x2 - x1;
    var dY = y2 - y1;
    var dashes = Math.floor(Math.sqrt(dX * dX + dY * dY) / dashLen);
    var dashX = dX / dashes;
    var dashY = dY / dashes;

    var q = 0;
    while (q++ < dashes) {
        x1 += dashX;
        y1 += dashY;
        this[q % 2 == 0 ? 'moveTo' : 'lineTo'](x1, y1);
    }
    this[q % 2 == 0 ? 'moveTo' : 'lineTo'](x2, y2);
};
//And you can use this as

var Timeline = function(el,date){
  this.element = el
  this.ctx = ctx = el.getContext('2d')
  //this.ctx.canvas.width = this.widthPx()
  //this.ctx.canvas.height = this.heightPx()
  this.center_lock = false
  this.join_distance = 3600

  this.span = undefined
  this.span_width = 7*24*3600*1000;

  this.date = new Date()
  this.setDate(date)
  this.metrics = []

  this.lastRenderRequestTime = undefined;
  this.animateWatchDog = undefined
  this.renderState = 'idle'

  this.textSizeLarge = 24
  this.textSizeSmall = 18
  this.textFontFamily = 'monospace'

  $(this.element).bind('touchy-pinch', this._handlePinch.bind(this));
  $(this.element).bind('touchy-drag', this._handleDrag.bind(this));

  return this;
}

Timeline.prototype._handlePinch = function(evt, target, data){
  console.log(data)
  this.span_width = Math.max(7200000, Math.min(3600*1000*24*365*10, this.span_width + this.span_width * (data.previousScale - data.scale) * 2));
  this.setDate(this.date)
  this.animate()

  /*
  var deltaX =  data.currentPoint.x
  var deltaXMs = this._getTimeMsFromPx(deltaX)*/
  //var newDate = moment(this.date).add(deltaXMs)

  //this.setDate(newDate.toDate())*/

  //this.setDate(new Date(deltaXMs))
  //this.animate()
  //this.render()

  /*
  this.ctx.beginPath();
  this.ctx.lineWidth=2
  this.ctx.moveTo(2*data.currentPoint.x, 0)
  this.ctx.lineTo(2*data.currentPoint.x, this.heightPx())
  this.ctx.strokeStyle = 'rgba(255,255,0,1)'
  this.ctx.stroke();*/
}

Timeline.prototype._handleDrag = function(evt, phase, target, data){
  var deltaX =  data.lastMovePoint.x - data.movePoint.x
  var deltaXMs = this._getTimeMsFromPx(deltaX*2)
  var newDate = moment(this.date).add(deltaXMs)

  this.setDate(newDate.toDate())
  this.animate()
}

Timeline.prototype.setDate = function(date){
  this.date = date
  var start = moment(date).subtract(this.span_width/2, 'ms')
  var end = moment(date).add(this.span_width/2, 'ms')
  this.span = moment.twix(start, end)
}

Timeline.prototype.getCenterDate = function(){
  return this.date
}

Timeline.prototype._sortMetrics = function(){
  this.metrics.sort(function(a,b){
    if(a.idx > b.idx){
      return 1
    }
    if(a.idx < b.idx){
      return -1
    }
    return 0
  }.bind(this))
}

Timeline.prototype.addMetric = function(name, index, data_sort){
  if(!data_sort){
    data_sort = function(a,b){
      if(a.time.getTime() > b.time.getTime()){
        return 1
      }
      if(a.time.getTime() < b.time.getTime()){
        return -1
      }
      return 0
    }
  }

  var metric = {
    name: name,
    idx: index,
    active: true,
    data: [],
    data_sort: data_sort,
    regions: []
  }

  this.metrics.push(metric)
  this._sortMetrics()

  return metric;
}

Timeline.prototype.getMetric = function(name){
  for(var i=0; i<this.metrics.length; i++){
    if(this.metrics[i].name == name){
      return this.metrics[i]
    }
  }

  return null
}

Timeline.prototype.addRegion = function(metric_name, span){
  var metric = this.getMetric(metric_name)

  if(!metric){
    var idx = (this.metrics.length > 0) ? this.metrics[this.metrics.length-1].idx+1 : 0
    this.addMetric(metric_name, idx)
  }

  if(metric){
    metric.regions = metric.regions.concat(span)

    metric.regions.sort(function(a,b){
      if(a.start.isAfter(b.start)){
        return 1
      }
      if(a.start.isBefore(b.start)){
        return -1
      }
      return 0
    })
  }
}

Timeline.prototype.addData = function(metric_name, data){
  var metric = this.getMetric(metric_name)

  if(metric){
    metric.data = metric.data.concat(data)
    metric.data.sort(metric.data_sort)
  }
}


Timeline.prototype._timeToXPx = function(time){
  var durationMs = this.span.length('ms')
  var widthPx = this.widthPx()

  var factor = widthPx / durationMs

  return factor * -this.span.start.diff(time)
}

Timeline.prototype._getPixelWidthMs = function(timeMs){
  var durationMs = this.span.length('ms')
  var widthPx = this.widthPx()

  var factor = widthPx / durationMs

  return factor * timeMs
}

Timeline.prototype._getTimeMsFromPx = function(x){
  var durationMs = this.span.length('ms')
  var widthPx = this.widthPx()

  var factor = durationMs / widthPx

  return factor * x
}

Timeline.prototype.widthPx = function(){
  //return $(this.element).width()
  return this.ctx.canvas.width
}

Timeline.prototype.heightPx = function(){
  //return $(this.element).height()
  return this.ctx.canvas.height
}


Timeline.prototype.render = function() {
  /*if(this.renderState != 'idle' && this.renderState != 'animate'){
    return false;
  }*/

  //var initialState = this.renderState
  //this.renderState = 'render'
  //this.ctx.canvas.width = this.widthPx()
  //this.ctx.canvas.height = this.heightPx()
  this.ctx.clearRect(0, 0, this.ctx.canvas.width, this.ctx.canvas.height)
  this.ctx.fillStyle = 'rgba(255,255,255,0.1)'
  this.ctx.fillRect(0, this.textSizeLarge+2, this.ctx.canvas.width, this.ctx.canvas.height-this.textSizeLarge)

  var now = new Date()
  var renderRegions = []
  var renderData = []

  for(var i=0; i<this.metrics.length; i++){
    var metric = this.metrics[i]

    if(!metric.active){continue;}

    for(var r=0; r<metric.regions.length; r++){
      var region = metric.regions[r]
      if(this.span.contains(region.start) || this.span.contains(region.end) || region.contains(this.span.start) || region.contains(this.span.end)){
        //console.log(region)


        var startX = this._timeToXPx(region.start)
        var endX = this._timeToXPx(region.end)

        this.ctx.beginPath();
        this.ctx.lineWidth=9
        this.ctx.moveTo(startX, this.textSizeLarge*2)
        this.ctx.lineTo(endX, this.textSizeLarge*2)
        this.ctx.strokeStyle = 'rgba(0,0,255,1)'
        this.ctx.stroke()

        this.ctx.beginPath();
        this.ctx.lineWidth=5
        this.ctx.strokeStyle = 'rgba(255,255,255,1)'
        this.ctx.moveTo(startX, this.textSizeLarge*2)
        this.ctx.lineTo(endX, this.textSizeLarge*2)
        this.ctx.stroke()
      }
    }

    /*
    for(var d=0; d<metric.data.length; d++){
      var datum = metric.data[d]
      if(this.span.contains(datum.time) || this.span.contains(datum.time)){
        renderData.push(datum.time)
      }
    }*/
  }

  //Hours/days

  if(this._getPixelWidthMs(3600*1000*24*31) < 150){
    var iter = timeline.span.iterate('months');

    while(iter.hasNext()){
      var d = iter.next();
      var date = d.toDate().getDate()
      var month = d.toDate().getMonth()
      var year = d.toDate().getFullYear()
      var x = this._timeToXPx(d)

      if(month == 0){
        this.ctx.font = this.textSizeLarge + "px " + this.textFontFamily
        this.ctx.moveTo(x, 0)
        this.ctx.lineWidth=1
        this.ctx.fillStyle = 'rgba(255,255,255,1)'
        ctx.fillText(year, x, this.textSizeLarge-2)

        this.ctx.strokeStyle = 'rgba(178,233,226, 1)'
      }
      else{
        this.ctx.strokeStyle = 'rgba(100,156,196, 0.5)'
      }

      this.ctx.beginPath();
      this.ctx.lineWidth=1
      this.ctx.moveTo(x, this.textSizeLarge+1)
      this.ctx.lineTo(x, this.heightPx())
      this.ctx.stroke();
    }
  }
  else if(this._getPixelWidthMs(3600*1000*24) < 100){
    var iter = timeline.span.iterate('days');

    while(iter.hasNext()){
      var d = iter.next();
      var date = d.toDate().getDate()
      var x = this._timeToXPx(d)

      if(date == 1){
        var locale = "en-us"
        var month = d.toDate().toLocaleString(locale, { month: "long" });

        this.ctx.font = this.textSizeLarge + "px " + this.textFontFamily
        this.ctx.moveTo(x, 0)
        this.ctx.lineWidth=1
        this.ctx.fillStyle = 'rgba(255,255,255,1)'
        ctx.fillText(month, x, this.textSizeLarge-2)
        this.ctx.strokeStyle = 'rgba(178,233,226, 0.75)'
      }
      else if(this._getPixelWidthMs(3600*1000*24) < 30){
        var day = d.toDate().getDay()

        if(day == 0){
          var locale = "en-us"
          var month = d.toDate().toLocaleString(locale, { month: "long" });

          this.ctx.moveTo(x, 0)
          this.ctx.lineWidth=1
          this.ctx.strokeStyle = 'rgba(178,233,226, 0.1)'
        }
        else{
          continue;
        }
      }
      else{
        this.ctx.strokeStyle = 'rgba(178,233,226, 0.1)'
      }

      this.ctx.beginPath();
      this.ctx.lineWidth=1
      this.ctx.moveTo(x, this.textSizeLarge+1)
      this.ctx.lineTo(x, this.heightPx())
      this.ctx.stroke();
    }

  }
  else{
    var iter = timeline.span.iterate('hours');

    while(iter.hasNext()){
      var d = iter.next();
      var h = d.toDate().getHours()
      var x = this._timeToXPx(d)



      if(h == 0 && this._getPixelWidthMs(3600*1000*24) > 100){
        this.ctx.font = this.textSizeLarge + "px " + this.textFontFamily
        this.ctx.moveTo(x, 0)
        this.ctx.lineWidth=1
        this.ctx.fillStyle = 'rgba(255,255,255,1)'
        ctx.fillText(d.toDate().getDate(), x, this.textSizeLarge-2)

        this.ctx.strokeStyle = 'rgba(178,233,226,0.75)'
      }
      else if(this._getPixelWidthMs(3600*1000) < 30){
        continue;
      }
      else if(h == 12){
        if(timeline.span.length() < 3*24*3600*1000){
          //span > 3 days
          this.ctx.font = this.textSizeLarge + "px sans-serif"
          this.ctx.moveTo(x, 0)
          this.ctx.fillStyle = 'rgba(255,255,255,1)'
          ctx.fillText('12 pm', x, this.textSizeLarge-2)
        }

        this.ctx.strokeStyle = 'rgba(178,233,226,0.5)'
      }
      else{
        if(timeline.span.length() < 3*24*3600*1000){
          //span < 3 days
          this.ctx.font = this.textSizeSmall + "px sans-serif"
          this.ctx.moveTo(x, 0)
          this.ctx.fillStyle = 'rgba(255,255,255,0.5)'
          var text = (h < 12) ? h+' am' : (h-12)+' pm'
          ctx.fillText(text, x, this.textSizeLarge-2)
        }

        this.ctx.strokeStyle = 'rgba(178,233,226, 0.2)'
      }



      this.ctx.beginPath();
      this.ctx.lineWidth=1
      this.ctx.moveTo(x, this.textSizeLarge+1)
      this.ctx.lineTo(x, this.heightPx())
      this.ctx.stroke();
    }
  }



  var nowX = this._timeToXPx(now)



  this.ctx.beginPath();
  this.ctx.lineWidth=3
  this.ctx.dashedLine(nowX, this.textSizeLarge+1, nowX, this.heightPx(), 3);
  //this.ctx.moveTo(nowX, this.textSizeLarge)
  //this.ctx.lineTo(nowX, this.heightPx())
  this.ctx.strokeStyle = 'rgba(255,0,0,0.5)'
  this.ctx.stroke();

  /*this.ctx.beginPath();
  this.ctx.lineWidth=1
  this.ctx.dashedLine(nowX, this.textSizeLarge, nowX, this.heightPx(), 2);
  //this.ctx.moveTo(0, this.textSizeLarge)
  //this.ctx.lineTo(this.widthPx(), this.textSizeLarge)
  this.ctx.strokeStyle = 'rgba(178,233,226,0.1)'
  this.ctx.stroke();*/

  /*if(this.renderState == 'render'){
    this.renderState = initialState
  }

  if(this.lastRenderRequestTime > now){
    this.render();
  }*/

  return true;
}


Timeline.prototype.animate = function() {
  this.lastRenderRequestTime = (new Date()).getTime()
  if(this.renderState != 'animate'){
    this.renderState = 'animate'
    console.log('Timeline: animating')
    this.animateCb();
  }
}

Timeline.prototype.animateCb = function(){
  if(this.renderState !== undefined && this.renderState != 'animate'){
    return;
  }

  if(this.animateWatchDog === undefined){
    this.animateWatchDog = setTimeout(function(){
      var now = (new Date()).getTime()
      if(now-this.lastRenderRequestTime > 500 && this.renderState != 'idle'){
        this.renderState='idle';
        console.log('Timeline: stopping animation')
      }
      else{
        console.log('Timeline: not stopping')
        this.animate()
      }
      this.animateWatchDog = undefined
    }.bind(this), 1000)
  }

  this.render()

  //setTimeout(this.animateCb.bind(this), 80);
  requestAnimationFrame(this.animateCb.bind(this))
}
