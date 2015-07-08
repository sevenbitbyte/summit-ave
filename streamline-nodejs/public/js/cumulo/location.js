
var Location = {};

Location.Gpx = function(gpxJson){
  this.segments = [];
  this.time = (typeof gpxJson.time === 'undefined') ? null : Date(gpxJson.time);
  this.creator = (typeof gpxJson.creator === 'undefined') ? null : String(gpxJson.creator);

  if(!Array.isArray(gpxJson.trk.trkseg)){
    var seg = new Location.Gpx.TrkSeg( gpxJson.trk.trkseg );
    this.segments.push(seg);
  }
  else{
    //Create segments
    for(var trkSegIdx in gpxJson.trk.trkseg){
      //var seg = new Segments();
      var seg = new Location.Gpx.TrkSeg( gpxJson.trk.trkseg[trkSegIdx] );

      this.segments.push(seg);
    }
  }

  console.log('segment translation complete');
  console.log(this.segments);

  return this;
}

Location.Gpx.TrkPt = function(opt){
  this.time = (typeof opt.time === "undefined") ? null : new Date(opt.time);
  this.elev = (typeof opt.ele === "undefined") ? undefined : Number(opt.ele);
  this.geoidheight = (typeof opt.geoidheight === "undefined") ? null : Number(opt.geoidheight);
  this.loc = [ Number(opt.lon), Number(opt.lat) ];
  this.speed  = (typeof opt.speed === "undefined") ? undefined : Number(opt.speed);
  this.course = (typeof opt.course === "undefined") ? undefined : Number(opt.course);
  this.hdop = (typeof opt.hdop === "undefined") ? null : Number(opt.hdop);
  this.vdop = (typeof opt.vdop === "undefined") ? null : Number(opt.vdop);
  this.pdop = (typeof opt.pdop === "undefined") ? null : Number(opt.pdop);
  this.sat_count = (typeof opt.sat === "undefined") ? null : Number(opt.sat);
  this.sats = (typeof opt.satellites === "undefined") ? null : String(opt.satellites);;
  this.src = (typeof opt.src === "undefined") ? null : String(opt.src);
  this.label  = (typeof opt.label === "undefined") ? null : String(opt.label);;

  return this;
}

Location.Gpx.TrkSeg = function(trkSeg){
  this.start_time = undefined;
  this.end_time = undefined;
  this.duration_ms = undefined;
  this.bounds = [[undefined, undefined], [undefined, undefined]];  //[x1,y1], [x2,y2]
  this.ele_hist = new Histogram();
  this.speed_hist = new Histogram();
  this.course_hist = new Histogram();
  this.points=[];

  if(trkSeg === "undefined"){throw 'Invalid track segment';}

  var startMoment = undefined;
  var endMoment = undefined;

  if(!Array.isArray(trkSeg.trkpt)){
    var trkPt = new Location.Gpx.TrkPt(trkSeg.trkpt);
    this.points.push(trkPt);

    this.start_time = new Date(trkPt.time);
    startMoment = new moment(new Date(trkPt.time));
    this.end_time = new Date(trkPt.time);
    endMoment = new moment(new Date(trkPt.time));

    this.updateBounds(trkPt);
  }
  else{
    for(var trkPtIdx in trkSeg.trkpt){
      var trkPt = new Location.Gpx.TrkPt(trkSeg.trkpt[trkPtIdx]);
      this.points.push(trkPt);

      this.ele_hist.push.bind(this.ele_hist)(trkPt.elev);
      this.speed_hist.push.bind(this.speed_hist)(trkPt.speed);
      this.course_hist.push.bind(this.course_hist)(trkPt.course);

      if(startMoment === undefined || startMoment.diff(trkPt.time) > 0){
        this.start_time = new Date(trkPt.time);
        startMoment = new moment(new Date(trkPt.time));
      }

      if(endMoment === undefined || endMoment.diff(trkPt.time) < 0){
        this.end_time = new Date(trkPt.time);
        endMoment = new moment(new Date(trkPt.time));
      }

      this.updateBounds(trkPt);
    }
  }

  this.duration_ms = (new moment.duration(endMoment.diff(startMoment))).milliseconds();

  return this;
}

Location.Gpx.TrkSeg.prototype.updateBounds = function(trkPt){
  //Update lower x bound
  if(this.bounds[0][0] == undefined || trkPt.loc[0] < this.bounds[0][0]){
    this.bounds[0][0] = trkPt.loc[0];
  }

  //Update lower y bound
  if(this.bounds[0][1] == undefined || trkPt.loc[1] < this.bounds[0][1]){
    this.bounds[0][1] = trkPt.loc[1];
  }

  //Update upper x bound
  if(this.bounds[1][1] == undefined || trkPt.loc[0] > this.bounds[1][0]){
    this.bounds[1][0] = trkPt.loc[0];
  }

  //Update upper y bound
  if(this.bounds[1][1] == undefined || trkPt.loc[1] > this.bounds[1][1]){
    this.bounds[1][1] = trkPt.loc[1];
  }
}


Location.GpxCollection = function(gpxArray){
  this.gpx = [];
  this.start_time = undefined;
  this.end_time = undefined;
  this.duration_ms = undefined;
  this.middle_time = undefined;
  this.it = undefined;
}


Location.GpxCollection.prototype.addGpx = function(gpx){
  for(segIdx in gpx.segments){
    this.addSegment(gpx.segments[segIdx]);
  }
  this.gpx.push(gpx);
}

Location.GpxCollection.prototype.addSegment = function(segment){
  if(this.start_time === undefined){
    this.start_time = new Date(segment.start_time.getTime());
  }
  else if(this.start_time > segment.start_time){
    this.start_time = new Date(segment.start_time.getTime());
  }

  if(this.end_time === undefined){
    this.end_time = new Date(segment.end_time.getTime());
  }
  else if(this.end_time < segment.end_time){
    this.end_time = new Date(segment.end_time.getTime());
  }

/*
  var new_duration_ms = new moment.duration( this.end_time.diff(this.start_time) );
  if(this.duration_ms === undefined){
    this.duration_ms = this.start_time.clone()
  }
    var new_middle_time = this.start_time.clone().add('ms', this.duration_ms.asMilliseconds()/2);
  }

  if(this.it === undefined || new_middle_time.toString() != this.middle_time.toString()){
    this.duration_ms = new_duration_ms;
    this.middle_time = new_middle_time;

    var middle_ms = moment.duration( this.middle_time ).asMilliseconds();
    var it_options = {
      startKey: 'start',
      endKey: 'end'
    }

    this.it = new IntervalTree(middle_ms, it_options);
  }
*/

  /*gpxInterval = {
    start: moment.duration(this.start_time).asMilliseconds(),
    end: moment.duration(this.start_time).asMilliseconds(),
    g: gpx,
    s:
  }*/
}


Location.GpxCollection.prototype._rebuildIt = function(){
  //
}
