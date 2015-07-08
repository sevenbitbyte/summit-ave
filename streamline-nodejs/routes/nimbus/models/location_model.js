
var LocationModel = function(mongoose){
  var TrackSegmentSchema = mongoose.Schema({
    originObj: String,
    start_time : Number,
    end_time : Number,
    lowerleft: {x: Number, y: Number},
    upperright: {x: Number, y: Number},
    centroid : {x: Number, y: Number},
    ele_hist : {},
    speed_hist : {},
    course_hist : {}
  });
  this.TrackSegment = mongoose.model('TrackSegment', TrackSegmentSchema);


  var TrackPointListSchema = mongoose.Schema({
    segId: { type : mongoose.Schema.ObjectId, ref : 'TrackSegment' },
    points : [{
      time : Number,
      pos_ll : { x: Number, y: Number},
      speed : Number,    //Velocity
      src : String,
      ele : Number,
      geoidheight: Number,
      sat_count : Number,
      hdop: Number,
      vdop: Number,
      pdop: Number,
      course: Number
    }]
  });
  this.TrackPointList = mongoose.model('TrackPointList', TrackPointListSchema);

  this.CrawlSchema = new mongoose.Schema({
    path : String,
    time : Number
  });
  this.Crawl = mongoose.model('Crawl', this.CrawlSchema);

  return this;
}

exports.LocationModel = LocationModel;
