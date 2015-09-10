var Mongoose = require('mongoose')
var db_connection = Mongoose.createConnection('mongodb://localhost/streamline-terrain');
var Utils = require('../methods/utils');

var TileInfoSchema = new Mongoose.Schema({
  lowerleft : {x:Number, y:Number},
  topright : {x:Number, y:Number},
  sizeLL : {x:Number, y:Number},
  elevHist : Object,
  crawlId: Mongoose.Schema.Types.ObjectId,
  dataId : Mongoose.Schema.Types.ObjectId
})

TileInfoSchema.statics.findByBounds = function(bounds, cb){
  return this.find({}, cb)
}

var TileDataSchema = new Mongoose.Schema({
  lowerleft : {x:Number, y:Number},
  topright : {x:Number, y:Number},
  sizePx : {x:Number, y:Number},
  sizeLL : {x:Number, y:Number},
  elev : Buffer
});

var TileRenderSchema = new Mongoose.Schema({
  tileSizePx : Number,
  tileSizeLL : Number,
  roi : {
    lowerleft : {x:Number, y:Number},
    topright : {x:Number, y:Number}
  },
  format: String
})


var TileRenderInfoSchema = new Mongoose.Schema({
  idx : Number,
  lowerleft : {x:Number, y:Number},
  topright : {x:Number, y:Number},
  sizePx : Number,
  sizeLL : Number,
  path : String,
  format: String,
  parent: {type: Object, default: undefined}

  /*
  {
    sizeLL : Number,
    sizePx : Number,
    format: String
  }
  */
});

var CrawlSchema = new Mongoose.Schema({
  path : String,
  start_time: Date,
  end_time: Date,
  downloadId: Mongoose.Schema.Types.ObjectId
});

/*
var DownloadRequestSchema = new Mongoose.Schema({
  host: String,
  start_time: Date,
  end_time: Date,
  tiles: [String],
  base_uri: String,
  ending: String,
  download_path: String
})*/

var DownloadSchema = new Mongoose.Schema({
  uri : String,
  tile: String,
  start_time: Date,
  end_time: Date,
  local_path : String
});

DownloadSchema.methods.fileExists = function(){
  return this.local_path && this.local_path.length > 0 && Utils.fileExists(this.local_path);
}


exports.TileInfo = db_connection.model('TileInfo', TileInfoSchema);
exports.TileData = db_connection.model('TileData', TileDataSchema);
exports.TileRenderInfo = db_connection.model('TileRenderInfo', TileRenderInfoSchema);
exports.Crawl = db_connection.model('Crawl', CrawlSchema);
exports.Download = db_connection.model('Download', DownloadSchema);
