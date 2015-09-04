
var TerrainModel = function(mongoose, connection){
  this.TileInfoSchema = new mongoose.Schema({
    origin : String,
    lowerleft : {x:Number, y:Number},
    topright : {x:Number, y:Number},
    sizeLL : {x:Number, y:Number},
    elevHist : Object,
    dataId : mongoose.Schema.Types.ObjectId
  })
  this.TileInfo = connection.model('TileInfo', this.TileInfoSchema);

  this.TileDataSchema = new mongoose.Schema({
    origin: String,
    lowerleft : {x:Number, y:Number},
    topright : {x:Number, y:Number},
    sizePx : {x:Number, y:Number},
    sizeLL : {x:Number, y:Number},
    elev : Buffer
  });
  this.TileData = connection.model('TileData', this.TileDataSchema);

  this.TileRenderInfoSchema = new mongoose.Schema({
    idx : Number,
    lowerleft : {x:Number, y:Number},
    topright : {x:Number, y:Number},
    sizePx : {x:Number, y:Number},
    sizeLL : {x:Number, y:Number},
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
  this.TileRenderInfo = connection.model('TileRenderInfo', this.TileRenderInfoSchema);

  this.CrawlSchema = new mongoose.Schema({
    path : String,
    time : Number,
    downloadId: mongoose.Schema.Types.ObjectId
  });
  this.Crawl = connection.model('Crawl', this.CrawlSchema);

  this.DownloadSchema = new mongoose.Schema({
    url : String,
    start_time: Date,
    end_time: Date
    local_path : String
  })

  return this;
}

exports.TerrainModel = TerrainModel;
