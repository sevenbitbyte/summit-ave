
var TerrainModel = function(mongoose, connection){
  this.TileInfoSchema = new mongoose.Schema({
    origin : String,
    lowerleft : {x:Number, y:Number},
    topright : {x:Number, y:Number},
    sizeLL : {x:Number, y:Number},
    elevHist : Object
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

  this.CrawlSchema = new mongoose.Schema({
    path : String,
    time : Number
  });
  this.Crawl = connection.model('Crawl', this.CrawlSchema);

  return this;
}


module.exports = TerrainModel;
