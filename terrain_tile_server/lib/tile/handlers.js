
var TerrainDataSource = require('../db/terrain_datasource');
var dataSource = new TerrainDataSource({});


exports.getTileInfo = function (request, reply) {
  var model = dataSource.getModel();

  console.log('TileInfo');

  var query = model.TileInfo.find({ $and: [
    {'lowerleft.x' : { $lt : req.params.long}},
    {'lowerleft.y' : { $lt : req.params.lat}},
    {'topright.x' : { $gt : req.params.long}},
    {'topright.y' : { $gt : req.params.lat}},
    ]}).limit(1);
  var promise = query.exec();
  promise.then(
    function(infoList){
      console.log('Found ' + infoList.length + ' tiles');
      res.send(JSON.stringify(infoList));
    }
  );
}

exports.getTileElevPng = function (request, reply) {

}


exports.getTileData = function (request, reply) {

}
