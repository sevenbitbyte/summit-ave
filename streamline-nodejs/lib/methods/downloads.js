var fs = require('fs')
var Hoek = require('hoek')
var Async = require('async')
var Wreck = require('wreck')
var Utils = require('./utils')
var Debug = require('debug')('lib.Downloads')
var TerrainModel = require('../models/terrain_model')

var SRTM_BASE_URL = "http://srtm.csi.cgiar.org/SRT-ZIP/SRTM_v41/SRTM_Data_ArcASCII/srtm_"
var DEFAULT_LOCAL_PATH = './download/'
var CONCURRENT_DOWNLOADS = 3;
var ALL_TILES = []


for(var i=1; i<4; i++){
  for (var j = 35; j< 62; j++) {
    ALL_TILES.push( Utils.niceNumber(j) + '_' + Utils.niceNumber(i) )   //Tile name
  }
}


var Downloads = function(options, finishedCb, progressCb){

  this.options = {
    tiles : Hoek.reach(options, 'tiles', {default: ALL_TILES}),
    baseUri : Hoek.reach(options, 'base_uri', {default: SRTM_BASE_URL}),
    ending : Hoek.reach(options, 'ending', {default: '.zip'}),
    downloadPath : Hoek.reach(options, 'local_path', {default: DEFAULT_LOCAL_PATH}),
    finished : finishedCb,
    progress : progressCb,
    concurrentDownloads : CONCURRENT_DOWNLOADS
  }

  console.log(this.options)

  this.queue = Hoek.clone(this.options.tiles);
  this.complete = [];
  this.failed = [];
  return this;
}

Downloads.prototype.start = function(done){
  var tasks = [];

  for(i=0; i < this.options.concurrentDownloads; i++){
    tasks.push(this._downloadThread.bind(this));
  }

  Async.parallel(
    tasks,
    function(err){

      done(err,this)
      return (this.options.finished) ? this.options.finished(err,this) : {}
    }.bind(this)
  )
}

Downloads.prototype.__storeOrUpdateDownload = function(download, callback){

  TerrainModel.Download.findOne(
    {
      tile: download.tile,
      uri: download.uri,
      local_path: download.local_path
    },
    function(err, results){

      console.log('results')

      if(err){
        Debug('Download.findOne Tile-' + download.tile + ' Err: ' + err)
        return callback(err, download)
      }

      if(!results){
        Debug('Saving download record Tile-'+download.tile)
        var dl = TerrainModel.Download(download);
        return dl.save(function(err){
          callback(err, dl)
        })
      }

      Debug('findOne - results: ' + results)
      callback(err, results)
    }
  )
}

Downloads.prototype._downloadItem = function(tile, callback){

  var self = this;

  tile = Hoek.clone(tile)

  console.log(this)

  var remotePath = this.options.baseUri + tile + this.options.ending
  var localPath = this.options.downloadPath + tile + this.options.ending;
  var startTime = new Date();

  if(Utils.fileExists(localPath)){
    Debug('Tile-' +tile +' Already downloaded: ' + localPath)

    // Store/update info
    this.__storeOrUpdateDownload(
      {
        tile: tile,
        uri: remotePath,
        local_path: localPath,
        start_time: startTime,
        end_time: new Date()
      },
      callback
    )
  }
  else{
    //Download
    Debug('Downloading Tile-'+tile+ ': ' + remotePath)
    Wreck.get(remotePath, function(err,res,payload){
      if(err){
        Debug('Tile-' + tile + ' Wreck error: ' + err)
        return callback(err, tile);
      }

      if(res.statusCode !== 200){
        Debug('Tile-' + tile + ' HTTP error: ' + res.statusCode)
        return callback(res.statusCode, tile);
      }

      Debug('Tile-' + tile + ' Saving: ' + localPath)

      fs.writeFile(localPath, payload, function(err){
        if(err){
          Debug('Tile-' + tile + ' File save error: ' + err)
          return callback(err, tile);
        }

        Debug('Tile-' + tile + ': Downloaded ' + remotePath);

        // Store/update info
        self.__storeOrUpdateDownload(
          {
            tile: tile,
            uri: remotePath,
            local_path: localPath,
            start_time: startTime,
            end_time: new Date()
          },
          callback
        )
      })
    });
  }
}

Downloads.prototype._downloadThread = function(callback){
  var self = this;

  Async.whilst(
    function(){
      return self.queue.length > 0
    },
    function(next){
      var tile = self.queue.pop();

      return self._downloadItem.bind(self)(tile, function(err, tile){

        var tileName = tile
        if(typeof tile === 'object'){
          tileName = tile.tile
        }

        if(!err){
          self.complete.push(tileName)
        }
        else{
          self.failed.push(tileName)
        }

        (self.options.progress) ? self.options.progress(err,tile) : {}
        return next();

      })
    },
    function(err){  //done or error
      if(err){
        callback(err);
      }


      callback();
    }
  )
}

module.exports = Downloads
