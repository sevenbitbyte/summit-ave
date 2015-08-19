var fs = require('fs');
var async = require('async');
var Wreck = require('wreck');

var SRTM_BASE_URL = "http://srtm.csi.cgiar.org/SRT-ZIP/SRTM_v41/SRTM_Data_ArcASCII/srtm_"
var CONCURRENT_DOWNLOADS = 3;
var downloadQueue = []


function n(n){
    return n > 9 ? "" + n: "0" + n;
}

for(var i=1; i<4; i++){
  for (var j = 35; j< 62; j++) {
    downloadQueue.push( n(j) + '_' + n(i) )   //Tile name
  }
}

var dirExists = function(path){
  try{
    var stats = fs.lstatSync(path)

    if(stats.isDirectory()){
      return true;
    }
  }
  catch(e){
    return false;
  }
}

var fileExists = function(path){
  try{
    var stats = fs.lstatSync(path)

    if(stats.isFile()){
      return true;
    }
  }
  catch(e){
    return false;
  }
}

var downloadItem = function(tile, callback){

  var url = SRTM_BASE_URL + tile + '.zip'

  if(fileExists('downloads/' +tile+ '.zip') || dirExists('downloads/' +tile+ '.zip')){
    console.log('Tile-' +tile +': Already downloaded')
    return callback(null, tile);
  }

  //console.log('Tile-' + tile + ' Downloading: ' + url);

  Wreck.get(url, function(err,res,payload){
    if(err){
      //console.log('Tile-' + tile + ' Wreck error: ' + err)
      return callback(err);
    }

    if(res.statusCode !== 200){
      //console.log('Tile-' + tile + ' HTTP error: ' + res.statusCode)
      return callback(res.statusCode);
    }

    console.log('Tile-' + tile + ' Saving')

    fs.writeFile('downloads/'+tile+'.zip', payload, function(err){
      if(err){
        console.log('Tile-' + tile + ' File error: ' + err)
        return callback(err);
      }

      console.log('Tile-' + tile + ': Downloaded ' + url);
      callback(null, tile);
    })

  });
}

var unzipTile = function(tile, callback){
  callback();
}

var downloadThread = function(callback){
  async.whilst(
    function(){
      return downloadQueue.length > 0
    },
    function(next){
      var tile = downloadQueue.pop();
      async.waterfall([
        function(next){
          downloadItem(tile, next);
        },
        unzipTile,
      ],
      function(err){
        if(err){
          console.log('Tile-' + tile + ' Error: ' + err)
        }

        return next();
      }
    )

    },
    function(err){  //done or error
      if(err){
        callback(err);
      }


      callback();
    }
  )
}

console.log(downloadQueue)

async.parallel([
  downloadThread,
  downloadThread,
  downloadThread,
  downloadThread
],
function(){
  console.log('Done')
}
)
