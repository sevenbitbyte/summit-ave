var fs = require('fs');

exports.niceNumber = function (n){
    return n > 9 ? "" + n: "0" + n;
}

exports.dirExists = function(path){
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


exports.fileExists = function(path){
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
