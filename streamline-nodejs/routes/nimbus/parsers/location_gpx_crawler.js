var fs = require('fs');
var xml2js = require('xml2js');
var moment = require('moment');



function readGpx(dispatchData, filePath, cb){
  try{
    var parser = new xml2js.Parser();

    if(fs.existsSync(filePath)){
      console.log("Found: " + filePath);
      fs.readFile(filePath,
        function(err, data) {
            parser.parseString(data,
              function (err, result) {
                  cb(dispatchData, err, result);
              }
            );
          }
      );
    }
    else{
      console.log("File not found: " + filePath);
      cb(dispatchData, "originObj not found", {});
    }
  }
  catch(err){
    cb(dispatchData, "No such gpx: " + filePath + " Error: " + err, undefined);
  }
}
