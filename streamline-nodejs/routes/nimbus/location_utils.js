exports.getSegmentByTime = function(Segments){
  return function(start, end){
    Segments.find({},
      function(err, result){
        //
      }
    );
  };
}

exports.insertSegment = function(Segments){
  return function(seg){
    //
  }
}
