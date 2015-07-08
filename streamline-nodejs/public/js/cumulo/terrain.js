var Terrain = {};

/**
 *  A client for downloading and caching raw terrain data
 *
 *  @param  {string} baseaddress - The base address of the service host
 */
Terrain.Datasource = function(baseaddress){
  this._address = baseaddress + '/terrain';

  return this;
}

/**
 *  Downloads tile data using the specified coordinates
 *
 *  @param  {number} lat - Latitude
 *  @param  {number} long - Longitude
 *  @param  {function} cb - Success callback of the form fn(result)
 *  @param  {function} err - Error callback of the form fn(errorMsg)
 */
Terrain.Datasource.prototype.fetchTileInfo = function(lat, long, cb, errCb){
  var endpoint = this._address + '/tile/info/' + lat + ',' + long + '.json';

  return $.ajax({
        type: "GET",
        dataType: 'json',
        url: endpoint,
        error : function(xhr, status, error){
          if(errCb !== undefined){
            errCb("getTileInfo:"+error);
          }
          else{
            console.log("Terrain.Datasource.getTileInfo() - Error: Failed to call endpoint=[" + endpoint + "] error=[" + error + "]");
          }
        }.bind(this),
        success: function(data){
           var result=data;

           if(cb !== undefined){
             cb(result);
           }
           else{
             console.log("Terrain.Datasource.getTileInfo() - " + data);
           }

        }
  });
}

/**
 *  Downloads all tile data using the specified coordinates
 *
 *  @param  {number} lat - Latitude
 *  @param  {number} long - Longitude
 *  @param  {function} cb - Success callback of the form fn(result)
 *  @param  {function} err - Error callback of the form fn(errorMsg)
 */
Terrain.Datasource.prototype.fetchTile = function(lat, long, cb, errCb){
  this.fetchTileInfo(lat, long, function(infoList){
    if(infoList !== undefined){
      if(infoList.length == 1){
        //Success

        var image = new Image();
        image.onload = function(){ // always fires the event.
          var imgCanvas = document.createElement("canvas");
          imgCanvas.width = image.width;
          imgCanvas.height = image.height;
          imgCanvas.getContext("2d").drawImage(image, 0, 0);

          //imgData = imgCanvas.getContext("2d").getImageData(0, 0, imgCanvas.width, imgCanvas.height).data;

          cb([{info: infoList[0], elev: imgCanvas}]);
        };
        // handle failure
        image.onerror = function(){
          errCb('fetchTile:image load error');
        };

        image.src = this._address + '/tile/elev/'+lat+','+long+'.png';
      }
      else{
        msg = "fetchTile:length[" + infoList.length + "]";
      }
    }
    else{
      msg = "fetchTile:undefined list";
    }

    if(errCb !== undefined){ errCb(msg); }
  }.bind(this), errCb);
}

Terrain.TileCollection = function(){
  this.quad = new QuadTree({
    x:-180,
		y:-90,
		width:360,
		height:180
	});

  return this;
}

Terrain.TileCollection.prototype.get = function(bounds){
  var options = {x : bounds.lat, y: bounds.long};

  if(bounds.width !== undefined){ options.width = bounds.width }
  if(bounds.height !== undefined){ options.height = bounds.height }

  return this.quad.retrieve(options);
}

Terrain.TileCollection.prototype.set = function(tile){
  this.quad.insert({x: tile.info.lowerleft.x, y: tile.info.lowerleft.y, width: tile.info.sizeLL.x, height: tile.info.sizeLL.y, data: tile});
}

Terrain.TileManager = function(datasource){
  this._datasource = datasource;
  this._tiles = new Terrain.TileCollection();

  return this;
}


/**
 *  Get an array of terrain tiles by bounds
 *
 *  @param  {Object}  bounds - Bounds to search with in
 *  @param  {Function} cb - Success callback fn( [ {info: {}, elev: {} }])
 *  @param  {Function} errCb - Error callback
 */
Terrain.TileManager.prototype.get = function(bounds, cb, errCb){
  //Look for tile in collection
  var tiles = this._tiles.get(bounds);

  if(tiles.length > 0){
    //Tile found in quadtree
    return cb([tiles[0].data]);
  }
  else{
    //Request tile from data source
    return this._datasource.fetchTile(bounds.y, bounds.x, function(tiles){
      for(idx in tiles){
        this._tiles.set(tiles[idx]);
      }
      return cb(tiles);
    }.bind(this), errCb);
  }
}

Terrain.Tile = function(tile){
  this.tile = tile;
  return this;
}

//Terrain.Tile.prototype.
