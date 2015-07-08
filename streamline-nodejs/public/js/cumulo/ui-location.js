Location.Ui = {};


Location.Ui.GpxThumb = function(opt){
  this.gpxGroup = new THREE.Object3D();



  //this.meshes = [];
  //this.geometry = [];
  //this.material = [];

  if(opt !== undefined){
    this.terrain = (opt.terrain !== undefined) ? opt.terrain : undefined;
  }

  this.size = {
    x: 10,
    y: 1,
    z: 10
  };

  this.group = new THREE.Object3D();
  this.group.userData = this;



  var size = 5;
  var step = 1;
  var gridHelper = new THREE.GridHelper( size, step );

  gridHelper.translateY(-0.5);
  //gridHelper.rotateOnAxis( new THREE.Vector3(1,0,0), -Math.PI * 0.5);
  gridHelper.setColors('yellow', 'white');
  gridHelper.material.opacity = 0.175;
  gridHelper.material.transparent = true;


  var axisHelper = new THREE.AxisHelper( 0.25 );
  axisHelper.translateX(-5);
  axisHelper.translateY(-0.5);
  axisHelper.translateZ(-5);
  axisHelper.geometry.computeBoundingBox();

  //this.group.add( axisHelper );
  this.group.add( gridHelper );

  return this;
}

Location.Ui.GpxThumb.prototype.updateTransforms = function(){
  var gpxBounds = new THREE.Box3();

  for(var idx in this.gpxGroup.children){
    gpxBounds.union( this.gpxGroup.children[idx].geometry.boundingBox );
  }


  var gpxSize = gpxBounds.size();
  this.gpxGroup.scale.x = Math.min(this.size.x / gpxSize.x, this.size.z / gpxSize.z);
  this.gpxGroup.scale.y = this.size.y / gpxSize.y;
  this.gpxGroup.scale.z = Math.min(this.size.x / gpxSize.x, this.size.z / gpxSize.z);


  var gpxCenter = gpxBounds.center();
  this.gpxGroup.position.x = -gpxCenter.x * this.gpxGroup.scale.x;
  this.gpxGroup.position.y = -gpxCenter.y * this.gpxGroup.scale.y;
  this.gpxGroup.position.z = -gpxCenter.z * this.gpxGroup.scale.z;







  this.boundsLL = {
    x: gpxCenter.z - (gpxSize.z/2),
    y: gpxCenter.x - (gpxSize.x/2),
    width: gpxSize.z,
    height: gpxSize.x
  };

  /*console.log(gpxCenter);
  console.log(gpxSize);
  console.log(this.gpxGroup.scale);
  console.log('GpxBounds');
  console.log(gpxBounds);*/

  console.log(this.terrain);

  if(this.terrain !== undefined){
    this.terrain.get(
      this.boundsLL,
      this.updateTerrain.bind(this)
    );
  }
}

Location.Ui.GpxThumb.prototype.updateTerrain = function(tiles){
  //this.terrain_tiles = new Terrain.Ui.Tile(tiles);
  this.terrain_ui = new Terrain.Ui.Tile(tiles, this.boundsLL, this.size);
  this.group.add(this.terrain_ui.group);
  console.log("Got " + tiles.length + " tiles");
  console.log(tiles);
  render()
}

Location.Ui.GpxThumb.prototype.addSegment = function(segment){
  var trackGeometry = new THREE.BufferGeometry();
  var trackMaterial = new THREE.LineBasicMaterial({ vertexColors: THREE.VertexColors });

  var trackPositions = new Float32Array( segment.points.length * 9 );
  var trackColors = new Float32Array( segment.points.length * 9);


/*
  var flagGeometry = new THREE.BufferGeometry();
  var flagMaterial = new THREE.LineBasicMaterial({ vertexColors: true });
  flagGeometry.addAttribute( 'position', new Float32Array( segment.points.length * 3 * 2), 3);
  flagGeometry.addAttribute( 'color', new Float32Array( segment.points.length * 3 * 2 ), 3 );
  */

  for(var i=0; i < segment.points.length; i++){
    var point = segment.points[i];
    var z = point.loc[0];
    var x = point.loc[1];
    var y = (point.elev === undefined) ? 0 : point.elev;

    trackPositions[ (i * 9) ] = x;
    trackPositions[ (i * 9) + 1 ] = y;
    trackPositions[ (i * 9) + 2 ] = z;

    trackPositions[ (i * 9) + 3 ] = x;
    trackPositions[ (i * 9) + 4 ] = 0;
    trackPositions[ (i * 9) + 5 ] = z;

    trackPositions[ (i * 9) + 6 ] = x;
    trackPositions[ (i * 9) + 7 ] = y;
    trackPositions[ (i * 9) + 8 ] = z;

    var speed = point.speed / 10;
    speed = 1 - (Math.min(speed, 1.0) * 0.8);

    var col = (new THREE.Color).setHSL( speed, speed, 0.6 );

    //Line of travel
    trackColors[ (i * 9) ] = col.r;
    trackColors[ (i * 9) + 1 ] = col.g;
    trackColors[ (i * 9) + 2 ] = col.b;

    trackColors[ (i * 9) + 3 ] = 0.2;
    trackColors[ (i * 9) + 4 ] = 0.2;
    trackColors[ (i * 9) + 5 ] = 0.2;

    trackColors[ (i * 9) + 6 ] = 0.2;
    trackColors[ (i * 9) + 7 ] = 0.2;
    trackColors[ (i * 9) + 8 ] = 0.2;

    var material = new THREE.MeshBasicMaterial( { color: 0x0000ff } );
    var object = new THREE.Mesh( new THREE.CircleGeometry( 1, 20, 0, Math.PI * 2 ), material );
		object.position.set( z, x, y );
    object.geometry.computeBoundingBox();
    object.geometry.computeBoundingSphere();
    //this.gpxGroup.add( object );


    /*var radius   = 0.001,
        segments = 64,
        material = new THREE.MeshBasicMaterial( { color: 0x0000ff } ),
        geometry = new THREE.CircleGeometry( radius, segments );

    // Remove center vertex
    //geometry.vertices.shift();

    var line = new THREE.Mesh( geometry, material );
    //line.position.set(y,x,z);
    console.log(line);
    /*line.translateX( x );
    line.translateY( y );
    line.translateZ( z );*/

/*
    geometry.computeBoundingBox();
    geometry.computeBoundingSphere();*/
    //this.gpxGroup.add( axisHelper );
  }


  trackGeometry.addAttribute( 'position', new THREE.BufferAttribute(trackPositions, 3));
  trackGeometry.addAttribute( 'color', new THREE.BufferAttribute(trackColors, 3) );

  trackGeometry.computeBoundingBox();
  trackGeometry.computeBoundingSphere();



  this.gpxGroup.add(new THREE.Line( trackGeometry, trackMaterial ));
}

Location.Ui.GpxThumb.prototype.addGpx = function(gpx){
  for(var segIdx in gpx.segments) {
    var segment = gpx.segments[segIdx];
    this.addSegment(segment);
  }

//this.gpxGroup.rotateY(Math.PI/2);

  this.updateTransforms();


  this.group.add(this.gpxGroup);

}
