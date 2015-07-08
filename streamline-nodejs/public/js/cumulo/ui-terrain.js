Terrain.Ui = {};

Terrain.Ui.Tile = function(tiles, bounds, size){
  this.tiles = tiles;
  this.gpxGroup = new THREE.Object3D();
  //this.meshes = [];
  //this.geometry = [];
  //this.material = [];

  this.bounds = (bounds !== undefined) ? bounds : undefined;

  this.size = size;
  /*this.size = (size !== undefined) ? size : {
    x: 10,
    y: 1,
    z: 10
  }*/;

  console.log(this);


  this.group = new THREE.Object3D();
  //this.group.add(this.gpxGroup);
  this.group.userData = this;

  var size = 5;
  var step = 1;
  var gridHelper = new THREE.GridHelper( size, step );

  gridHelper.translateY(-0.5);
  //gridHelper.rotateOnAxis( new THREE.Vector3(1,0,0), -Math.PI * 0.5);
  gridHelper.setColors('yellow', 'white');
  gridHelper.material.opacity = 0.1;
  gridHelper.material.transparent = true;


  var axisHelper = new THREE.AxisHelper( 0.25 );
  axisHelper.translateX(-5);
  axisHelper.translateY(-0.5);
  axisHelper.translateZ(5);
  axisHelper.geometry.computeBoundingBox();

  //this.group.add( axisHelper );
  this.group.add( gridHelper );

  this.oceanMeshMaterial = new THREE.MeshBasicMaterial( { color: 0x72AB97, wireframe: true,
    transparent: true,
    opacity: 0.1
  });

  this.blackMaterial = new THREE.MeshPhongMaterial( {
    color: 0x003925, ambient: 0x003925, specular: 0x010101, shininess: 0,
    side: THREE.DoubleSide, opacity: 0.2, transparent: false,
    shading: THREE.SmoothShading, shadow: THREE.PCFShadowMap, colors: THREE.VertexColors
  });

  this.oceanUniforms = {
    u_color : { type: "c", value: new THREE.Color( 0x383a49 ) }
  };





  var oceanWidth = this.size.z;
  var oceanLength = this.size.x;
  var widthPx = Math.round(this.bounds.width / (this.tiles[0].info.sizeLL.x / this.tiles[0].elev.width));   //width in px
  var heightPx = Math.round(this.bounds.height / (this.tiles[0].info.sizeLL.y / this.tiles[0].elev.height)); //height in px
  var sizePx = Math.max(widthPx, heightPx);

  console.log(widthPx);
  console.log(heightPx);

  this.oceanPlane = new THREE.PlaneGeometry(oceanWidth, oceanLength, widthPx, heightPx); //imgCanvas.width-1, imgCanvas.height-1);
  this.oceanPlane.doubleSided = true;
  this.oceanPlane.dynamic = true;
  this.oceanPlane.computeFaceNormals();
  this.oceanPlane.computeVertexNormals();

  var cellSize = {
    x: (this.tiles[0].info.sizeLL.x / this.tiles[0].elev.width),
    y: (this.tiles[0].info.sizeLL.y / this.tiles[0].elev.height)
  };

  if(this.bounds.width == 0 || this.bounds.height == 0){
    return this;
  }

  var imgWidthPx = this.bounds.width / cellSize.x;
  var imgHeightPx = this.bounds.height / cellSize.y;
  var imgSizePx = Math.max(imgWidthPx, imgHeightPx);




  var imgData = this.tiles[0].elev.getContext("2d").getImageData(0, 0, imgSizePx, imgSizePx).data;

  console.log(this.oceanPlane.vertices.length)

  for ( var j = 0; j < this.oceanPlane.vertices.length; j ++ ) {

    this.oceanPlane.vertices[  j ].z = 1.0 - ((imgData[j *4] / 255) * 1.0);
  }

  var blackMaterial = new THREE.MeshPhongMaterial( {
    color: 0x003925, ambient: 0x003925, specular: 0x003925, shininess: 0,
    side: THREE.DoubleSide, opacity: 0.75, transparent: false,
    shading: THREE.FlatShading
  });

  this.ocean = new THREE.Mesh(this.oceanPlane, this.blackMaterial);
  //this.ocean = new THREE.Mesh(this.oceanPlane, new THREE.MeshNormalMaterial());
  this.ocean.name = name +"_ocean";
  this.ocean.rotation.x = Math.PI/2;
  this.ocean.rotation.z = Math.PI/2;
  this.ocean.doubleSided = true;


  this.ocean2 = new THREE.Mesh(this.oceanPlane, this.oceanMeshMaterial);
  this.ocean2.name = name +"_ocean2";
  this.ocean2.rotation.x = Math.PI/2;
  this.ocean2.rotation.z = Math.PI/2;
  this.ocean2.doubleSided = true;

  this.ocean.translateZ(-0.5);
  this.ocean2.translateZ(-0.5);

  //this.group.add(this.ocean);
  this.group.add(this.ocean2);
  this.group.add( new THREE.AmbientLight( 0xD0ECBD ) );


  return this;
}
