
if ( ! Detector.webgl ) Detector.addGetWebGLMessage();

var container, stats;

var camera, scene, renderer, controls, keyboard;


var gaze = new THREE.Vector3(0, 0, 0);

var mesh;
var lastRenderRequestTime = undefined
var animateWatchDog = undefined;
var renderState='idle'
var timeline;
var target;
var location_data;
var loc_center;
var terrain_enabled = true;
var gpx_collection = new Location.GpxCollection();
var terrain_datasource = new Terrain.Datasource(window.location.origin);
var terrain_manager = new Terrain.TileManager(terrain_datasource);


$( document ).ready(function() {

  timeline = new Timeline($('#timeline')[0])

  init();

  var opts = {
    readAsDefault: "Text",
    on: {
      load: function(e, file) {
        console.log("Read: " + file.name);
        uploadGpx(file, e.target.result);
      }
    }
  };
  $("#fileUploader").fileReaderJS(opts);


  $('#activeDate').change(
    function(){
      var dateString = $('#activeDate').val();

      console.log(dateString)

      var date = new Date(dateString);
      console.log(date)
      //date = new Date(Date.UTC(date.getFullYear(), date.getMonth(), date.getDate()))
      date.setHours(1)
      date.setMinutes(0)

      //requestGpxData(date);
      console.log(date)

      timeline.setDate(date);
      getGpxData();
    }
  );

  //$('#activeDate').val("2014-03-24").change();
  var now = new Date();
  var day = ("0" + now.getDate()).slice(-2);
  var month = ("0" + (now.getMonth() + 1)).slice(-2);

  var today = now.getFullYear()+"-"+(month)+"-"+(day) ;

  $('#activeDate').val(today).change();

  //getGpxData();

});


function requestGpxData(date){
  dateStr = date.toISOString();
  dateStr = dateStr.substr( 0, dateStr.indexOf('T') );
  dateStr = dateStr.replace('-', '');
  dateStr = dateStr.replace('-', '');

  var request = {
          owner: "sevenbit",
            target: "location/gpx",
            action: "READ",
            params: {
              originObj: dateStr,
              //fields: ["time", "lat", "lon", "speed", "ele"],
              //stats: ["max", "min"]
            }
          };

  var val = nimbus.call(request, renderGpxResponse);
}

function getGpxData(){
  var iter = timeline.span.iterate('days');

  while(iter.hasNext()){
    var d = iter.next();
    requestGpxData( d.toDate() )
  }
}

function renderGpxResponse(gpxJson){
  console.log(gpxJson.res.data);

  if(gpxJson.res.data.gpx === undefined){
    console.log('ERROR: Failed to retrieve location data');
    return;
  }

  var gpx = new Location.Gpx(gpxJson.res.data.gpx);
  gpx_collection.addGpx(gpx);
  addGpxToScene(gpx);
}

function uploadGpx(file, data){
  fileJson = $.xml2json($.parseXML(data));

  var gpx = new Location.Gpx(fileJson);
  gpx_collection.addGpx(gpx);
  addGpxToScene(gpx);


  console.log('gpx...');
  console.log(gpx);

  /*var request = {
          owner: "sevenbit",
            target: "location/gpx",
            action: "WRITE",
            params: {
              originObj: 'user@host/path',
              format: 'gpx'
            },
            data : fileJson
          };*/

  var request = {
          owner: "sevenbit",
            target: "location/gpx",
            action: "WRITE",
            params: {
              originObj: 'user@host/path',
              format: 'location-track'
            },
            data : gpx
          };


  console.log(request);


  var val = nimbus.call(request, uploadCallback);
}

function uploadCallback(result){
  console.log("Upload complete");
  console.log(result);
}

function drawDate(month, day, year) {

  var dateStr = "" + year + "" + month + "" + day;
  var request = {
          owner: "sevenbit",
            target: "location/points",
            action: "READ",
            params: {
              originObj: dateStr,
              fields: ["time", "lat", "lon", "speed", "ele"],
              stats: ["max", "min"]
            }
            };

  var val = nimbus.call(request, init);
}

var tileCount = 0;
function addGpxToScene(gpx){
  addGpxToTimeline(gpx);
  console.log("Segments: " + gpx.segments.length);
  console.log('Constructing GpxThumb...');

  if(terrain_enabled){
    mesh = new Location.Ui.GpxThumb({terrain: terrain_manager});
  }
  else{
    mesh = new Location.Ui.GpxThumb();
  }
  mesh.group.translateX(tileCount*10 );
  mesh.addGpx(gpx);
  console.log(mesh);

  scene.add( mesh.group );

  //camera.translateY(tileCount*10);
  //camera.lookAt(mesh.group.position);

  tileCount++;
  render();
}

function addGpxToTimeline(gpx){
  console.log('addGpxToTimeline')
  console.log(gpx)

  for(var i=0; i<gpx.segments.length; i++){
    timeline.addRegion(
      'location',
      moment.twix(
        moment(gpx.segments[i].start_time),
        moment(gpx.segments[i].end_time)
      )
    )
  }
}

function init() {

  container = document.getElementById( 'container' );

  console.log($(container).width() +' ' + $(container).height())
  console.log($(container).width() / $(container).height())
  console.log(window.innerWidth / window.innerHeight)

  camera = new THREE.PerspectiveCamera( 45, window.innerWidth / window.innerHeight, .1, 4000 );
  camera.position.set( 0, 20, 0 );

  var axisHelper = new THREE.AxisHelper( 0.05 );
  axisHelper.translateX(-0.411 * (window.innerWidth / window.innerHeight));
  axisHelper.translateY(-0.41);
  axisHelper.translateZ(-1);
  axisHelper.geometry.computeBoundingBox();

  camera.add(axisHelper);

  //camera.rotateOnAxis( new THREE.Vector3(0,1,0), Math.PI * 0.5);

  scene = new THREE.Scene();


  scene.add(camera);

  //console.log(scene);
  //console.log(camera);

  //target = new THREE.Vector3(camera.position.x, camera.position.y, 0);

  //camera.lookAt(target);

  //camera.up = new THREE.Vector3(0, 1, 0);

  /*target = new THREE.Vector3(camera.position.x, camera.position.y, 0);

  camera.lookAt(target);
  camera.rotation.z = 0;*/

  if(renderer !== undefined){

    while (container.hasChildNodes()) {
        container.removeChild(container.lastChild);
    }
  }

  renderer = new THREE.WebGLRenderer( { antialias: false } );
  renderer.setSize( window.innerWidth, window.innerHeight );

  renderer.gammaInput = true;
  renderer.gammaOutput = true;

  container.appendChild( renderer.domElement );


  THREEx.WindowResize(renderer, camera);
  THREEx.FullScreen.bindKey({ charCode : 'f'.charCodeAt(0) });
  controls = new THREE.OrbitControls( camera, renderer.domElement );
  //controls = new THREE.FirstPersonControls( camera, renderer.domElement );
  //controls.rotateSpeed = 1.0;
  controls.zoomSpeed = 0.5
  /*controls.zoomSpeed = 1.2;
  controls.panSpeed = 0.8;
  controls.noZoom = false;
  controls.noPan = false;
  controls.noRotate = false;
  controls.staticMoving = false;
  controls.dynamicDampingFactor = 0.3;*/



  stats = new Stats();
  stats.domElement.style.position = 'absolute';
  stats.domElement.style.top = '0px';
  container.appendChild( stats.domElement );

  keyboard	= new THREEx.KeyboardState();


  window.addEventListener( 'resize', onWindowResize, false );

  renderState = 'idle'

  render();


  controls.addEventListener('change', function(){

    /*if(animateWatchDog !== undefined && renderState == 'animate'){
      return
    }

    console.log('controls.change - starting animation');*/


    animate();

  })

  //stats.update();
  //setTimeout(animate);

}

function onWindowResize() {

  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();

  renderer.setSize( window.innerWidth, window.innerHeight );

  render();
}

//

function animate() {
  lastRenderRequestTime = (new Date()).getTime()
  if(renderState != 'animate'){
    renderState = 'animate'
    console.log('animating')
    animateCb();
  }
}

function animateCb(){
  if(renderState !== undefined && renderState != 'animate'){
    return;
  }

  if(animateWatchDog === undefined){
    animateWatchDog = setTimeout(function(){
      var now = (new Date()).getTime()
      if(now-lastRenderRequestTime > 1500 && renderState != 'idle'){
        renderState='idle';
        console.log('stopping animation')
      }
      else{
        //console.log('not stopping')
      }
      animateWatchDog = undefined
    }, 3000)
  }

  controls.update();

  renderer.render( scene, camera );
  stats.update();
  requestAnimationFrame( animateCb );
}

function render() {
  /*if(keyboard.pressed("A")){
    camera.position.x-=1.5;
    gaze.x-=1.5;
    camera.up = camera.up;
    camera.lookAt(gaze);
  }

  if(keyboard.pressed("D")){
    camera.position.x+=1.5;
    gaze.x+=1.5;
    camera.up = camera.up;
    camera.lookAt(gaze);
  }*/

  if(renderState != 'idle'){
    return false;
  }

  renderState = 'render'
  var now = (new Date()).getTime()

  controls.update();
  timeline.render()
  renderer.render( scene, camera );
  stats.update();
  renderState = 'idle'

  if(lastRenderRequestTime > now){
    render();
  }

  return true;
}
