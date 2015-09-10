#!/usr/bin/env nodejs
var debug = require('debug')('my-application');
var Hoek = require('hoek');
var Async = require('async');
var fs = require('fs');
var colors = require('colors');
var TerrainRenderer = require('../routes/nimbus/terrain_renderer.js');
var Args = require('command-line-args');

var cli = Args([
  { name: 'point', alias: 'p', type: String, multiple: true },  // Selection point
  { name: 'rect', alias: 'r', type: String, multiple: true },   // Selection rectangle
  { name: 'input', alias: 'i', type: String, multiple: true},   // Input data files
  { name: 'dir', type: String }                                 // Download directory
])

{
  input : [String],
  point : [ [x,y], ... ],
  rect : [ [x1,y1,x2,y2], ... ],
  tile : [{
    sizeLL : { x: Number, y: Number },
    scale: Number
  }]
}

var config = {
  tiles : [
    {
      sizeLL : 0.25,
      sizePx : 300
    },
    {
      sizeLL : 5.0,
      sizePx : 375
      parent: { sizeLL : 0.25 }
    },
    {
      sizeLL : 10.0,
      sizePx : 375,
      parent: { sizeLL : 5.0 }
    },
    {
      sizeLL : 22.5,
      sizePx : 421,
      parent: { sizeLL : 10.0 }
    },
    {
      sizeLL : 45,
      sizePx : 421,
      parent: { sizeLL : 22.5 }
    },
    {
      sizeLL : 90.0,
      sizePx : 421,
      parent: { sizeLL : 45.0 }
    },
    {
      sizeLL : 180.0,
      sizePx : 421,
      parent: { sizeLL : 90.0 }
    }
  ]
}

var generateTiles = function(grid){
  var gridCount = Math.pow(Math.ceil(360.0 / grid.sizeLL), 2);

  for(var idx=0; idx < gridCount; idx++);
}

var gridIndexToRectLL = function(grid, idx){
  var tileWidth = Math.ceil(360.0 / grid.sizeLL)

  var tileX = idx % tileWidth;
  var tileY = Math.floor(idx / tileWidth)

  var lowerleft = {
    x : tileX * grid.sizeLL,
    y : tileY * grid.sizeLL
  }

  var topRight = {
    x : (tileX+1) * grid.sizeLL,
    y : (tileY+1) * grid.sizeLL
  }

  return {
    lowerleft: lowerleft,
    topright: topRight
  }
}

//try{

  var path = (process.argv.length > 2) ? process.argv[2] : undefined;

  console.log(path)

  var TerrainParser = require('../routes/nimbus/parsers/terrain_arcascii_crawler');
  var dataSource = new TerrainParser.TerrainDataSource({path: path});
  var render = undefined;

  var limitLowerleft = {
    // Commerece Bay, Tacoma, WA
    x: -122.4548532,
    y: 47.2856862
  }

  var limitTopRight = {
    // Echo Lake, Snohomish, WA
    x: -122.0508225,
    y: 47.7864735
  }



  dataSource.on('ready', function(){
    console.log('ready');
    var model = dataSource.getModel();

    render = new TerrainRenderer(dataSource);
    dataSource.crawl();

    var query = model.TileInfo.find({}).or([
    { // Top Right
      'topright.x' : { $gt: limitLowerleft.x, $lt: limitTopRight.x },
      'topright.y' : { $gt: limitLowerleft.y, $lt: limitTopRight.y },
    },
    { // Bottom Left
      'lowerleft.x' : { $gt: limitLowerleft.x, $lt: limitTopRight.x },
      'lowerleft.y' : { $gt: limitLowerleft.y, $lt: limitTopRight.y },
    },
    { // Top left
      'lowerleft.x' : { $gt: limitLowerleft.x, $lt: limitTopRight.x },
      'topright.y' : { $gt: limitLowerleft.y, $lt: limitTopRight.y },
    },
    { // Bottom right
      'topright.x' : { $gt: limitLowerleft.x, $lt: limitTopRight.x },
      'lowerleft.y' : { $gt: limitLowerleft.y, $lt: limitTopRight.y },
    }
  ]);

    var promise = query.exec();
    promise.then(
      function(infoList){
        console.log('Selected ' + infoList.length + ' tiles');
      }
    );


  });
