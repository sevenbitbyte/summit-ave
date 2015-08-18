var fs = require('fs');
var Path = require('path');
var byline = require('byline');
var events = require('events');
var Promise = require('mpromise');
var mongoose = require('mongoose');
var Histogram = require('../histogram.js');


var TerrainModel = require('../models/terrain_model');

var reportError = function(value){
  console.log('ERROR: Value = ' + value);
}

var db_connection = mongoose.createConnection('mongodb://localhost/streamline-terrain');

db_connection.on('error', reportError);

db_connection.on('connecting', function(){ console.log('mongoose connecting'); });
db_connection.on('connected', function(){ console.log('mongoose connected'); });


db_connection.on('disconnecting', function(){ console.log('mongoose disconnecting'); });
db_connection.on('disconnected', function(){ console.log('mongoose disconnected'); });
db_connection.on('close', function(){ console.log('mongoose close'); });
db_connection.on('reconnected', function(){ console.log('mongoose reconnected'); });



var TerrainDataSource = function(options){
  //events.EventEmitter.call(this);

  //db_connection.on('connected',
  //  function(){

      this.model = new TerrainModel(mongoose, db_connection);
  //    this.emit('ready');
  //  }.bind(this)
  //);

  return this;
}
