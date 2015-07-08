var fs = require('fs');
var ini = require('ini');
var xml2json = require('xml2json');
var moment = require('moment');


var mongoose = require('mongoose');
var db_connection = mongoose.createConnection('localhost', 'streamline-location');
var model = require('./models/location_model');
var LocationModel = new model.LocationModel(mongoose);


function readGpx(dispatchData, filePath, cb){
	try{
		//var parser = new xml2js.Parser();

		if(fs.existsSync(filePath)){
			console.log("Found: " + filePath);
			fs.readFile(filePath,
				function(err, data) {
			    	/*parser.parseString(data,
			    		function (err, result) {
			    	    	cb(dispatchData, err, result);
			    		}
			    	);*/
						var gpxJson = null
						try{
							gpxJson = xml2json.toJson(data, {object:true})
						}
						catch(e){
							err = e
						}
						
						cb(dispatchData,
							err,
							gpxJson);
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



exports.processReadGpx = function(dispatchData, completeCb){
	//TODO: Should verify that originObj is allowed by fs
	originObj = "/home/sevenbit/Vault/Location/gpx/" + dispatchData.nimbusReq.params["originObj"] + ".gpx";

	readGpx(dispatchData,
			originObj,
			function(data, error, result){
				if(result === undefined){
					dispatchData.nimbusRes.res = {err: error};
				} else {
					dispatchData.nimbusRes.res = {data: result};
				}
				completeCb(data);
			});
};

exports.processWriteGpx = function(dispatchData, completeCb){
		dispatchData.nimbusRes.res = dispatchData.nimbusReq.data;

		//Translate to storage format ->

		completeCb(dispatchData);
}


exports.processReadPoints = function(dispatchData, completeCb){

	//TODO: Should verify that originObj is allowed by fs
	//TODO: Support empty fields param, should select all available originObj
	originObj = "/home/sevenbit/Vault/Location/gpx/" + dispatchData.nimbusReq.params["originObj"] + ".gpx";

	console.log("Reading points from gpx: " + originObj);

	readGpx(dispatchData,
			originObj,
			function(data, error, result){
				if(result === undefined){
					if(dispatchData.nimbusRes.res === undefined){
						dispatchData.nimbusRes.res = {err: error};
					}
					else{
						dispatchData.nimbusRes.res.err = error;
					}
				} else{

					if(result === undefined || result == null || result.gpx === undefined || result.gpx == null || result.gpx.trk === undefined){
						dispatchData.nimbusRes.res = {err: "invalid file"};
						completeCb(dispatchData);
						return;
					}

					points_array = [];
					stats_array = [];

					for(segIdx in result.gpx.trk.trkseg){
						segment = result.gpx.trk.trkseg[segIdx];
						segment_data = {};
						segment_stats = {};

						segment_stats = {};
						fields = [];

						//Create field array
						for(fieldIdx in dispatchData.nimbusReq.params.fields){
							field = dispatchData.nimbusReq.params.fields[fieldIdx];
							segment_data[ field ] = new Array();
							fields.push(field);

							//Create stats entry
							segment_stats[ field ] = {};
							if(dispatchData.nimbusReq.params.stats !== undefined){
								for(statIdx in dispatchData.nimbusReq.params.stats) {
									metric = dispatchData.nimbusReq.params.stats[statIdx];
									segment_stats[field][metric] = null;
								}
							}
						}

						//Iterate over points
						for(ptIdx in segment.trkpt){
							point = segment.trkpt[ptIdx];
							unusedFields = new Array().concat(fields);

							for(attrIdx in point.$){
								attr = point.$[attrIdx];

								if(segment_data.hasOwnProperty(attrIdx)){
									point[attrIdx] = attr;
								}
							}


							//Iterate over fields
							for(attrName in point){
								attr = point[attrName];

								if(segment_data.hasOwnProperty(attrName)){
									if(Array.isArray(attr) && attr.length == 1){
										attr = attr[0]
									}

									//Do conversions
									if(attrName == 'time') {
										attr = moment(attr).valueOf();
									}
									else if(typeof(attr) == 'string') {
										//attempt to convert to float
										number = Number(attr);

										if(!isNaN(number)){
											attr = number;
										}
									}

									//Update stats
									attr_stats = segment_stats[attrName];
									if(attr_stats !== undefined){
										//Iterate over requested metrics
										for(statName in attr_stats){
											if(statName == 'max'){
												//Is value larger
												if(segment_stats[attrName][statName] == null || segment_stats[attrName][statName] < attr){
													segment_stats[attrName][statName] = attr;
												}
											}
											else if(statName == 'min'){
												//Is value larger
												if(segment_stats[attrName][statName] == null || segment_stats[attrName][statName] > attr){
													segment_stats[attrName][statName] = attr;
												}
											}
											/*else if(statName == 'avg' && typeof(attr) == 'number'){
												//Is value larger
												segment_stats[attrName][statName] += attr;
											}*/
										}
									}

									segment_data[attrName].push(attr);

									delete(unusedFields[fields.indexOf(attrName)]);
								}
							}


							for(attrIdx in unusedFields){
								attrName = unusedFields[attrIdx];
								console.log("Unused " + attrName);
								segment_data[attrName].push(null);
							}
						}


						points_array.push(segment_data);
						stats_array.push(segment_stats);
					}

					data_obj = {
						points: points_array,
						stats: stats_array
					}

					dispatchData.nimbusRes.res = {data: data_obj};


				};
				completeCb(data);
			});
};



var srv = {
	gpx    : {READ: exports.processReadGpx,
						WRITE: exports.processWriteGpx },
	points : { READ: exports.processReadPoints }
};


exports.Srv = srv;
