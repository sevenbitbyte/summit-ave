var fs = require('fs');
var ini = require('ini');
var xml2js = require('xml2js');

/* GET users listing. */
exports.info = function(req, res){
	var config = undefined;
	var overview = {};

	console.log(req.body);



	try{
		var nonAlphaNum = new RegExp(/\W/g)
		if(!nonAlphaNum.test(req.params.user)){
			config = ini.parse(fs.readFileSync('/home/'+req.params.user+'/.config/cumulonimbus/cumulo.conf', 'utf-8'));
		}
	}
	catch(err){
		console.log("No such user: " + req.params.user + " Error: " + err);
		overview.message="Not Found";
	  	res.json(overview);
	  	return;
	}

	if(config === undefined) {
		console.log("No such user" + req.params.user);
		overview.message="Not Found";
	  	res.json(overview);
	  	return;
	}

	var nimbusReq = req.body;
	if(!(nimbusReq === undefined)){
		if(nimbusReq.gpx != null) {
			try{
				var parser = new xml2js.Parser();
				fs.readFile('/home/'+req.params.user+"/"+nimbusReq.gpx, function(err, data) {
				    parser.parseString(data, function (err, result) {
				        console.dir(result);
				        console.log('Done');
				        res.json(result);
				    });
				});
				return;
			}
			catch(err){
				console.log("No such gpx: " + nimbusReq.gpx + " Error: " + err);
				overview.message="Gpx Not Found";
			  	res.json(overview);
			  	return;
			}
		}
	}

	overview.datatypes = {};


	for(var index in config.General) { 
		if (config.General.hasOwnProperty(index)) {
			isDbPath = index.indexOf("crawlerDbPath");
			if(isDbPath > -1){
				var type = index.substring(0,isDbPath);

				if(type === undefined || type.length < 1){continue;}

				if(overview.datatypes[type] === undefined){
					overview.datatypes[type] = {
													count: 1
												};
				}
				else{
					overview.datatypes[type].count++;
				}
			}
		}
	}


	res.json(overview);

	/*
	fs.readFile('/home/'+req.params.id+'/.config/cumulonimbus/cumulo.conf', {encoding:'utf-8'}, function (err, data) {
	  if (err){
	  	console.log("Reading cumulo config file: " + err);
	  	res.send("{}");
	  }
	  else{
	  	console.log("Reading cumulo config file" + data);
	  	res.send("test"+ typeof(data) + '<br>'+ data);
	  }
	});*/
};
