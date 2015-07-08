var nimbus = {
	srv : {
		location : require('./location').Srv
	}
};

nimbus.getService = function(targetList, action) {
	var v = nimbus.srv;

	for(var idx in targetList){
		v = v[ targetList[idx] ];
		if( v === undefined ){
			return undefined;
		}
	}

	if(v[action] === undefined){
		console.log('ERROR: nimbus.getService - No such action [' + action + '] on target ' + JSON.stringify(targetList));
		return undefined;
	}

	return v[action];
}

/*

nimbusReq = {
    ownerId: "sevenbit",
    tx_timestamp : (new Date()).getTime(),
    target: "location/segments",
    action: "READ",
    params: {},
    data: {},
    user: "sevenbit",
    gpx: "Vault/Location/gpx/20140413.gpx"
}, {}


The client may:
	-> Make a synchronous request which returns the results in the HTTP response
	-> Make a synchronous request which is conditionally dispatched into the session(session defered result)
		-> This should be a result of system policy (ie: max latency guarantee)
	-> Make an asychronous request which goes directly to the users session store
	-> Request the status of all defered results
	-> Grant any valid permission on defered requests
		*This could be implemented several ways: ... actually both make sense user should be able to dispatch both requests and results
			- Server could dispatch copies/references to the result to all selected identities by way of NOTIFY mechanism
			- Server could dispatch copies of the request into the targeted identities' global session and run request as their identity

The nimbus system may:
	-> Choose to defer processing of any request (dispatch defered, computation defered)
	-> Choose to cancel or session defer transmission of result

The system must:
	-> Reply within a 50ms window to all requests with a status and optional reason/explanation
*/

var reqCount = 0;
var dispatchQueue = {};
var responseList = {};

function enqueueDispatch(dispatchData){
	dispatchData.id = reqCount++;
	dispatchQueue[dispatchData.id] = dispatchData;
}

exports.dispatchError = function(dispatchData, err){
	dispatchData.nimbusRes.err = err;
	exports.dispatchComplete(dispatchData);
}

exports.dispatchComplete = function(dispatchData){

	dispatchData.tx_timestamp = (new Date()).getTime();
	dispatchData.nimbusRes.latency = dispatchData.tx_timestamp - dispatchData.rx_timestamp;
	dispatchData.nimbusRes.req = dispatchData.nimbusReq;

	console.log("TX: " + dispatchData.nimbusRes);
	dispatchData.res.json(dispatchData.nimbusRes);

	delete(dispatchQueue[dispatchData.id])
};




exports.dispatch = function(req, res){
	var requests = [];

	if( !(req.body === undefined) ){

		if(Array.isArray(req.body)){
			requests = req.body;
		} else {
			requests.push(req.body);
		}

		for(idx in requests){
			//TODO: Validation

			dispatchData = {
				req 			: req,
				res 			: res,
				nimbusReq 		: requests[idx],
				nimbusRes		: { res: {} },
				rx_timestamp	: (new Date()).getTime(),
			};

			console.log("Dispatching - {target:" + dispatchData.nimbusReq.target
									+ ", action:" + dispatchData.nimbusReq.action
									+ ", owner: + " + dispatchData.nimbusReq.owner );

			if( dispatchData.nimbusReq.target === undefined){
				dispatchData.target_list = [];
			} else {
				dispatchData.target_list =  dispatchData.nimbusReq.target.split('/');
			}

			enqueueDispatch(dispatchData);

			srv = nimbus.getService(dispatchData.target_list, dispatchData.nimbusReq.action);

			if( dispatchData.target_list.length > 0 && srv !== undefined ){
				srv(dispatchData, exports.dispatchComplete);
			} else {
				dispatchData.nimbusRes.res.err = "undefined target";
				console.log("Undefined target: [" + dispatchData.target_list + "]");
				exports.dispatchComplete(dispatchData);
			}
		}
	} else {
		console.log("Empty request");
		res.json({});
	}
};
