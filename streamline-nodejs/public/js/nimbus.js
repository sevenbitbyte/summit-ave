var nimbus = {};

nimbus.call = function(request, complete){
	request.tx_timestamp = (new Date()).getTime();

	return $.ajax({ 
        type: "POST",
        dataType: "json",
        data: request,
        url: "http://" + window.location.host + "/nimbus",
        error : function(xhr, status, error){
        	alert("Status: [" + status + "]");
        	alert("Error: [" + error + "]");
        },
        success: function(data){
           var result=data;
           result.req.rx_timestamp = (new Date()).getTime();
           result.req.latency = result.req.rx_timestamp - result.req.tx_timestamp;
           console.log("Total latency = " + result.req.latency + "ms");
           console.log("Compute time = " + result.latency + "ms");
           console.log("Network time = " + (result.req.latency - result.latency) + "ms");
           if(complete !== undefined){
           	complete(result);
           }
           else{
           	alert("invalid completion callback");
           }

        }
	});
}