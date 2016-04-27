function Utils() {

	var loadScript = function (src, callback) {

		var script = document.createElement("script");
		script.type = "text/javascript";
		script.src = src;
		script.async = true;
		if(callback) script.onload = callback;
	  document.getElementsByTagName("head")[0].appendChild(script);
	};

	return {
		loadScript: loadScript
	}

}
