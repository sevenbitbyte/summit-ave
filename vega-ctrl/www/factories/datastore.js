function DataStore() {
	console.log('DataStore');

	var db;

	function get(key) {
		if(db.hasOwnProperty(key))
			return db[key];
		else
			return undefined;
	}

	function set(key, val) {
		if(db.hasOwnProperty(key))
			return db[key] = val;
		else
			return undefined;
	}

	return {
		get: get,
		set: set
	};
}
