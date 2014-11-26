var dbus = require('dbus-native');
var sysBus = dbus.systemBus()
//var nm = sysBus.getService('org.freedesktop.NetworkManager')

sysBus.connection.on('connect', function(a,b){
	console.log('Connected: ' + a)
});

sysBus.connection.on('error', function(a,b){
	console.log('Error: ' + a)
});

//sysBus.connection.on('message', console.log);
//sysBus.listNames(console.log)
var called = false;
sysBus.getInterface('/org/freedesktop/NetwokManager', 
	'1.318',
'org.freedesktop.DBus.Introspectable',
     function(err, nmIf){
	console.log(err)
	console.log(nmIf)
	called = true
     }
);





//sysBus.addMatch("type='signal'");
sysBus.connection.on('message', console.log);
