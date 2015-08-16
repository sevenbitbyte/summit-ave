var Hapi = require('hapi');
var Good = require('good');
var GoodConsole = require('good-console');
var Package = require('./package.json')

var routes = require('./lib/routes').routes;

var serverConfig = {}
var server = new Hapi.Server(serverConfig);


server.connection({
    host: 'localhost',
    port: 3000
});


server.route(routes);

var plugins = [];
plugins.push({
    register: Good,
    options: {
        reporters: [{
            reporter: GoodConsole,
            events: {
                log: '*',
                response: '*',
                ops: '*'
            }
        }]
    }
});

server.register(plugins, function (err) {

    if (err) {
        throw err;
    }

    server.start(function () {

        console.log( Package.name + ' (v' + Package.version + '): ' + server.info.uri);
    });
});
