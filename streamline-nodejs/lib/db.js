var mongoose = require('mongoose');
var db_connection = mongoose.createConnection('mongodb://localhost/streamline-terrain');


module.exports = db_connection;
