var http = require('http');
var dm = require('./datemod');


http.createServer(function (req, res) {
  res.writeHead(200, {'Content-Type': 'text/html'});
  res.write("Current Data and Time:" +dm.myDateTime());
  res.end();
}).listen(8080);
