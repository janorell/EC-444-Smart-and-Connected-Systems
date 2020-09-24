var uc = require('upper-case');
var http = require('http');

http.createServer(function (req, res) {
  res.writeHead(200, {'Content-Type': 'text/plain'});
  res.write(uc("Hellow World in Upper-Case!!"));
  res.end();
}).listen(8080);
