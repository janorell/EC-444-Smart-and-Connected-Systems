var http = require('http');
var url = require('url');

http.createServer(function(req,res){
  res.writeHead(200, {'Content-Type': 'text/html'});
  var quer = url.parse(req.url, true).query;
  var text = quer.year + " " + quer.month;
  res.write("Hello World! My name is Jenni Norell     ");
  res.end(text);
}).listen(8080);
