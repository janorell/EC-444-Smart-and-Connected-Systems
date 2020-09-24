var fs = require('fs');
var express = require('express');
var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);


http.listen(3000, function() {
  console.log('listening on port 3000    :o');
});


app.get('/', function(req, res){
  res.sendFile(__dirname + '/testing.html');
  app.get('/data', function(req, res) {
    res.sendFile(__dirname + '/log.txt');
  });
});

var file = __dirname + '/log.txt';
var filedata = fs.readFileSync(file, 'utf8').toString().split("\n");
var data = [];
for (i=1;i<filedata.length -1; i++) {
  data[i-1] = filedata[i];
}

io.on('connection', function(socket){
  io.emit('transmit_data', data);
  socket.on('disconnect', function(){
  });
});
