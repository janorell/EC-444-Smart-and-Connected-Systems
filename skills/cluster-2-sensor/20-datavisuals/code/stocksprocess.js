var express = require('express');
var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);
var fs = require('fs');

http.listen(3000, function() {
  console.log('go to localhost:3000');
});



app.get('/', function(req,res){
  res.sendFile(__dirname + '/stocks.html');

app.get('/data', function(req,res){
  res.sendFile(__dirname + '/cvsstock.txt');
});
});

var file = __dirname + '/cvsstock.txt';


//io.on('transmit_data', function(socket){
var data = fs.readFileSync(file, 'utf8').toString().split("\n");
var readfile = [];
for (var i =0; i <data.length; i++)
  {
    readfile[i-1] = data[i];
//  }
};

io.on('connection', function(socket){
  io.emit('transmit_data', data);
  socket.on('disconnect', function(){
  });
});
