var SerialPort = require('serialport');
var Readline = require('@serialport/parser-readline')
var express = require('express');
var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);
const fs = require('fs');

// Open serial port
var port = new SerialPort('/dev/cu.SLAB_USBtoUART', {baudRate: 115200});

function tryParseJson(str) {
    try {
        JSON.parse(str.toString());
    } catch (e) {
        return false;
    }
    return JSON.parse(str.toString());
}


var read_data;
var msg;

const parser = port.pipe(new Readline({ delimiter: '\r\n' }))
parser.on('data', function (data) {
  read_data = data;
  console.log('Read:', read_data);
  msg = tryParseJson(read_data);  // Convert to int
  io.emit('message', msg);
});

// Points to index.html to serve webpage
app.get('/', function(req, res){
  res.sendFile(__dirname + '/serialread.html');

});

// Listening on localhost:3000
http.listen(3000, function() {
  console.log('listening on :3000');
});


// var filedata = fs.readFileSync(file, 'utf8').toString().split("\n");
// var data = [];
// for (i=1;i<filedata.length -1; i++) {
//   data[i-1] = filedata[i];
// }

// User socket connection
io.on('connection', function(socket){
  console.log('a user connected');
  socket.on('disconnect', function(){
    console.log('user disconnected');
  });
});
