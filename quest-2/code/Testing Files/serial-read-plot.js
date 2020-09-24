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

// Read data from serial
var read_data;
var msg;
// fs.writeFile('log.txt', '0.0 , 0.0 , 0.0', function(err) {});
var parser = new Readline();
port.pipe(parser);
parser.on('data', function (data) {
  const sensorData = tryParseJson(data);
  fs.appendFile('log.txt', `${sensorData.temp} , ${sensorData.ir} , ${sensorData.ultrasonic}\n`, function (err) {
    if (err) return console.log(err);
    console.log(sensorData);
  });
});

// Points to index.html to serve webpage
app.get('/', function(req, res){
  res.sendFile(__dirname + '/index.html');
  app.get('/data', function(req, res) {
    res.sendFile(__dirname + '/log.txt');
  });
});

// Listening on localhost:3000
http.listen(3000, function() {
  console.log('listening on :3000');
});

var file = __dirname + '/log.txt';
// var filedata = fs.readFileSync(file, 'utf8').toString().split("\n");
// var data = [];
// for (i=1;i<filedata.length -1; i++) {
//   data[i-1] = filedata[i];
// }

io.on('connection', function(socket){
  var filedata = fs.readFileSync(file, 'utf8').toString().split("\n");
  var data = [];
  for (i=0; i<filedata.length; i++){
    data[i]=filedata[i]
    console.log(data)
    io.emit('transmit_data', data);
    socket.on('disconnect', function(){
    });
  }
  // for (j=0; j<filedata.length-5; j++) {
  //   // sending windows of data
  //   for (i=j;i<j+6;i++){
  //     data[i-j] = filedata[i];
  //   }
  //   console.log(data)
  //   io.emit('transmit_data', data);
  //   socket.on('disconnect', function(){
  //   });
  // }
//   for (i=1;i<filedata.length -1; i++) {
//     data[i-1] = filedata[i];
// }
  // console.log(data)
  // io.emit('transmit_data', data);
  // socket.on('disconnect', function(){
  // });
});
