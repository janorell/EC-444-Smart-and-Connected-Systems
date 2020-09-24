var SerialPort = require('serialport');
var Readline = require('@serialport/parser-readline')
var express = require('express');
var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);
const fs = require('fs');

// Open serial port
// var port = new SerialPort('/dev/cu.SLAB_USBtoUART', {baudRate: 115200}); //port for macs
var port = new SerialPort('COM5', {baudRate: 115200}); //port for windows

// function for parsing JSON
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
var parser = new Readline();
port.pipe(parser);
parser.on('data', function (data) { //parsing data
  const sensorData = tryParseJson(data);
  //append to log.txt file in csv format
  fs.appendFile('log.txt', `${sensorData.temp} , ${sensorData.ir} , ${sensorData.ultrasonic}\n`, function (err) {
    if (err) return console.log(err);
    console.log(sensorData); //printing data on console
  });
});

// Points to dynamic-index.html to bring up webpage
app.get('/', function(req, res){
  res.sendFile(__dirname + '/dynamic-index.html');
  //get data from log.txt
  app.get('/data', function(req, res) {
    res.sendFile(__dirname + '/log.txt');
  });
});

// Listening on localhost:3000
http.listen(3000, function() {
  console.log('listening on :3000');
});

var file = __dirname + '/log.txt';
io.on('connection', function(socket){
  // split data at new line
  var filedata = fs.readFileSync(file, 'utf8').toString().split("\n");
  var data = [];
  for (i = 0; i < filedata.length; i++){
    data[i]=filedata[i] //iterating to grab next row of data
    io.emit('transmit_data', data); //send data to html
    socket.on('disconnect', function(){
    });
  }
});
