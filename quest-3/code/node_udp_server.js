// Required module
var dgram = require('dgram');
var express = require('express');
var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);
const fs = require('fs');
// Port and IP
var PORT = 8081;
var HOST = '192.168.1.111';

// Create socket
var server = dgram.createSocket('udp4');
// Create server
server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});
function tryParseJson(str) {
    try {
        JSON.parse(str.toString());
    } catch (e) {
        return false;
    }
    return JSON.parse(str.toString());
}

var clickcheck; //button variable for red LED

// On connection
server.on('message', function (message, remote) {
    const sensorData = tryParseJson(message);
        // Send Ok acknowledgement
        server.send(`${clickcheck}`,remote.port,remote.address,function(error){
          if(error){
            console.log('MEH!');
          }
          else{
            console.log(`Sent: ${clickcheck}`);
          }
          clickcheck = 0;
        });
    fs.appendFile('log.txt', `${sensorData.steps} , ${sensorData.temp} , ${sensorData.voltage}, ${sensorData.water}\n`, function (err) {
      if (err) return console.log(err);
      console.log(sensorData);
    });
    // Send to webpage!!!
});

// Points to index.html to serve webpage
app.get('/', function(req, res){
  res.sendFile(__dirname + '/client_graph.html');
  app.get('/data', function(req, res) {
    res.sendFile(__dirname + '/log.txt');
  });
});

io.on('connection', function(client) {
	//when the server receives clicked message, do this
    client.on('clicked', function(data) {
    	  console.log("clicked")
        clickcheck = 1;
		  //send a message to ALL connected clients
		  io.emit('buttonUpdate', clickcheck);
    });
});

http.listen(3000, function() {
  console.log('running on :3000');
});
// Bind server to port and IP
var file = __dirname + '/log.txt';

io.on('connection', function(socket){
  // split data at new line
  var filedata = fs.readFileSync(file, 'utf8').toString().split("\n");
  var data = [];
    data=filedata[filedata.length-2];
    console.log(data);
    io.emit('transmit_data', data); //send data to html
    socket.on('disconnect', function(){
    });
});
server.bind(PORT, HOST);