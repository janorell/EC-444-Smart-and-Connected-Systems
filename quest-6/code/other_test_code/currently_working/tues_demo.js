// Team 7: Vindhya Kuchibhotla, Jennifer Norell, Vanessa Schuweh
// EC444, Fall 2019

// Node modules
var level = require('level');
const dgram = require('dgram');
var express = require('express');
var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);
const fs = require('fs');

var timer = 1;
var int = setInterval(increment, 1000);

function increment()
{
  timer = timer % 360 + 1;
}


increment();
var qrcode = fs.statSync("qrcode.txt")
var filesize = qrcode["size"];
if (filesize > 0)
{
  console.log("stuff in file");
}

var dbSplit = level('./enterdb', {valueEncoding: 'json'});

// Parse data in JSON format
function tryParseJson(str) {
    try {
        JSON.parse(str.toString());
    } catch (e) {
        return false;
    }
    return JSON.parse(str.toString());
}

// Port and IP for UDP
var PORT = 8082;
// IP of RPi
//var HOST = '192.168.1.108';
var HOST = '192.168.1.111'
// Create socket
var server = dgram.createSocket('udp4');
server.on('error', (err) => {
  console.log(`server error:\n${err.stack}`);
  server.close();
});
// Create server
server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

// 'Action' variables per fob to indicate whether the fob has been authorized or not
// Initialized as "not authorized"
var action = 0;

// New data from hub to be verified
var beaconID;
var splitTime;

// REC beacon: , split: MM:SS
// SEND auto: -1, fast: 1, slow: 2, left: 3, right: 4

// might be multiple signals
// use database to pick which one
server.on('message', function (message, remote) {
    console.log(remote.address + ':' + remote.port +' - ' + message);
    // Parse information from hub as JSON and extract values based on keys
    const fromHUB = tryParseJson(message);
    beaconID = fromHUB.beaconID;
    splitTime = fromHUB.splitTime;
    console.log(`split: ${splitTime}`);
    if (!(beaconID == '-1')) {
      putdb();
    }

      server.send(`${action}`,remote.port,remote.address,function(error){
        if(error){
          console.log('MEH!');
        }
        action = 0;
      });

});



// When a new client connects
io.on('connection', function(socket){

  console.log('a user connected');


  socket.on('disconnect', function(){
    console.log('user disconnected');
    dbSplit.clear();
  });
});


// Once the fob has been authorized, the successful unlocked is logged in the database and sent to the client
function putdb() {
      // Key = timestamp
      var key = parseFloat(beaconID);
      var value = [{"split": splitTime}];

      //var isValid = getData()
     // if (isValid == 1) {
        console.log('is valid');
        // stored and sent as JSON format
        dbSplit.put([key], value, function (err) {
          if (err) return console.log('Ooops!', err)
        })
        var msg = {[key]: value};
        // Send unlock entry to client
        io.emit('database', msg);
     // }
}

function getData() {
  dbSplit.get([beaconID], function (err, value) {
    if (err) {
      console.log('beacon not already present- will be added')
      return 1;
    }
    console.log('in db = id: ' + beaconID + ' time: ' + value[0].split)
    return 0;
  });
}

// SEND auto: -1, fast: 1, slow: 2, left: 3, right: 4
  io.on('fast', function(data) {
      console.log(data);
      io.emit('fastupdate', action);
      console.log("fast received");
      action = 1;


  });
  io.on('slow', function(data) {
      console.log(data);
      io.emit('slowupdate', action);
      console.log("slow received");
      action = 2;


  });
  io.on('left', function(data) {
      console.log(data);
      io.emit('leftupdate', action);
      console.log("left received");
      action = 3;


  });
  io.on('right', function(data) {
      console.log(data);
      io.emit('rightupdate', action);
      console.log("right received");
      action = 4;


  });
  io.on('auto', function(data) {
      console.log(data);
      io.emit('autoupdate', action);
      console.log("auto received");
      action = -5;

  });
  io.on('manual', function(data) {
      console.log(data);
      io.emit('manualupdate', action);
      console.log("manual received");
      action = 5;

  });

io.on('connection', function(client){

  client.on('fast', function(data) {
    console.log("fast clicked")
    action = 1;
  //send a message to ALL connected clients
    //io.emit('buttonUpdate', clickcheck);
  });
  client.on('slow', function(data) {
    console.log("slow clicked")
    action = 2;

  });
  client.on('left', function(data) {
    console.log("left clicked")
    action = 3;

  });
  client.on('right', function(data) {
    console.log("right clicked")
    action = 4;

  });
  client.on('auto', function(data) {
    console.log("auto clicked")
    action = -5;

  });
  client.on('manual', function(data) {
    console.log("manual clicked")
    action = 5;

  });
});
// Listening on localhost:4000 (web client)
http.listen(4000, function() {
  console.log('listening on *:4000');
});


// Points to demo.html to serve webpage
app.get('/', function(req, res){
  res.sendFile(__dirname + '/demo.html');
});

// Bind server to port and host
server.bind(PORT, HOST);
