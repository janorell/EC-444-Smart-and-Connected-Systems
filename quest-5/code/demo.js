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

// Authorization Database
var dbAuth = level('./authdb', {valueEncoding: 'json'});
// Authentication Database
var dbEnter = level('./enterdb', {valueEncoding: 'json'});

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
var HOST = '192.168.1.108';

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
var actionK = 0;
var actionS = 0;
var actionN = 0;

// New data from hub to be verified
var aTime;
var aHubID;
var aCode;
var aFobID;
var aName;
var aPlace;
var tempTime;

server.on('message', function (message, remote) {
    console.log(remote.address + ':' + remote.port +' - ' + message);
    // Parse information from hub as JSON and extract values based on keys
    const fromHUB = tryParseJson(message);
    aFobID = fromHUB.fobID;
    aHubID = fromHUB.hubID;
    aCode = fromHUB.code;
    // Verify that the these IDs exist in the Authorization database (who is allowed to access)
    getData();

    // Send back the specific action based on which fob ESP communicated
    if (aFobID == 1) { // code S
      server.send(`${actionS}`,'8082','192.168.1.130',function(error){
        if(error){
          console.log('MEH!');
        }
        // reset 'action'
        actionS = 0;
      });
    }
    else if (aFobID == 2) { // code K
          server.send(`${actionK}`,'8082','192.168.1.106',function(error){
            if(error){
              console.log('MEH!');
            }
            // reset 'action'
            actionK = 0;
          });
    }
    else if (aFobID == 3) { // code N
          server.send(`${actionN}`,'8082','192.168.1.136',function(error){
            if(error){
              console.log('MEH!');
            }
            // reset 'action'
            actionN = 0;
          });
    }
    else {
      // TO HUB (192.168.1.138)
      server.send(`hi`,remote.port,remote.address,function(error){
        if(error){
          console.log('MEH!');
        }
      });
    }
});

// Append data to authorize.txt in TSV format (fobID, name, hubID, code)
fs.appendFile('authorize.txt', `2\tVindhya\t7\tK\n`, function (err) {
  if (err) return console.log(err);
});
fs.appendFile('authorize.txt', `1\tVanessa\t7\tS\n`, function (err) {
  if (err) return console.log(err);
});
fs.appendFile('authorize.txt', `3\tJenni\t7\tN\n`, function (err) {
  if (err) return console.log(err);
});

// When a new client connects
io.on('connection', function(socket){
  console.log('a user connected');
  // Enter the authorize.txt information in the Authorization database
  putAuth();

  socket.on('disconnect', function(){
    console.log('user disconnected');
  });
});

var file = __dirname + '/authorize.txt';
var filedata = fs.readFileSync(file, 'utf8').toString().split("\n");

function putAuth() {
    for (var i = 1; i < filedata.length; i++) {
      var elements = filedata[i].split("\t");
      // key = fobID
      var key = elements[0];
      // encoded as JSON for accessing the values
      var value = [{Name: elements[1], hubID: elements[2], code: elements[3]}];
      // Put parsed information into the database
      dbAuth.put([key], value, function (err) {
        if (err) return console.log('Ooops!', err)
      })
  }
}

// Once the fob has been authorized, the successful unlocked is logged in the database and sent to the client
function putEnterdb(timestamp, fobID, hubID, name, place) {
      // Key = timestamp
      var key = parseFloat(timestamp);
      var value = [{"fobID": fobID, "hubID": parseFloat(hubID), "name": name, "place": place}];
      // stored and sent as JSON format
      dbEnter.put([key], value, function (err) {
        if (err) return console.log('Ooops!', err)
      })
      var msg = {[key]: value};
      // Send unlock entry to client
      io.emit('database', msg);

}

// check if information is authorized by FOB ID, and then by code
function getData() {
  dbAuth.get([aFobID], function (err, value) {
    if (err) return console.log('not authorized')
    console.log('authorized = ' + value[0].Name + ', ' + value[0].hubID)

    // timestamp of unlock attempt
    var currentdate = new Date();
    aTime = currentdate.getDate() + "/"
                    + (currentdate.getMonth()+1)  + "/"
                    + currentdate.getFullYear() + " @ "
                    + currentdate.getHours() + ":"
                    + currentdate.getMinutes() + ":"
                    + currentdate.getSeconds();

    aName = value[0].Name;
    aPlace = "EC444";

    // check code (who accessed- correct code or not)
    if (aCode == 'S') {
      console.log('writing to enter db for S');
      actionS = 1;
      console.log(`action is ${actionS}`);
      // logs to Authentication database
      putEnterdb(aTime, aFobID, aHubID, aName, aPlace)
    }
    else if (aCode == 'K') {
      console.log('writing to enter db for K');
      actionK = 1;
      console.log(`action is ${actionK}`);
      // logs to Authentication database
      putEnterdb(aTime, aFobID, aHubID, aName, aPlace)
    }
    else if (aCode == 'N') {
      console.log('writing to enter db for N');
      actionN = 1;
      console.log(`action is ${actionN}`);
      // logs to Authentication database      
      putEnterdb(aTime, aFobID, aHubID, aName, aPlace)
    }
    else {
      console.log('not authorized');
    }
  })
}

// Listening on localhost:3000 (web client)
http.listen(3000, function() {
  console.log('listening on *:3000');
});


// Points to demo.html to serve webpage
app.get('/', function(req, res){
  res.sendFile(__dirname + '/demo.html');
});

// Bind server to port and host
server.bind(PORT, HOST);
