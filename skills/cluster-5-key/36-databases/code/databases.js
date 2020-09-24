//modules
var level = require('level');
var assert = require('assert');
var express = require('express');
var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);
const fs = require('fs');
var db = level('./newdb', {valueEncoding: 'json'});
//var db = level('./newdb');
var data = [];

// Function to stream from database
function readDB(arg) {
  db.createReadStream()
    .on('data', function (data) {
      console.log(data.key, '=', data[0].value)
      // Parsed the data into a structure but don't have to ...
      // var dataIn = {[data.key]: data.value};
      var dataIn = {[data.key]: data[0].value};
      // Stream data to client
      io.emit('message', dataIn);
    })
    .on('error', function (err) {
      console.log('Oh my!', err)
    })
    .on('close', function () {
      console.log('Stream closed')
    })
    .on('end', function () {
      console.log('Stream ended')
    })
}

// When a new client connects
//var clientConnected = 0; // this is just to ensure no new data is recorded during streaming
io.on('connection', function(socket){
  console.log('a user connected');
  //clientConnected = 0;
  putFunc()
  getData();
  // readDB();
//  clientConnected = 1;
  socket.on('disconnect', function(){
    console.log('user disconnected');
  });
});

var file = __dirname + '/smoke.txt';

var filedata = fs.readFileSync(file, 'utf8').toString().split("\n");

function putFunc() {
  //if (clientConnected == 1) {
    for (var i = 1; i < filedata.length; i++) {
      console.log(filedata[i]);
      var elems = filedata[i].split("\t");
      var key = i;
      // var key = parseFloat(elements[1]);
      var value = [{time: parseFloat(elems[0]), ID: parseFloat(elems[1]), smoke: parseFloat(elems[2]),temp: parseFloat(elems[3])}];

      db.put([key], value, function (err) {
        if (err) return console.log('Ooops!', err) // some kind of I/O error
      })
      // Parse data to send to client
      var msg = {[key]: value};
      // Send to client
      io.emit('message', msg);
    //}
    // Log to console
     console.log(Object.keys(msg)); // second line on console
  }
}

function getData() {
  for (var i = 3; i < filedata.length; i=i+5) {
    var keyFind = i;
    db.get(keyFind, function (err, value) {
      if (err) return console.log('does not exist')
    //  console.log(value.length) always 1, even if multiple "IDs" (overwritten)
      console.log('ID: 3 - temperature = ' + value[0].temp)
      console.log('ID: 3 - smoke = ' + value[0].smoke)
      // console.log(typeof(value));
    })
  }
}

// Listening on localhost:3000
http.listen(3000, function() {
  console.log('listening on *:3000');
});


// Points to index.html to serve webpage
app.get('/', function(req, res){
  res.sendFile(__dirname + '/index.html');
});
