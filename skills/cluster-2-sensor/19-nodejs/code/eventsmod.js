var events = require('events');
var eventEmitter = new events.EventEmitter();

var Handler = function(){
  console.log('I hear a scream! I be testing!!');
}

eventEmitter.on('scream', Handler);
eventEmitter.emit('scream');
