var http = require('http');
var fs = require('fs');

var server = http.createServer((req, res) => {
  console.log('request made ' + req.url);
  res.writeHead(200, {'Content-Type': 'text/html'});
  var myReadStream = fs.createReadStream(__dirname + '/main.html', 'utf8');
  // res.writeHead(200, {'Content-Type': 'text/javascript'});
  myReadStream.pipe(res);
});

server.listen(3000, '127.0.0.1');
console.log('--- Localhost up ---');


/** also listen for buckler **/

// var serverStr = '[Server] ';
//
// const port = 46500;
// var net = require('net');
// var express = require('express')
// var app = express();
// var io = require('socket.io')(3000);
//
// app.use(express.static(__dirname)); //?
//
// var server = net.createServer(function(connection) {
//     console.log(serverStr + 'client connected');
//
//     connection.on('close', function (){
//         console.log(serverStr + 'client disconnected');
//      });
//
//     connection.on('data', function (data) {
//         data = data.toString();
//         var arr = data.split(',');
//         arr = arr.map((a) => parseFloat(a));
//         console.log(arr);
//
//         // app.get('/', function(req,res) {
//         //    // var JSONdata = JSON.stringify(dataToSendToClient);
//         //    res.send(data);
//         // });
//
//         io.sockets.emit('update-msg', { data: 'this is the data'});
//
//         connection.write(serverStr + "server received your string");
//         connection.end();
//    });
//
// });
//
//
// server.listen(port, function () {
//     console.log('--- server is listening for buckler ---');
// });
