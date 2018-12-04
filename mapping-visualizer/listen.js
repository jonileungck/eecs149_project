// server

var serverStr = '[Server] ';

const port = 46500;
var net = require('net');
var server = net.createServer(function(connection) {
    console.log(serverStr + 'client connected');

    connection.on('close', function (){
        console.log(serverStr + 'client disconnected');
     });

    connection.on('data', function (data) {
        data = data.toString();
        console.log(serverStr + ' data: '+ data);
        router.get('/', function(req,res) {
           // var JSONdata = JSON.stringify(dataToSendToClient);
           res.send(data);
        });
        connection.write(serverStr + "server received your string");
        connection.end();
   });

});


server.listen(port, function () {
    console.log('server is listening');
});
