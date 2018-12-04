var data = [20, 15, 20, 20,20, 15, 20, 20,20, 15, 20]; // demo data
var myChart = null;

console.log('in showchartjs');
/** create the chart **/
window.onload(() => {
  var labelsArr = [...Array(data.length).keys()];

  var ctx = document.getElementById("myChart");
  var myChart = new Chart(ctx, {
    type: 'radar',
    data: {
        labels: labelsArr,
        datasets: [{
            data
        }]
    },
    options: {
      scale: {
        ticks: {
          beginAtZero: true
        }
      }
    }
});

});


/**  get data from buckler listener server **/
//https://stackoverflow.com/questions/44415130/how-can-i-send-information-from-nodejs-server-to-client-side

var req = new XMLHttpRequest();
var url = '/';

req.open('GET', url, true); // set this to POST if you would like

//on load
req.addEventListener('load', () => {
  var response = this.responseText;
  var parsedResponse = JSON.parse(response); // does res has to be json?
  data = parsedResponse;
  myChart.update();
});

//on error
req.addEventListener('error', () => {
  console.log('error receiving async AJAX call');
});

req.send();
